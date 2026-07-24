/**
 * @file ultrasonic.c
 * @brief HC-SR04P / RCWL-1601 / US-100 ultrasonic rangefinder driver.
 *
 * Implementation strategy
 * -----------------------
 * Primary:  ESP-IDF v5.4 RMT RX peripheral.  rmt_new_rx_channel() claims a
 *           hardware channel.  A single rmt_receive() call is issued after each
 *           TRIG pulse; the callback fills a one-item ring buffer and posts a
 *           FreeRTOS task notification so ultrasonic_measure() can block with a
 *           timeout instead of spinning.
 *
 * Fallback: If RMT initialisation fails (all channels claimed) the driver
 *           switches to a gpio_get_level() + esp_timer_get_time() polling loop.
 *           This is less accurate at long ranges but functional.
 *
 * Speed-of-sound conversion
 * -------------------------
 *   distance_cm = pulse_us / 58
 *
 * This approximation holds within ±1 % at 20 °C sea-level.  Good enough for a
 * 15 cm obstacle threshold.
 */

#include "ultrasonic.h"

#ifndef ULTRASONIC_HOST_TEST

#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "ultrasonic";

/* -------------------------------------------------------------------------
 * RMT configuration constants
 * ---------------------------------------------------------------------- */

/** RMT clock resolution: 1 µs per tick (1 MHz). */
#define RMT_RESOLUTION_HZ 1000000U

/**
 * RMT RX memory-block size in symbols.
 *
 * We only ever need one echo pulse (echo HIGH + trailing LOW = 2 symbols), but
 * ESP-IDF v5.4's rmt_new_rx_channel() requires mem_block_symbols to be even and
 * at least one channel memory block — 48 on the ESP32-S3
 * (SOC_RMT_MEM_WORDS_PER_CHANNEL). A smaller value (this was 4) fails init with
 * ESP_ERR_INVALID_ARG ("mem_block_symbols must be even and at least 48"), which
 * silently drops the driver into the slower GPIO polling fallback on every boot.
 * 48 = exactly one memory block; the same constant sizes s_rx_buf (192 bytes).
 */
#define RMT_RX_BUFFER_SYMBOLS 48U

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */

typedef enum {
    DRIVER_UNINIT = 0,
    DRIVER_RMT,  /* RMT RX path is active       */
    DRIVER_POLL, /* GPIO polling fallback active */
} driver_mode_t;

static driver_mode_t s_mode = DRIVER_UNINIT;

/* RMT path state */
static rmt_channel_handle_t s_rx_chan = NULL;
static rmt_symbol_word_t s_rx_buf[RMT_RX_BUFFER_SYMBOLS];
static TaskHandle_t s_waiting_task = NULL; /* task blocked in ultrasonic_measure */
static uint32_t s_echo_us = 0;             /* result written by the RX callback   */

/* -------------------------------------------------------------------------
 * Host-test injection (on-target: value is ignored)
 * ---------------------------------------------------------------------- */
static uint16_t s_injected_distance = ULTRASONIC_DIST_ERROR;

void ultrasonic_test_set_distance(uint16_t cm)
{
    /* No-op on real hardware — the RMT always wins. */
    (void)cm;
}

/* -------------------------------------------------------------------------
 * RMT RX callback (runs in ISR context)
 * ---------------------------------------------------------------------- */

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel,
                                     const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t higher_priority_woken = pdFALSE;

    /* edata->received_symbols[0] is the echo HIGH pulse.
     * duration0 and duration1 together encode the high + low durations in
     * the rmt_symbol_word_t union; duration0 is the FIRST level duration. */
    if (edata->num_symbols > 0) {
        /* The echo signal level is HIGH for the distance duration. */
        s_echo_us = edata->received_symbols[0].duration0;
    } else {
        s_echo_us = 0;
    }

    if (s_waiting_task != NULL) {
        vTaskNotifyGiveFromISR(s_waiting_task, &higher_priority_woken);
        s_waiting_task = NULL;
    }

    return higher_priority_woken == pdTRUE;
}

/* -------------------------------------------------------------------------
 * Trigger pulse helper
 * ---------------------------------------------------------------------- */

static void send_trig_pulse(void)
{
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    /* 10 µs hold — esp_rom_delay_us is accurate and safe in task context */
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ultrasonic_init(void)
{
    if (s_mode != DRIVER_UNINIT) {
        return ESP_OK; /* idempotent */
    }

    /* --- Configure TRIG GPIO --- */
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << ULTRASONIC_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&trig_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TRIG GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    /* --- Attempt RMT RX channel --- */
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = ULTRASONIC_ECHO_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ, /* 1 µs / tick */
        .mem_block_symbols = RMT_RX_BUFFER_SYMBOLS,
        .flags.invert_in = false,
        .flags.with_dma = false,
        .flags.io_loop_back = false,
    };

    ret = rmt_new_rx_channel(&rx_cfg, &s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "RMT RX channel unavailable (%s) — falling back to GPIO poll",
                 esp_err_to_name(ret));

        /* Configure ECHO as plain input for polling fallback */
        gpio_config_t echo_conf = {
            .pin_bit_mask = (1ULL << ULTRASONIC_ECHO_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&echo_conf));
        s_mode = DRIVER_POLL;
        ESP_LOGI(TAG, "Ultrasonic driver ready (GPIO poll mode)");
        return ESP_OK;
    }

    /* Register RX-done callback */
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_cb,
    };
    ret = rmt_rx_register_event_callbacks(s_rx_chan, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT callback registration failed: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return ret;
    }

    ret = rmt_enable(s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return ret;
    }

    s_mode = DRIVER_RMT;
    ESP_LOGI(TAG, "Ultrasonic driver ready (RMT RX mode, TRIG=GPIO%d ECHO=GPIO%d)",
             ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    return ESP_OK;
}

esp_err_t ultrasonic_measure(uint16_t *distance_cm)
{
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mode == DRIVER_UNINIT) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ESP_ERR_INVALID_STATE;
    }

    /* Minimum 60 µs between consecutive TRIG pulses per HC-SR04P datasheet. */
    esp_rom_delay_us(60);

    /* ---- RMT path ---- */
    if (s_mode == DRIVER_RMT) {
        /*
         * Configure the RMT receive window. Both bounds are constrained by the
         * hardware register widths (ESP-IDF v5.4 rmt_receive validates them):
         *
         *   - signal_range_min_ns (glitch filter): counted on the RMT SOURCE
         *     clock (~80 MHz), whose threshold register is 8-bit (max 255). So
         *     min_ns must be <= ~3187 ns; the previous 10 000 ns overflowed it
         *     (800 > 255 -> "signal_range_min_ns too big"). 1 µs still rejects
         *     electrical glitches and passes any real echo (>= 58 µs = 1 cm).
         *   - signal_range_max_ns (idle threshold): counted on the 1 MHz channel
         *     resolution, register max 32767 -> max_ns must be <= 32.767 ms; the
         *     previous 38 ms overflowed it. 30 ms comfortably covers the 4 m
         *     (~23.2 ms) max echo (ULTRASONIC_MAX_RANGE_CM) with headroom.
         */
        rmt_receive_config_t recv_cfg = {
            .signal_range_min_ns = 1000U,     /* 1 µs glitch filter (hw 8-bit @ source clk) */
            .signal_range_max_ns = 30000000U, /* 30 ms idle (hw cap 32.767 ms @ 1 MHz)      */
        };

        s_echo_us = 0;
        s_waiting_task = xTaskGetCurrentTaskHandle();

        esp_err_t ret = rmt_receive(s_rx_chan, s_rx_buf, sizeof(s_rx_buf), &recv_cfg);
        if (ret != ESP_OK) {
            s_waiting_task = NULL;
            ESP_LOGE(TAG, "rmt_receive failed: %s", esp_err_to_name(ret));
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ESP_FAIL;
        }

        send_trig_pulse();

        /* Block until callback fires or timeout (ULTRASONIC_ECHO_TIMEOUT_US µs) */
        const TickType_t timeout_ticks = pdMS_TO_TICKS((ULTRASONIC_ECHO_TIMEOUT_US / 1000U) + 5U);
        uint32_t notified = ulTaskNotifyTake(pdTRUE, timeout_ticks);

        if (notified == 0) {
            /* Timeout — cancel any pending receive */
            s_waiting_task = NULL;
            ESP_LOGD(TAG, "Echo timeout");
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ESP_FAIL;
        }

        if (s_echo_us == 0) {
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ESP_FAIL;
        }

        /* Convert: distance_cm = pulse_us / 58 */
        uint32_t dist = s_echo_us / 58U;
        *distance_cm =
            (dist > ULTRASONIC_MAX_RANGE_CM) ? (uint16_t)ULTRASONIC_MAX_RANGE_CM : (uint16_t)dist;
        return ESP_OK;
    }

    /* ---- GPIO polling fallback ---- */
    send_trig_pulse();

    int64_t t_start = esp_timer_get_time();

    /* Wait for ECHO to go HIGH */
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - t_start > ULTRASONIC_ECHO_TIMEOUT_US) {
            ESP_LOGD(TAG, "Echo timeout (poll: waiting HIGH)");
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ESP_FAIL;
        }
    }

    int64_t t_high = esp_timer_get_time();

    /* Measure HIGH duration */
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - t_high > ULTRASONIC_ECHO_TIMEOUT_US) {
            ESP_LOGD(TAG, "Echo timeout (poll: measuring HIGH)");
            *distance_cm = ULTRASONIC_DIST_ERROR;
            return ESP_FAIL;
        }
    }

    int64_t pulse_us = esp_timer_get_time() - t_high;
    uint32_t dist = (uint32_t)pulse_us / 58U;
    *distance_cm =
        (dist > ULTRASONIC_MAX_RANGE_CM) ? (uint16_t)ULTRASONIC_MAX_RANGE_CM : (uint16_t)dist;
    return ESP_OK;
}

esp_err_t ultrasonic_deinit(void)
{
    if (s_mode == DRIVER_RMT && s_rx_chan != NULL) {
        rmt_disable(s_rx_chan);
        rmt_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    s_mode = DRIVER_UNINIT;
    return ESP_OK;
}

/* =========================================================================
 * Host-test build (ULTRASONIC_HOST_TEST=1) — entire stub implementation
 * ========================================================================= */
#else /* ULTRASONIC_HOST_TEST */

#include <stddef.h>
#include <stdint.h>

static uint16_t s_injected_distance = ULTRASONIC_DIST_ERROR;
static int s_initialized = 0;

void ultrasonic_test_set_distance(uint16_t cm)
{
    s_injected_distance = cm;
}

esp_err_t ultrasonic_init(void)
{
    s_initialized = 1;
    return ESP_OK;
}

esp_err_t ultrasonic_measure(uint16_t *distance_cm)
{
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        *distance_cm = ULTRASONIC_DIST_ERROR;
        return ESP_ERR_INVALID_STATE;
    }
    *distance_cm = s_injected_distance;
    return (s_injected_distance == ULTRASONIC_DIST_ERROR) ? ESP_FAIL : ESP_OK;
}

esp_err_t ultrasonic_deinit(void)
{
    s_initialized = 0;
    return ESP_OK;
}

#endif /* ULTRASONIC_HOST_TEST */
