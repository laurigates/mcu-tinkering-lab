/**
 * @file standalone_mode.c
 * @brief Button, piezo, and WS2812 status LED driver for the Brainbox.
 *
 * GPIO assignments:
 *   GPIO 4  — WS2812 single-pixel status LED (led_strip, DMA disabled)
 *   GPIO 5  — Piezo buzzer (LEDC channel 0, timer 0, 50 % duty cap)
 *   GPIO 9  — Tactile button (pull-up, active LOW, FALLING edge)
 *
 * Button tap behaviour:
 *   1. 200 ms tone at 880 Hz on piezo (duty ≤ 50 % — LEDC 13-bit: 4095/2 ≈ 2047)
 *   2. WS2812 advances through palette: red → amber → green → blue → violet
 */

#include "standalone_mode.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "standalone_mode";

/* ------------------------------------------------------------------ */
/* Pin and LEDC configuration                                          */
/* ------------------------------------------------------------------ */

#define BTN_GPIO ((gpio_num_t)9)
#define PIEZO_GPIO ((gpio_num_t)5)
#define LED_GPIO ((gpio_num_t)4)

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT /**< 13-bit: 0–8191 */
#define PIEZO_FREQ_HZ 880u
#define PIEZO_DURATION_MS 200u
#define PIEZO_MAX_DUTY 4095u /**< ~50 % of 8191 */

/* ------------------------------------------------------------------ */
/* Status LED palette                                                  */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static const rgb_t s_palette[] = {
    {153, 0, 0},   /* red    */
    {153, 80, 0},  /* amber  */
    {0, 153, 0},   /* green  */
    {0, 0, 153},   /* blue   */
    {100, 0, 153}, /* violet */
};

#define PALETTE_SIZE ((int)(sizeof(s_palette) / sizeof(s_palette[0])))

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

static led_strip_handle_t s_led_strip = NULL;
static int s_palette_idx = 0;
static uint32_t s_piezo_stop_ms = 0; /**< When to silence the piezo (0 = silent) */
static bool s_ledc_running = false;

/* ------------------------------------------------------------------ */
/* LED helpers                                                         */
/* ------------------------------------------------------------------ */

static void led_show_palette(int idx)
{
    if (!s_led_strip) {
        return;
    }
    const rgb_t *c = &s_palette[idx % PALETTE_SIZE];
    esp_err_t err = led_strip_set_pixel(s_led_strip, 0, c->r, c->g, c->b);
    if (err == ESP_OK) {
        led_strip_refresh(s_led_strip);
    } else {
        ESP_LOGW(TAG, "led_strip_set_pixel failed: %s", esp_err_to_name(err));
    }
}

/* ------------------------------------------------------------------ */
/* Piezo helpers                                                       */
/* ------------------------------------------------------------------ */

static void piezo_start(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, PIEZO_MAX_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    s_ledc_running = true;
}

static void piezo_stop(void)
{
    if (!s_ledc_running) {
        return;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    s_ledc_running = false;
}

/* ------------------------------------------------------------------ */
/* Button ISR                                                          */
/* ------------------------------------------------------------------ */

static QueueHandle_t s_btn_queue = NULL;

static void IRAM_ATTR btn_isr_handler(void *arg)
{
    uint8_t val = 1;
    xQueueSendFromISR(s_btn_queue, &val, NULL);
}

/* ------------------------------------------------------------------ */
/* Button task                                                         */
/* ------------------------------------------------------------------ */

static void btn_task(void *pvParam)
{
    uint8_t val;
    for (;;) {
        if (xQueueReceive(s_btn_queue, &val, portMAX_DELAY) == pdTRUE) {
            standalone_mode_on_button();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t standalone_mode_init(void)
{
    esp_err_t ret;

    /* --- WS2812 status LED --- */
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = (int)LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false},
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, /* 10 MHz */
        .mem_block_symbols = 48,
        .flags = {.with_dma = false},
    };
    ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    led_strip_clear(s_led_strip);
    led_show_palette(s_palette_idx);
    ESP_LOGI(TAG, "WS2812 status LED on GPIO %d", LED_GPIO);

    /* --- LEDC piezo --- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = PIEZO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ledc_channel_config_t ch_cfg = {
        .gpio_num = (int)PIEZO_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Piezo on GPIO %d (LEDC channel %d, %u Hz)", PIEZO_GPIO, LEDC_CHANNEL,
             PIEZO_FREQ_HZ);

    /* --- Button GPIO + ISR --- */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ret = gpio_config(&btn_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config (button) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_btn_queue = xQueueCreate(4, sizeof(uint8_t));
    if (!s_btn_queue) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return ESP_ERR_NO_MEM;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means ISR service already installed — OK */
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    xTaskCreatePinnedToCore(btn_task, "btn_task", 2048, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "Button on GPIO %d (pull-up, active LOW)", BTN_GPIO);
    return ESP_OK;
}

void standalone_mode_on_button(void)
{
    /* Advance palette */
    s_palette_idx = (s_palette_idx + 1) % PALETTE_SIZE;
    led_show_palette(s_palette_idx);

    /* Chime: 880 Hz for PIEZO_DURATION_MS */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_piezo_stop_ms = now_ms + PIEZO_DURATION_MS;
    piezo_start();

    ESP_LOGI(TAG, "Button pressed — palette idx %d, chime started", s_palette_idx);
}

void standalone_mode_tick(uint32_t now_ms)
{
    if (s_ledc_running && s_piezo_stop_ms > 0 && now_ms >= s_piezo_stop_ms) {
        piezo_stop();
        s_piezo_stop_ms = 0;
    }
}
