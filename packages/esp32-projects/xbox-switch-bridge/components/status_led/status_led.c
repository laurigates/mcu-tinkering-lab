/**
 * @file status_led.c
 * @brief WS2812 status LED driver using RMT-based led_strip.
 *
 * No background task — call status_led_update() from an existing loop.
 * DMA is disabled to avoid conflicts with TinyUSB on ESP32-S3.
 */
#include "status_led.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

static const char *TAG = "status_led";

#define LED_GPIO 21
#define LED_COUNT 1
#define LED_BRIGHTNESS 32
#define BLINK_SLOW_US (500 * 1000) /* 500ms per phase (1 Hz) */
#define BLINK_FAST_US (200 * 1000) /* 200ms per phase (2.5 Hz) */

static led_strip_handle_t s_strip;
static volatile status_led_mode_t s_mode = STATUS_LED_OFF;
static bool s_blink_on = false;
static int64_t s_last_toggle_us = 0;

static void set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

/**
 * @brief Update blink state and return whether the LED should be "on".
 */
static bool update_blink(int64_t now, int64_t period_us)
{
    if ((now - s_last_toggle_us) >= period_us) {
        s_blink_on = !s_blink_on;
        s_last_toggle_us = now;
    }
    return s_blink_on;
}

esp_err_t status_led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48, /* ESP32-S3 has 48 symbols per block, not 64 */
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", LED_GPIO);
    return ESP_OK;
}

void status_led_set_mode(status_led_mode_t mode)
{
    s_mode = mode;
}

void status_led_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    set_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    set_color(0, 0, 0);
}

void status_led_update(void)
{
    int64_t now = esp_timer_get_time();

    switch (s_mode) {
        case STATUS_LED_SCANNING: /* Blue blink (slow) */
            if (update_blink(now, BLINK_SLOW_US)) {
                set_color(0, 0, LED_BRIGHTNESS);
            } else {
                set_color(0, 0, 0);
            }
            break;

        case STATUS_LED_USB_ERROR: /* Red solid */
            set_color(LED_BRIGHTNESS, 0, 0);
            break;

        case STATUS_LED_CONNECTED_NO_USB: /* Purple blink (slow) */
            if (update_blink(now, BLINK_SLOW_US)) {
                set_color(LED_BRIGHTNESS, 0, LED_BRIGHTNESS);
            } else {
                set_color(0, 0, 0);
            }
            break;

        case STATUS_LED_CONNECTED_USB: /* Yellow blink (fast) */
            if (update_blink(now, BLINK_FAST_US)) {
                set_color(LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
            } else {
                set_color(0, 0, 0);
            }
            break;

        case STATUS_LED_BRIDGING: /* Green solid */
            set_color(0, LED_BRIGHTNESS, 0);
            break;

        case STATUS_LED_OFF:
        default:
            set_color(0, 0, 0);
            break;
    }
}
