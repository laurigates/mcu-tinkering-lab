/**
 * @file status_led_core.c
 * @brief Shared WS2812 status LED layer using the RMT-based led_strip.
 *
 * Not an ESP-IDF component of its own (no CMakeLists.txt): each consumer's
 * status_led component compiles this file, so the builds that take every
 * directory under packages/components (robocar, the thinkpack boxes) neither
 * compile it nor pull in led_strip. See README.md.
 */
#include "status_led_core.h"

#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

static const char *TAG = "status_led";

static led_strip_handle_t s_strip;
static const status_led_pattern_t *s_patterns;
static size_t s_count;
static volatile unsigned s_mode;
static bool s_blink_on = false;
static int64_t s_last_toggle_us = 0;

static void set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

esp_err_t status_led_core_init(int gpio, const status_led_pattern_t *patterns, size_t count)
{
    s_patterns = patterns;
    s_count = count;

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48, /* ESP32-S3 has 48 symbols per block, not 64 */
        .flags.with_dma = false, /* DMA breaks after TinyUSB init */
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", gpio);
    return ESP_OK;
}

void status_led_core_set_mode(unsigned mode)
{
    s_mode = mode;
}

void status_led_core_update(void)
{
    unsigned mode = s_mode;
    if (s_patterns == NULL || mode >= s_count) {
        set_color(0, 0, 0);
        return;
    }

    const status_led_pattern_t *p = &s_patterns[mode];
    if (p->blink_us == 0) {
        set_color(p->r, p->g, p->b);
        return;
    }

    int64_t now = esp_timer_get_time();
    if ((now - s_last_toggle_us) >= (int64_t)p->blink_us) {
        s_blink_on = !s_blink_on;
        s_last_toggle_us = now;
    }
    if (s_blink_on) {
        set_color(p->r, p->g, p->b);
    } else {
        set_color(0, 0, 0);
    }
}

void status_led_core_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    set_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    set_color(0, 0, 0);
}
