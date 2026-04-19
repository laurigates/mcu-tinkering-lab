/**
 * @file led_single.c
 * @brief Single WS2812 LED on GPIO 4 — brightness-capped RMT driver.
 *
 * Brightness cap: each channel is scaled to ≤ 60 % of the requested
 * value before writing to the strip (FR-T21).
 * DMA disabled, mem_block_symbols=48 for ESP32-S3 RMT compatibility.
 */

#include "led_single.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "led_single";

#define LED_GPIO 4
#define LED_COUNT 1

/* 60 % brightness cap: multiply by 3, divide by 5 */
#define BRIGHTNESS_NUM 3u
#define BRIGHTNESS_DEN 5u

static led_strip_handle_t s_strip;
static bool s_flash_active;
static uint32_t s_flash_end_ms;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static uint8_t cap_brightness(uint8_t v)
{
    return (uint8_t)((uint32_t)v * BRIGHTNESS_NUM / BRIGHTNESS_DEN);
}

static void write_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_strip, 0, cap_brightness(r), cap_brightness(g), cap_brightness(b));
    led_strip_refresh(s_strip);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t led_single_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48, /* ESP32-S3: 48 symbols per block, not 64 */
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);
    s_flash_active = false;
    s_flash_end_ms = 0;

    ESP_LOGI(TAG, "Single LED initialized on GPIO%d (60%% brightness cap)", LED_GPIO);
    return ESP_OK;
}

void led_single_set(uint8_t r, uint8_t g, uint8_t b)
{
    s_flash_active = false;
    write_pixel(r, g, b);
}

void led_single_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    write_pixel(r, g, b);
    s_flash_active = true;
    /* Compute end time from current wall-clock ms */
    s_flash_end_ms = (uint32_t)(esp_timer_get_time() / 1000) + duration_ms;
}

void led_single_tick(uint32_t now_ms)
{
    if (s_flash_active && now_ms >= s_flash_end_ms) {
        s_flash_active = false;
        led_strip_clear(s_strip);
        led_strip_refresh(s_strip);
    }
}

void led_single_clear(void)
{
    s_flash_active = false;
    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);
}
