/**
 * @file led_ring.c
 * @brief WS2812B LED ring driver with toddler-safe brightness cap.
 *
 * Uses espressif/led_strip with RMT backend. DMA is disabled to avoid
 * known conflicts on ESP32-S3 (mem_block_symbols=48, with_dma=false).
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "led_ring.h"

#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "led_ring";

static led_strip_handle_t s_strip;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * Apply brightness cap: scale = BRIGHTNESS_SCALE / 255.
 * Integer arithmetic avoids floating-point in the hot path.
 */
static inline uint8_t cap(uint8_t v)
{
    return (uint8_t)(((uint16_t)v * LED_RING_BRIGHTNESS_SCALE) / 255u);
}

/**
 * Fast 8-bit HSV → RGB (no floating-point).
 * Algorithm: https://web.mit.edu/6.111/www/f2017/handouts/1_colors.pdf
 * h,s,v are all 0-255.
 */
static void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0) {
        *r = v;
        *g = v;
        *b = v;
        return;
    }

    uint8_t region = h / 43;
    uint8_t remainder = (h - (region * 43)) * 6;

    uint8_t p = (uint8_t)(((uint16_t)v * (255u - s)) >> 8);
    uint8_t q = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * (255u - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t led_ring_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_RING_GPIO,
        .max_leds = LED_RING_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 48, /* ESP32-S3: 48 symbols per block, not 64 */
        .flags.with_dma = false, /* Avoid conflicts with WiFi DMA on S3 */
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);

    ESP_LOGI(TAG, "LED ring initialized: GPIO%d, %d LEDs, brightness cap %d/255", LED_RING_GPIO,
             LED_RING_COUNT, LED_RING_BRIGHTNESS_SCALE);
    return ESP_OK;
}

void led_ring_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LED_RING_COUNT) {
        return;
    }
    led_strip_set_pixel(s_strip, index, cap(r), cap(g), cap(b));
}

void led_ring_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        led_strip_set_pixel(s_strip, i, cap(r), cap(g), cap(b));
    }
}

void led_ring_clear(void)
{
    led_strip_clear(s_strip);
}

esp_err_t led_ring_show(void)
{
    return led_strip_refresh(s_strip);
}

void led_ring_set_pixel_hsv(uint32_t index, uint8_t h, uint8_t s, uint8_t v)
{
    uint8_t r, g, b;
    hsv_to_rgb(h, s, v, &r, &g, &b);
    led_ring_set_pixel(index, r, g, b);
}
