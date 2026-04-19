/**
 * @file led_ring.h
 * @brief WS2812B LED ring driver with toddler-safe brightness cap.
 *
 * Wraps espressif/led_strip with RMT backend. All RGB values are scaled
 * to ≤60% brightness (BRIGHTNESS_SCALE = 153/255) before writing to the
 * hardware, enforcing PRD FR-T21.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef LED_RING_H
#define LED_RING_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPIO connected to the WS2812B data line. */
#define LED_RING_GPIO 4

/** Number of LEDs in the ring (8 or 12; compile-time constant). */
#define LED_RING_COUNT 12

/**
 * @brief Brightness scale factor (153/255 ≈ 60%).
 *
 * Applied to every channel before writing to hardware. Enforces the
 * toddler safety constraint from PRD FR-T21.
 */
#define LED_RING_BRIGHTNESS_SCALE 153

/**
 * @brief Initialise the RMT-based LED ring driver.
 *
 * Must be called once before any other led_ring_* function.
 *
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t led_ring_init(void);

/**
 * @brief Set a single pixel's colour (brightness-capped).
 *
 * The raw r, g, b values are scaled by BRIGHTNESS_SCALE/255 before being
 * written to the strip. Does not refresh the hardware — call led_ring_show().
 *
 * @param index  Pixel index (0 … LED_RING_COUNT-1).
 * @param r      Red channel (0-255, full scale before capping).
 * @param g      Green channel (0-255, full scale before capping).
 * @param b      Blue channel (0-255, full scale before capping).
 */
void led_ring_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set all pixels to the same colour (brightness-capped).
 *
 * Does not refresh the hardware — call led_ring_show().
 */
void led_ring_fill(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Clear all pixels (set to off).
 *
 * Does not refresh the hardware — call led_ring_show().
 */
void led_ring_clear(void);

/**
 * @brief Flush pending pixel data to the hardware.
 *
 * Must be called after led_ring_set_pixel() / led_ring_fill() / led_ring_clear()
 * for changes to be visible.
 *
 * @return ESP_OK on success.
 */
esp_err_t led_ring_show(void);

/**
 * @brief Convert HSV to RGB and set a pixel (brightness-capped).
 *
 * Convenience wrapper that performs HSV→RGB conversion in software then
 * calls led_ring_set_pixel(). Does not refresh the hardware.
 *
 * @param index  Pixel index (0 … LED_RING_COUNT-1).
 * @param h      Hue (0-255, wraps).
 * @param s      Saturation (0-255).
 * @param v      Value / brightness (0-255, additionally capped by scale).
 */
void led_ring_set_pixel_hsv(uint32_t index, uint8_t h, uint8_t s, uint8_t v);

#ifdef __cplusplus
}
#endif

#endif /* LED_RING_H */
