/**
 * @file led_ring.h
 * @brief WS2812B LED ring driver with toddler-safe brightness cap.
 *
 * Copied verbatim from packages/thinkpack/glowbug/main/led_ring.h — same
 * brightness cap (PRD FR-T21), same API surface.  See the glowbug copy for
 * any updates.
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

esp_err_t led_ring_init(void);
void led_ring_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
void led_ring_fill(uint8_t r, uint8_t g, uint8_t b);
void led_ring_clear(void);
esp_err_t led_ring_show(void);
void led_ring_set_pixel_hsv(uint32_t index, uint8_t h, uint8_t s, uint8_t v);

#ifdef __cplusplus
}
#endif

#endif /* LED_RING_H */
