/**
 * @file led_single.h
 * @brief Single WS2812 LED driver on GPIO 4 with brightness cap.
 *
 * Brightness is capped at 60 % of the requested value (FR-T21 toddler
 * safety). Uses RMT-based led_strip with DMA disabled and 48 symbol
 * blocks for ESP32-S3 compatibility.
 *
 * led_single_flash() sets a colour for a fixed duration then clears it.
 * Service led_single_tick() every control loop iteration (≤ 10 ms) to
 * drive the flash timeout.
 */

#ifndef LED_SINGLE_H
#define LED_SINGLE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the WS2812 RMT driver.
 *
 * @return ESP_OK on success.
 */
esp_err_t led_single_init(void);

/**
 * @brief Set the LED to a colour immediately (brightness-capped).
 *
 * @param r  Red   [0, 255] — output capped to 60 %.
 * @param g  Green [0, 255] — output capped to 60 %.
 * @param b  Blue  [0, 255] — output capped to 60 %.
 */
void led_single_set(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Flash the LED for @p duration_ms milliseconds then clear.
 *
 * Non-blocking. Call led_single_tick() every loop iteration to drive
 * the timeout. A subsequent flash call resets the timer.
 *
 * @param r            Red   [0, 255].
 * @param g            Green [0, 255].
 * @param b            Blue  [0, 255].
 * @param duration_ms  How long to stay lit before clearing.
 */
void led_single_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);

/**
 * @brief Service pending flash timeouts. Call every control loop tick.
 *
 * @param now_ms  Current time in milliseconds.
 */
void led_single_tick(uint32_t now_ms);

/**
 * @brief Turn the LED off immediately.
 */
void led_single_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_SINGLE_H */
