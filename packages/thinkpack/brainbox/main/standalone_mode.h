/**
 * @file standalone_mode.h
 * @brief Local-peripheral driver for the ThinkPack Brainbox (button, piezo, status LED).
 *
 * Handles the physical interaction layer when the Brainbox is operating without
 * a full mesh group.  A button tap triggers a piezo chime and cycles the
 * single WS2812 status LED through a colour palette.
 *
 * GPIO assignments:
 *   - GPIO 4  — WS2812 status LED (single pixel, led_strip managed component)
 *   - GPIO 5  — Piezo buzzer (LEDC channel 0, timer 0)
 *   - GPIO 9  — Tactile button (internal pull-up, active LOW, FALLING edge ISR)
 */

#ifndef STANDALONE_MODE_H
#define STANDALONE_MODE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise GPIO, LEDC, and the WS2812 LED strip driver.
 *
 * Must be called once from app_main() before thinkpack_mesh_start().
 *
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t standalone_mode_init(void);

/**
 * @brief Handle a button press event.
 *
 * Plays a 200 ms 880 Hz tone on the piezo and advances the WS2812 status LED
 * to the next colour in the palette (red → amber → green → blue → violet → …).
 *
 * Safe to call from an ISR via a deferred task notification.  In practice,
 * the ISR posts to a queue and a task calls this function.
 */
void standalone_mode_on_button(void);

/**
 * @brief Periodic tick — drives transient LED and buzzer effects.
 *
 * Call from the main status task at regular intervals (e.g. every 50 ms).
 * Turns off the piezo once the tone duration has elapsed.
 *
 * @param now_ms  Current time in milliseconds (e.g. esp_timer_get_time() / 1000).
 */
void standalone_mode_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STANDALONE_MODE_H */
