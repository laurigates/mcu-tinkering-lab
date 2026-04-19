/**
 * @file piezo.h
 * @brief LEDC-PWM piezo tone generator for the Finderbox.
 *
 * Generates a square-wave tone at the specified frequency on the piezo
 * GPIO.  Calls are non-blocking: the hardware keeps oscillating after
 * @ref piezo_play_tone returns, and an esp_timer one-shot stops the
 * tone after the requested duration.
 */

#ifndef FINDERBOX_PIEZO_H
#define FINDERBOX_PIEZO_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPIO connected to the piezo buzzer. */
#define PIEZO_GPIO 2

/**
 * @brief Initialise LEDC timer + channel for the piezo.
 *
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t piezo_init(void);

/**
 * @brief Play a tone.  Non-blocking.
 *
 * A duration of 0 plays the tone until @ref piezo_stop is called.
 *
 * @param freq_hz      Frequency in hertz (200–10000 Hz recommended).
 * @param duration_ms  Tone duration in milliseconds, or 0 for sustained.
 */
void piezo_play_tone(uint32_t freq_hz, uint32_t duration_ms);

/** Immediately silence the piezo. */
void piezo_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* FINDERBOX_PIEZO_H */
