/**
 * @file buzzer.h
 * @brief Piezo buzzer control (GPIO toggle tone generation)
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t buzzer_init(void);

/**
 * @brief Whether buzzer_init() succeeded.
 *
 * The tone routines already no-op while uninitialised; this is how self_report
 * names the buzzer on a board that came up without it (issue #500). Mirrors
 * led_is_initialized() / servo_is_initialized() / motor_is_initialized().
 */
bool buzzer_is_initialized(void);
void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms);
void buzzer_beep(void);
void buzzer_melody(void);
void buzzer_alert(void);

#endif  // BUZZER_H
