/**
 * @file buzzer.h
 * @brief Piezo buzzer control (GPIO toggle tone generation)
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "esp_err.h"

esp_err_t buzzer_init(void);
void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms);
void buzzer_beep(void);
void buzzer_melody(void);
void buzzer_alert(void);

#endif  // BUZZER_H
