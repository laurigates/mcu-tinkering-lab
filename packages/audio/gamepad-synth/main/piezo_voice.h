/**
 * @file piezo_voice.h
 * @brief GPIO-driven piezo voices using LEDC square-wave generation.
 *
 * Each voice owns its own LEDC timer so frequencies are independent.
 * Suitable as an accent layer alongside the main I2S DAC output.
 */

#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIEZO_A = 0,
    PIEZO_B,
    PIEZO_COUNT,
} piezo_id_t;

/** Initialize one piezo voice on the given GPIO. Safe to call once per id. */
void piezo_voice_init(piezo_id_t id, gpio_num_t gpio);

/** Start (or retune) the voice at freq_hz. Clamped to the LEDC-reachable range. */
void piezo_voice_note_on(piezo_id_t id, float freq_hz);

/** Silence the voice. Idempotent. */
void piezo_voice_note_off(piezo_id_t id);

#ifdef __cplusplus
}
#endif
