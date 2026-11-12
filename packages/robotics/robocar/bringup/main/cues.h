/**
 * @file cues.h
 * @brief Audible result vocabulary for the bringup sweep.
 *
 * The sweep has to be readable with no tether. Both serial monitors in use here
 * (`just robocar-bringup::monitor` and the `serial-monitor` helper) are
 * READ-ONLY and reset the board on attach, so a console-driven design would
 * mean unplugging the soldering iron to type. The buzzer is therefore the
 * primary output, not a decoration.
 *
 * The buzzer is also the only indicator that works before anything is soldered:
 * it is on a dedicated GPIO (PIEZO_PIN), while the LEDs and the OLED both sit
 * behind the I2C multiplexer. On a board with nothing but power and a piezo
 * fitted, these tones are the entire user interface.
 *
 * Cue design rules, so a fault is distinguishable by ear alone:
 *   - PITCH carries the verdict. High = good, low = bad. Never rely on rhythm
 *     alone, which is what a listener loses track of across fourteen checks.
 *   - LENGTH carries severity. A pass is a blip; a failure is long enough to
 *     interrupt what you are doing.
 *   - SKIP is deliberately quiet and mid-range. "Not fitted yet" is the normal
 *     state through most of a build and must not sound like a fault, or the
 *     whole sweep reads as broken while you are halfway through soldering.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CUE_SWEEP_START, /**< Two rising notes: the sweep has begun. */
    CUE_PASS,        /**< One short high blip. */
    CUE_WARN,        /**< Two mid blips: present, but the reading looks off. */
    CUE_SKIP,        /**< One short mid-low blip: hardware not fitted. */
    CUE_FAIL,        /**< Two long low notes. */
    CUE_MOTOR_ARMED, /**< Three urgent notes before the wheels are driven. */
    CUE_ALL_PASS,    /**< Rising triad. Nothing failed. */
    CUE_ANY_FAIL,    /**< Falling triad. At least one check failed. */
} cue_t;

/** Play @p cue on the piezo. Blocking, and never longer than ~600 ms. */
void cues_play(cue_t cue);

/**
 * @brief Play a fixed test tone sequence through the MAX98357A.
 *
 * Locally generated — a sine, an octave above it, and a 200 Hz to 2 kHz sweep —
 * so the amplifier path can be judged with no network, no API key and no cost.
 *
 * This is the control the TTS path never had. A synthesised sine exercises
 * exactly the same I2S channel, DMA descriptors, ring buffer and amplifier as
 * a spoken sentence, but with none of the streaming: the samples are produced
 * far faster than real time, so the ring cannot run dry and the DMA cannot
 * tear. Clean here but garbled on speech puts the fault upstream of I2S, in the
 * fetch or the decode. Garbled here too puts it in the amplifier, its supply,
 * or the wiring — which is the discriminator you want before reaching for a
 * scope.
 *
 * Phase is carried across the tones so the joins do not click, because a click
 * is indistinguishable from the defect being listened for.
 *
 * @return ESP_OK once the whole sequence has been queued and played out.
 */
esp_err_t cues_amp_tone_sequence(void);

#ifdef __cplusplus
}
#endif
