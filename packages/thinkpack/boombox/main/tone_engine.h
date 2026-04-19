/**
 * @file tone_engine.h
 * @brief LEDC PWM tone engine for passive piezo buzzer.
 *
 * Volume cap: duty cycle is limited to 1<<12 (50% of 13-bit resolution)
 * for toddler safety (FR-T21). Frequency is clamped to [100, 3500] Hz.
 *
 * GPIO: 5, LEDC channel 0, timer 0.
 */

#ifndef TONE_ENGINE_H
#define TONE_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Musical note enumeration covering C4 through C6 plus REST.
 *
 * Values are indices into the internal frequency table.
 */
typedef enum {
    NOTE_REST = 0, /**< Silence — LEDC duty goes to 0 */
    NOTE_C4,
    NOTE_Cs4,
    NOTE_D4,
    NOTE_Ds4,
    NOTE_E4,
    NOTE_F4,
    NOTE_Fs4,
    NOTE_G4,
    NOTE_Gs4,
    NOTE_A4,
    NOTE_As4,
    NOTE_B4,
    NOTE_C5,
    NOTE_Cs5,
    NOTE_D5,
    NOTE_Ds5,
    NOTE_E5,
    NOTE_F5,
    NOTE_Fs5,
    NOTE_G5,
    NOTE_Gs5,
    NOTE_A5,
    NOTE_As5,
    NOTE_B5,
    NOTE_C6,
    NOTE_COUNT, /**< Sentinel — number of valid note values */
} note_t;

/**
 * @brief Initialise LEDC timer and channel for piezo output.
 *
 * Sets 13-bit resolution, 440 Hz initial frequency, duty 0 (silent).
 * Must be called once before tone_engine_play() or tone_engine_stop().
 *
 * @return ESP_OK on success, or a forwarded LEDC error code.
 */
int tone_engine_init(void);

/**
 * @brief Play a note with optional semitone shift.
 *
 * Looks up the base frequency for @p note, applies @p semitone_shift
 * (clamped to [-12, +12]), then clamps the result to [100, 3500] Hz.
 * Sets duty to 1<<12 (50% — the safety cap) while the note sounds.
 * NOTE_REST silences the output regardless of shift.
 *
 * @param note          Note to play (NOTE_REST silences output).
 * @param semitone_shift Shift in semitones, clamped to [-12, +12].
 */
void tone_engine_play(note_t note, int8_t semitone_shift);

/**
 * @brief Silence the piezo by setting duty to 0.
 */
void tone_engine_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TONE_ENGINE_H */
