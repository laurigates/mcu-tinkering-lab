/**
 * @file melody_gen.h
 * @brief Procedural melody pattern generator.
 *
 * Returns one melody_step_t per beat tick. The caller drives timing;
 * this module is stateless aside from the selected pattern and an
 * internal random seed for the pentatonic walk.
 */

#ifndef MELODY_GEN_H
#define MELODY_GEN_H

#include <stdbool.h>
#include <stdint.h>

#include "tone_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Available melody patterns.
 */
typedef enum {
    MELODY_SILENCE = 0, /**< Always returns NOTE_REST — musical-round silence partner */
    MELODY_MARCH,       /**< 4/4 march in C major */
    MELODY_WALTZ,       /**< 3/4 in G major */
    MELODY_PENTATONIC,  /**< Pseudo-random pentatonic walk */
    MELODY_COUNT,       /**< Sentinel */
} melody_pattern_t;

/**
 * @brief Single beat output from the melody generator.
 */
typedef struct {
    note_t note;           /**< Note to play this beat (NOTE_REST = silence) */
    uint8_t hold_fraction; /**< 1=whole-beat, 2=half-beat, 4=quarter-beat  */
    bool is_downbeat;      /**< True on pattern-defining downbeats (for LED flash) */
} melody_step_t;

/**
 * @brief Set the active melody pattern.
 *
 * Thread-safe with respect to melody_gen_next() because patterns are
 * selected at the start of the next beat tick.
 *
 * @param pattern  Pattern to select.
 */
void melody_gen_set_pattern(melody_pattern_t pattern);

/**
 * @brief Return the currently selected pattern.
 */
melody_pattern_t melody_gen_get_pattern(void);

/**
 * @brief Advance the generator by one beat and return the step to play.
 *
 * @param beat_index  Monotonically incrementing beat counter.
 * @return            Step descriptor for this beat.
 */
melody_step_t melody_gen_next(uint32_t beat_index);

#ifdef __cplusplus
}
#endif

#endif /* MELODY_GEN_H */
