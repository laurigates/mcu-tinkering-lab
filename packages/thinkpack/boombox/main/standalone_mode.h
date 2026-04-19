/**
 * @file standalone_mode.h
 * @brief Beat scheduler — drives tone_engine and led_single from pots.
 *
 * Reads tempo and pitch from pot_reader, advances melody_gen one step
 * per beat, plays the note via tone_engine, and flashes the LED on
 * downbeats. Beat index can be overridden by group_mode when a sync
 * pulse arrives.
 */

#ifndef STANDALONE_MODE_H
#define STANDALONE_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise standalone mode state. Call once before the tone task.
 */
void standalone_mode_init(void);

/**
 * @brief Advance the beat scheduler. Call every control-loop tick (~10 ms).
 *
 * Reads the tempo pot, fires note-on at the correct beat time, schedules
 * note-off based on hold_fraction, and fires led_single_flash on downbeats.
 *
 * @param now_ms  Current time in milliseconds.
 */
void standalone_mode_tick(uint32_t now_ms);

/**
 * @brief Return the current beat index (monotonically increasing).
 */
uint32_t standalone_mode_get_beat_index(void);

/**
 * @brief Force the beat index to a specific value.
 *
 * Used by group_mode to align this node's beat phase with the leader's
 * sync pulse.
 *
 * @param beat_index  Beat index to jump to.
 */
void standalone_mode_set_beat_index(uint32_t beat_index);

/**
 * @brief Reset beat index to 0 and restore MELODY_MARCH pattern.
 *
 * Implements the 2-second hold "soft factory reset" described in FR-T21.
 */
void standalone_mode_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* STANDALONE_MODE_H */
