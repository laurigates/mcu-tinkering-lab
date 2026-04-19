/**
 * @file standalone_mode.h
 * @brief Touch-to-record / touch-to-playback loop when no peers are around.
 *
 * Owns a thinkpack_audio_sm_t and drives the audio engine from touch edge
 * events.  Call standalone_mode_init() once at boot.  Touch edge callbacks
 * arrive via standalone_mode_on_touch() which posts the corresponding
 * THINKPACK_AUDIO_EVENT_* to the state machine and triggers the audio
 * engine accordingly.
 */

#ifndef STANDALONE_MODE_H
#define STANDALONE_MODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the state machine and internal state. */
void standalone_mode_init(void);

/** Feed a touch edge event (true=pressed, false=released). */
void standalone_mode_on_touch(bool pressed);

/**
 * @brief Tick the recorder while a touch is held.
 *
 * Called every TOUCH poll interval from the application task.  Pulls
 * samples from the audio engine and forces playback if the clip-length
 * cap is reached.
 */
void standalone_mode_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* STANDALONE_MODE_H */
