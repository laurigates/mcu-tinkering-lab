/**
 * @file sound_clips.h
 * @brief The prebuilt clips' PCM in flash, and writing one into the playback
 *        ring. Names, tags and the line split are in sound_clips_core.h.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "sound_clips_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write @p clip's PCM into the audio player's ring.
 *
 * Call only from the TTS task, between audio_player_begin_utterance() and its
 * end/abort: the ring has one producer, and that task is it. The clip is
 * already 24 kHz mono int16 (tools/build-sounds.py), so it is copied as is.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG for SOUND_CLIP_NONE, or the
 *         audio_player_write() error (ESP_ERR_TIMEOUT if the ring stayed full).
 */
esp_err_t sound_clips_write(sound_clip_t clip, uint32_t timeout_ms);

/** Length of @p clip in milliseconds, 0 for SOUND_CLIP_NONE. */
uint32_t sound_clips_duration_ms(sound_clip_t clip);

/**
 * @brief Ask the TTS task to play @p clip on its own, from any task.
 *
 * For the obstacle reflex. The executor runs with under 1 kB of stack headroom,
 * and speech_queue_post() needs about that much for its filter scratch and the
 * request copy, so the reflex leaves a request here instead: one atomic word,
 * no stack, no blocking. The TTS task takes it within ~50 ms when the speaker
 * is free, or after the line it is speaking. A second request before the first
 * is taken replaces it.
 */
void sound_clips_request(sound_clip_t clip);

/** Take and clear the pending request; SOUND_CLIP_NONE when there is none. */
sound_clip_t sound_clips_take_request(void);

#ifdef __cplusplus
}
#endif
