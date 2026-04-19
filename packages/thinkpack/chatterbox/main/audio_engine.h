/**
 * @file audio_engine.h
 * @brief I2S record (INMP441) + playback (MAX98357A) driver for Chatterbox.
 *
 * Owns two i2s_chan handles (RX + TX) and a single PSRAM-backed clip buffer
 * sized for THINKPACK_AUDIO_CLIP_MAX_SAMPLES.  The API is intentionally small:
 * start recording, stop recording, play a captured clip, play a clip at a
 * semitone offset.  All playback happens on the calling task (no internal
 * task/queue) — callers that need deferred playback should wrap these calls.
 *
 * Hardware-unverified.  See WIRING.md for the pin table.
 */

#ifndef AUDIO_ENGINE_H
#define AUDIO_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the I2S RX + TX channels and allocate the PSRAM clip
 *        buffer.
 *
 * Safe to call once at boot.  Idempotent calls return ESP_ERR_INVALID_STATE.
 */
esp_err_t audio_engine_init(void);

/**
 * @brief Begin capturing samples into the PSRAM clip buffer.
 *
 * Samples drain into the buffer until audio_engine_record_stop() is called
 * or the clip-length cap is reached.  The function blocks briefly on I2S RX
 * enable but does not block for the full clip duration — the caller must
 * poll audio_engine_record_samples() and stop explicitly.
 */
esp_err_t audio_engine_record_start(void);

/**
 * @brief Pull samples from the I2S RX channel into the clip buffer.
 *
 * Call this from a task loop until @p reached_limit is set or the user
 * releases the touch pad.  Non-blocking (uses pdMS_TO_TICKS(10) timeout
 * internally).
 *
 * @param[out] reached_limit  Set to true when the clip-length cap is hit.
 * @return     ESP_OK on success.
 */
esp_err_t audio_engine_record_tick(bool *reached_limit);

/**
 * @brief Stop the RX channel and return the captured sample count.
 */
esp_err_t audio_engine_record_stop(size_t *out_samples);

/**
 * @brief Play the currently captured clip via I2S TX (blocking).
 */
esp_err_t audio_engine_playback(void);

/**
 * @brief Play the captured clip, pitch-shifted by @p semitone_shift.
 *
 * Uses thinkpack_audio_pitch_shift() and a scratch buffer.  Blocks until
 * playback completes.
 */
esp_err_t audio_engine_playback_with_pitch_shift(int8_t semitone_shift);

/**
 * @brief Pointer + length accessors for callers that need to forward the
 *        clip over the mesh (fragmented send).
 *
 * The pointer remains valid until the next record_start().
 */
const int16_t *audio_engine_clip_samples(void);
size_t audio_engine_clip_sample_count(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_ENGINE_H */
