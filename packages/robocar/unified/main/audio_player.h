/**
 * @file audio_player.h
 * @brief I2S playback to the MAX98357A, fed by a PSRAM ring buffer.
 *
 * Split from the TTS fetch (gemini_tts.c) because esp_http_client_perform()
 * blocks for the duration of the download.  The fetcher pushes decoded PCM in
 * as it arrives; this module's task drains the ring into I2S concurrently, so
 * the robot starts speaking while the rest of the utterance is still on the
 * wire.  That overlap is what keeps perceived latency near the time-to-first-
 * byte rather than the full download.
 *
 * Audio is 24 kHz mono int16 (Gemini TTS native rate — see pin_config.h).
 * The MAX98357A is driven in a standard Philips stereo frame with the mono
 * sample duplicated into both slots; the amp averages them.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise I2S, allocate the PSRAM ring, and start the player task.
 *
 * Idempotent. Call once at boot, after PSRAM is up.
 *
 * @return ESP_OK, ESP_ERR_NO_MEM if the ring could not be allocated in PSRAM,
 *         or an I2S driver error.
 */
esp_err_t audio_player_init(void);

/**
 * @brief Push decoded PCM into the playback ring.
 *
 * Blocks up to @p timeout_ms when the ring is full — this is the backpressure
 * that stops a fast download from outrunning real-time playback. Called from
 * the TTS fetch task's HTTP callback.
 *
 * @param pcm        int16 mono samples at AUDIO_SAMPLE_RATE_HZ.
 * @param bytes      Length of @p pcm in bytes.
 * @param timeout_ms Max block time.
 * @return ESP_OK, ESP_ERR_TIMEOUT if the ring stayed full, or
 *         ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t audio_player_write(const uint8_t *pcm, size_t bytes, uint32_t timeout_ms);

/**
 * @brief Mark the end of an utterance.
 *
 * Lets the player drain the tail and power the I2S channel down cleanly
 * instead of waiting out an idle timeout.
 */
void audio_player_end_utterance(void);

/**
 * @brief Drop buffered audio and stop immediately.
 *
 * For reflexes that need silence now (e.g. an obstacle stop).
 */
void audio_player_abort(void);

/** True while audio is buffered or actively playing. */
bool audio_player_is_busy(void);

#ifdef __cplusplus
}
#endif
