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
 * The overlap is deliberately NOT started at the first byte.  Gemini's TTS
 * stream is frequently slower than real time (measured 2026-07: real-time
 * factors 0.55-3.81 across seven utterances, three of them below 1.0), so
 * playing on arrival drains the ring dry mid-sentence and the I2S DMA tears.
 * Playback is therefore gated on AUDIO_PREROLL_BYTES having banked, OR on the
 * fetch having completed — see player_task().  Short utterances, which are the
 * ones that starve, take the second branch and play from a complete buffer.
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
 * @brief Mark the start of an utterance, before the first write.
 *
 * Closes the preroll gate so this utterance must bank AUDIO_PREROLL_BYTES
 * before anything plays — unless the ring still holds the previous utterance,
 * in which case the gate stays open (that audio must keep playing) and this
 * utterance inherits the buffer already in front of it.
 *
 * Must be paired with audio_player_end_utterance() or audio_player_abort() on
 * every exit path — exactly one of the two, never both. Skipping it leaves a
 * short utterance sitting in the ring unplayed; calling both re-opens the gate
 * that abort() just closed, and the next utterance then starts with nothing
 * banked.
 */
void audio_player_begin_utterance(void);

/**
 * @brief Mark the end of an utterance: everything that is coming has arrived.
 *
 * Releases the preroll gate if there is audio to play — an utterance shorter
 * than AUDIO_PREROLL_BYTES never reaches the threshold, so waiting for it would
 * mean never playing at all — then lets the player drain the tail and power the
 * I2S channel down cleanly. A request that produced no audio leaves the gate
 * shut, so it cannot hand a free pass to the next utterance.
 */
void audio_player_end_utterance(void);

/**
 * @brief Drop buffered audio and stop immediately.
 *
 * For reflexes that need silence now (e.g. an obstacle stop).
 */
void audio_player_abort(void);

/** True once the player task and PSRAM ring are up (i.e. init succeeded). */
bool audio_player_is_ready(void);

#ifdef __cplusplus
}
#endif
