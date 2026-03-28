#ifndef I2S_AUDIO_H
#define I2S_AUDIO_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * Initialize I2S output for MAX98357A DAC.
 * Must be called before any playback functions.
 */
esp_err_t i2s_audio_init(void);

/**
 * Play mono PCM audio through I2S (blocking).
 * Handles mono-to-stereo duplication for the MAX98357A.
 *
 * @param pcm_data   Pointer to 16-bit signed PCM samples (mono).
 * @param pcm_len    Length in bytes.
 * @param sample_rate Sample rate in Hz (e.g., 24000).
 */
esp_err_t i2s_audio_play_mono(const int16_t *pcm_data, size_t pcm_len, uint32_t sample_rate);

/**
 * Start the audio playback task on Core 1.
 * Called once during initialization.
 */
esp_err_t i2s_audio_start_task(void);

/**
 * Queue a PSRAM-allocated PCM buffer for playback.
 * The audio task on Core 1 will play it and free the buffer when done.
 *
 * @param pcm_data    Pointer to 16-bit signed PCM samples in PSRAM (mono).
 * @param pcm_len     Length in bytes.
 * @param sample_rate Sample rate in Hz.
 */
esp_err_t i2s_audio_queue_playback(int16_t *pcm_data, size_t pcm_len, uint32_t sample_rate);

/**
 * Wait until the current queued playback finishes.
 * Blocks the calling task.
 *
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return ESP_OK if playback finished, ESP_ERR_TIMEOUT if timed out.
 */
esp_err_t i2s_audio_wait_done(uint32_t timeout_ms);

#endif  // I2S_AUDIO_H
