#ifndef GEMINI_TTS_H
#define GEMINI_TTS_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * Result of a TTS request — PCM audio allocated in PSRAM.
 * Caller must free pcm_data when done (or pass to audio queue which frees it).
 */
typedef struct {
    int16_t *pcm_data;     // PSRAM-allocated PCM buffer (mono, 16-bit signed)
    size_t pcm_len;        // Length in bytes
    uint32_t sample_rate;  // Always 24000 Hz from Gemini
} tts_result_t;

/**
 * Request TTS audio from the Gemini API.
 *
 * Sends the prompt text to Gemini, receives JSON with base64-encoded PCM,
 * decodes it into PSRAM, and returns the result.
 *
 * @param prompt      Text to speak (the transcript portion of the TTS prompt).
 * @param result      Output: decoded PCM audio in PSRAM.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t gemini_tts_request(const char *prompt, tts_result_t *result);

#endif  // GEMINI_TTS_H
