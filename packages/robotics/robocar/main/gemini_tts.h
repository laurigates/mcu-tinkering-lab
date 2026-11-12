/**
 * @file gemini_tts.h
 * @brief Gemini text-to-speech fetch task — turns queued text into PCM.
 *
 * Consumes speech_queue and streams the resulting audio into audio_player.
 * This is a *second* Gemini call, distinct from the Robotics-ER planner call
 * in gemini_backend.c: the ER model decides what to say, a TTS model renders
 * it. They are separate models and separate requests.
 *
 * The response body is decoded incrementally (base64_stream_feed) straight
 * into the playback ring. It is never buffered whole — a few seconds of
 * 24 kHz PCM is hundreds of kilobytes of base64, far beyond the 16 kB
 * response buffer the planner backend uses.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the TTS fetch task.
 *
 * Requires speech_queue_init() and audio_player_init() to have run, and WiFi
 * to be up by the time the first utterance is queued.
 *
 * @return ESP_OK, or ESP_FAIL if the task could not be created.
 */
esp_err_t gemini_tts_start(void);

#ifdef __cplusplus
}
#endif
