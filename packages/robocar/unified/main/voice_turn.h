/**
 * @file voice_turn.h
 * @brief Push-to-talk: record a clip, ask Gemini, speak the reply.
 *
 * ## What this is
 *
 * The robot's first *conversational* path. Everything else it says is
 * unprompted — the planner decides there is something worth remarking on and
 * the gates decide whether it may. A voice turn is the opposite: a person asks
 * it something and it answers, once, on demand.
 *
 * The TTS half is reused entirely unchanged. This module produces a line of
 * text and hands it to `speech_queue_post()`, exactly as the planner does.
 *
 * ## Why a persistent task and a 1-deep queue
 *
 * Matching `gemini_tts.c`'s shape rather than creating a task per command. Two
 * reasons, and neither is style:
 *
 *   - A 1-deep queue makes "already listening" a *queue-full* rejection rather
 *     than a race. Two `listen` commands in quick succession cannot both hold
 *     the microphone, and the second gets an immediate, honest "busy".
 *   - A per-turn task would have to size its stack for an HTTPS handshake
 *     (>= 8 KB, see .claude/rules/freertos-task-gotchas.md) and would allocate
 *     and free it around every utterance, on a device where the fragmentation
 *     that causes is paid by the camera.
 *
 * ## Why it does not use the planner's model
 *
 * Gemini's free-tier quota is **per model**, and the planner already saturates
 * `gemini-robotics-er-1.6-preview` at 5 requests/minute. A voice turn on a flash
 * model draws on a separate allowance, so answering someone can never cost the
 * robot its ability to plan a movement.
 *
 * ## The clip is WAV, not raw PCM
 *
 * Measured, not assumed: `audio/pcm` is rejected with an HTTP 400 that names no
 * field; `audio/wav` works. See audio_clip.h for the probe.
 */

#ifndef VOICE_TURN_H
#define VOICE_TURN_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 8 KB because this task performs an HTTPS request: the mbedTLS handshake is
 *  several KB deep on its own, and a 4 KB stack survives only while DNS is
 *  broken enough to short-circuit before it. That intermittency is why the
 *  number is not tuned down. */
#define VOICE_TURN_TASK_STACK_SIZE 8192
#define VOICE_TURN_TASK_PRIORITY 3
#define VOICE_TURN_TASK_CORE 1

/**
 * @brief Start the voice-turn task.
 *
 * Non-fatal like the rest of the AI phase: without it the console simply
 * reports that voice turns are unavailable.
 */
esp_err_t voice_turn_start(void);

/**
 * @brief Request one turn of @p window_ms milliseconds of recording.
 *
 * Returns immediately; the turn runs on the voice-turn task.
 *
 * Deliberately callable from anywhere, including the ambient listener, so an
 * energy-triggered (VAD) turn can be added later without redesign. Before doing
 * that, note the pacing problem: a VAD in a noisy room would issue turns
 * continuously and Gemini's `retryDelay` grows while a client keeps asking. The
 * likely answer is that a VAD trigger must draw on `speech_budget` too, which is
 * a decision worth making deliberately rather than discovering.
 *
 * @return ESP_OK when queued; ESP_ERR_INVALID_STATE if the task is not running
 *         or the robot is currently speaking; ESP_ERR_NO_MEM if a turn is
 *         already in flight; ESP_ERR_INVALID_ARG on an out-of-range window.
 */
esp_err_t voice_turn_request(uint32_t window_ms);

/** @brief True while a turn is recording, uploading or awaiting a reply. */
bool voice_turn_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif  // VOICE_TURN_H
