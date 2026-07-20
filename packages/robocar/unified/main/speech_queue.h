/**
 * @file speech_queue.h
 * @brief Utterance queue — the speech sibling of goal_state.
 *
 * Why this is a queue and not a `goal_kind_t`
 * -------------------------------------------
 * goal_state is *state*: last-write-wins, TTL-bounded, and deliberately
 * collapses to GOAL_KIND_STOP when it goes stale (see goal_state.h).  Those
 * are exactly the wrong semantics for speech:
 *
 *   - Last-write-wins would truncate an utterance the moment the planner
 *     emitted its next motion goal (~1 s later), cutting sentences mid-word.
 *   - TTL-expiry-to-STOP has no meaning for audio that is already playing.
 *   - A union member would make talking and driving mutually exclusive, when
 *     the robot should be able to do both at once.
 *
 * Speech is an *event stream*, so it gets a queue that runs alongside the
 * goal — the planner writes to both, and the two are consumed independently
 * by tasks on different cores.
 *
 * Overflow policy is drop-newest: if the robot is still speaking when the
 * planner produces another line, the in-flight utterance wins.  Dropping the
 * new line keeps speech intelligible; dropping the old one would produce the
 * same stutter this module exists to avoid.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Longest utterance accepted, including the NUL.
 *
 *  Sized for one or two spoken sentences.  Gemini TTS bills per character and
 *  the robot narrates continuously, so this is a cost guard as much as a
 *  memory one — the planner prompt also asks for short lines.
 */
#define SPEECH_TEXT_MAX 160

/** Pending utterances held before the TTS fetch. Small on purpose. */
#define SPEECH_QUEUE_DEPTH 2

/** One queued line of speech. */
typedef struct {
    char text[SPEECH_TEXT_MAX];
} speech_request_t;

/**
 * @brief Create the queue. Call once at boot, before the planner starts.
 *
 * Idempotent — a second call returns ESP_OK without reallocating.
 */
esp_err_t speech_queue_init(void);

/**
 * @brief Enqueue an utterance. Never blocks.
 *
 * @param text NUL-terminated line to speak. Truncated to SPEECH_TEXT_MAX-1.
 * @return ESP_OK if queued, ESP_ERR_NO_MEM if the queue was full (utterance
 *         dropped by design), ESP_ERR_INVALID_ARG on NULL/empty text,
 *         ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t speech_queue_post(const char *text);

/**
 * @brief Block until an utterance is available.
 *
 * @param out            Destination. Must not be NULL.
 * @param timeout_ms     Max wait; UINT32_MAX to wait forever.
 * @return ESP_OK on receipt, ESP_ERR_TIMEOUT if none arrived.
 */
esp_err_t speech_queue_receive(speech_request_t *out, uint32_t timeout_ms);

/** Drop all pending utterances (e.g. on an obstacle reflex). */
void speech_queue_flush(void);

#ifdef __cplusplus
}
#endif
