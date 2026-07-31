/**
 * @file gemini_parse.h
 * @brief Parse Gemini Robotics-ER 1.6 function-call responses into goal_t.
 *
 * Extracted from gemini_backend.c so the parser can be unit-tested on the
 * host without dragging in esp_http_client, WiFi, etc. Fail-safe contract:
 * on any error the parser sets ``out_goal->kind = GOAL_KIND_STOP`` before
 * returning ``ESP_FAIL`` so callers never operate on an uninitialised goal.
 */

#ifndef GEMINI_PARSE_H
#define GEMINI_PARSE_H

#include <stddef.h>

#include "esp_err.h"
#include "goal_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Navigate candidates[0].content.parts[0].functionCall from the raw
 *        Gemini API response body and populate *out_goal.
 *
 * @param json_text  NUL-terminated response body. Must not be NULL.
 * @param out_goal   Destination goal. Must not be NULL. On failure its
 *                   ``kind`` is forced to ``GOAL_KIND_STOP``.
 * @return ESP_OK on success, ESP_FAIL on any parse/structure error,
 *         ESP_ERR_INVALID_ARG if either pointer is NULL.
 */
esp_err_t gemini_parse_function_call(const char *json_text, goal_t *out_goal);

/**
 * @brief As gemini_parse_function_call(), but also extracts a `speak` call.
 *
 * The model emits motion and speech as *parallel* function calls in the same
 * response, so both are recovered in one pass. Speech is returned by value
 * rather than folded into goal_t because the two have different lifetimes —
 * see speech_queue.h for why speech cannot be a goal.
 *
 * The return value reflects the **motion** goal only. A response carrying a
 * `speak` but no motion call still returns ESP_FAIL with kind=GOAL_KIND_STOP
 * (the pre-existing fail-safe contract), while `out_speech` is populated —
 * so the robot holds position and still talks.
 *
 * @param json_text   NUL-terminated response body. Must not be NULL.
 * @param out_goal    Destination goal. Must not be NULL.
 * @param out_speech  Destination for the utterance, or NULL to ignore speech.
 *                    Set to "" when the response carries no `speak` call.
 * @param speech_cap  Size of @p out_speech in bytes. Text is truncated to fit.
 * @return ESP_OK if a motion goal was parsed, ESP_FAIL otherwise,
 *         ESP_ERR_INVALID_ARG if @p json_text or @p out_goal is NULL.
 */
esp_err_t gemini_parse_response(const char *json_text, goal_t *out_goal, char *out_speech,
                                size_t speech_cap);

/**
 * @brief Extract a plain TEXT reply — candidates[0].content.parts[*].text.
 *
 * The counterpart to gemini_parse_function_call() for the calls that ask the
 * model for prose rather than a tool call: the self-report narrator and the
 * voice turn. Every `text` part is concatenated in order, then newlines, returns
 * and tabs are collapsed to spaces and the result is trimmed — a spoken line
 * must be one line, and the TTS renderer would otherwise pronounce the layout.
 *
 * Lives here rather than in gemini_backend.c (where it began, static and
 * untested) so that it can be exercised on the host, and so the two callers
 * cannot drift apart. Extracting it gave the narrate path its first test as a
 * side effect.
 *
 * @param json_text  NUL-terminated response body. Must not be NULL.
 * @param out        Destination buffer. Must not be NULL.
 * @param out_len    Capacity of @p out, including the NUL.
 * @return ESP_OK when at least one non-empty text part was found and survived
 *         sanitisation; ESP_FAIL on a parse error, a missing/!array parts list,
 *         or a reply that sanitises to nothing; ESP_ERR_INVALID_ARG on a NULL
 *         pointer or zero capacity.
 */
esp_err_t gemini_parse_text(const char *json_text, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif /* GEMINI_PARSE_H */
