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

#ifdef __cplusplus
}
#endif

#endif /* GEMINI_PARSE_H */
