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

#ifdef __cplusplus
}
#endif

#endif /* GEMINI_PARSE_H */
