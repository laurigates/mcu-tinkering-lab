/**
 * @file gemini_backend.h
 * @brief Google Gemini Robotics-ER 1.6 planner backend.
 *
 * Provides a synchronous planner call: upload a JPEG frame, receive a
 * structured goal_t parsed from the model's function-call response.
 *
 * Thread safety: gemini_backend_plan() must not be called concurrently.
 * The caller (planner_task) is the sole consumer and runs on a single task.
 */

#ifndef GEMINI_BACKEND_H
#define GEMINI_BACKEND_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* goal_t is defined by Agent B in goal_state.h.  If that header is not yet
 * in place during early compilation passes, the forward-declaration comment
 * below serves as a placeholder reminder.  Remove this comment once
 * goal_state.h exists and is stable. */
#include "goal_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the Gemini backend.
 *
 * Allocates the HTTP response buffer and validates that the Gemini API key
 * is available via credentials_loader.  Must be called once before
 * gemini_backend_plan().
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the API key is absent,
 *         ESP_ERR_NO_MEM if the response buffer cannot be allocated.
 */
esp_err_t gemini_backend_init(void);

/**
 * @brief Upload a JPEG frame to Gemini ER 1.6 and retrieve a structured goal.
 *
 * Sends the image together with a function-declaration system prompt.  The
 * model responds with a single functionCall object which is parsed into
 * *out_goal.  On any failure (HTTP error, JSON parse error, missing
 * functionCall) the function returns ESP_FAIL and sets
 * out_goal->kind = GOAL_KIND_STOP so the executor defaults to a safe hold.
 *
 * @param jpeg          Pointer to JPEG-encoded frame data.
 * @param jpeg_len      Length of the JPEG data in bytes.
 * @param out_goal      Output: parsed goal.  Must not be NULL.
 * @param latency_ms_out  Output: round-trip latency in milliseconds.
 *                        May be NULL if the caller does not need it.
 * @return ESP_OK on success, ESP_FAIL on recoverable error.
 */
esp_err_t gemini_backend_plan(const uint8_t *jpeg, size_t jpeg_len, goal_t *out_goal,
                              uint32_t *latency_ms_out);

/**
 * @brief Release resources allocated by gemini_backend_init().
 *
 * Safe to call even if gemini_backend_init() was not called or failed.
 */
void gemini_backend_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* GEMINI_BACKEND_H */
