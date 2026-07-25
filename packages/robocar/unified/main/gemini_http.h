/**
 * @file gemini_http.h
 * @brief One-shot HTTPS POST to a Gemini generateContent endpoint.
 *
 * Every Gemini call in this firmware — the planner, the narrate helper and the
 * TTS renderer — sends the same request shape: POST, JSON body, an
 * `x-goog-api-key` header, TLS via the certificate bundle, and a per-call event
 * handler that accumulates or streams the response. That plumbing was
 * copy-pasted at three call sites, so a fix to one (a header, a timeout, a
 * TLS setting) silently missed the other two.
 *
 * What stays at the call site is what genuinely differs: building the request
 * body, the event handler, and interpreting the response.
 */

#pragma once

#include "esp_err.h"
#include "esp_http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POST a JSON body to a Gemini endpoint and run @p handler over the reply.
 *
 * Performs the request synchronously and always cleans up the client, including
 * on every error path.
 *
 * @param url         Full endpoint URL, including `:generateContent`.
 * @param api_key     Value for the `x-goog-api-key` header. Must not be NULL.
 * @param body        NUL-terminated JSON request body. Must not be NULL.
 * @param timeout_ms  Request timeout.
 * @param handler     Event handler invoked for response data.
 * @param user_ctx    Passed to @p handler as `evt->user_data`.
 * @param[out] status_out  HTTP status code, when non-NULL. Written even when the
 *                    call returns an error, so callers can distinguish a
 *                    transport failure (status 0) from an API rejection.
 *
 * @return ESP_OK when the transport succeeded *and* the status was 200;
 *         ESP_ERR_INVALID_ARG on a NULL url/api_key/body;
 *         ESP_FAIL if the client could not be created or the status was not 200;
 *         otherwise the esp_http_client_perform() error.
 */
esp_err_t gemini_http_post(const char *url, const char *api_key, const char *body, int timeout_ms,
                           http_event_handle_cb handler, void *user_ctx, int *status_out);

#ifdef __cplusplus
}
#endif
