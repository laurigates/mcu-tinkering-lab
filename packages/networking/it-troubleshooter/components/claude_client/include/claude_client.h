/**
 * @file claude_client.h
 * @brief Claude API client for IT Troubleshooter (Phase 3).
 *
 * Sends operator-pasted command output to the Anthropic Claude API over
 * HTTPS and returns the suggested next diagnostic command.
 *
 * Must be called from a task context (not ISR or event handler) — the
 * underlying HTTP+TLS stack blocks until the request completes.
 * Minimum recommended task stack: 8 KB.
 */
#pragma once

#include <stddef.h>
#include "esp_err.h"

/**
 * @brief Initialize the Claude API client with credentials.
 *
 * Stores pointers to caller-owned strings (typically constants from
 * credentials.h). Does not allocate heap resources — those are allocated
 * per-request in claude_client_analyze().
 *
 * @param api_key  Anthropic API key (null-terminated, e.g. "sk-ant-...").
 * @param model    Claude model ID (e.g. "claude-haiku-4-5-20251001").
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if either argument is NULL.
 */
esp_err_t claude_client_init(const char *api_key, const char *model);

/**
 * @brief Analyze command output and return the next suggested diagnostic command.
 *
 * Sends the output to Claude API over HTTPS POST. Blocks until the response
 * is received or the 30-second timeout expires.
 *
 * Output is truncated to 1024 bytes before transmission if longer, to
 * limit payload size and avoid heap exhaustion.
 *
 * @param output        Null-terminated command output from the target computer.
 * @param next_cmd      Buffer to receive the suggested command (null-terminated).
 *                      Contains "DONE" if Claude considers troubleshooting complete.
 * @param next_cmd_len  Size of the next_cmd buffer (minimum 64 bytes recommended).
 * @return ESP_OK on success.
 *         ESP_FAIL if the API returned an error or the response could not be parsed.
 *         ESP_ERR_TIMEOUT if the request timed out.
 *         ESP_ERR_INVALID_STATE if the client was not initialized.
 *         ESP_ERR_INVALID_ARG if output or next_cmd is NULL.
 */
esp_err_t claude_client_analyze(const char *output, char *next_cmd, size_t next_cmd_len);

/**
 * @brief Deinitialize the Claude API client and clear stored credentials.
 */
void claude_client_deinit(void);
