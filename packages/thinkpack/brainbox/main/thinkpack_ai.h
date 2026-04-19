/**
 * @file thinkpack_ai.h
 * @brief Text-query AI backend vtable for the ThinkPack Brainbox.
 *
 * Defines a common interface for sending text prompts to an AI backend and
 * receiving a text response. The concrete backend is selected at compile time
 * via Kconfig (CONFIG_AI_BACKEND_CLAUDE / _GEMINI / _OLLAMA).
 */

#ifndef THINKPACK_AI_H
#define THINKPACK_AI_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Heap-allocated text response returned by query_text().
 *
 * The caller is responsible for releasing the allocation via free_response().
 */
typedef struct {
    char *response_text;    /**< NUL-terminated response string (heap-allocated) */
    size_t response_length; /**< Length of response_text excluding NUL terminator */
} ai_response_t;

/**
 * @brief Configuration supplied to the backend during init().
 */
typedef struct {
    const char *api_key; /**< may be NULL for backends that don't need it (Ollama) */
    const char *api_url; /**< base URL; NULL means use backend default */
    const char *model;   /**< model name; NULL means use backend default */
} ai_config_t;

/**
 * @brief Backend vtable.
 *
 * All function pointers are mandatory — implementations must not be NULL.
 * Only one backend is compiled into a given firmware image (Kconfig choice).
 */
typedef struct {
    /**
     * @brief Initialise the backend.
     *
     * Must be called once before query_text(). Fields in @p config that are
     * NULL instruct the backend to use its built-in defaults.
     *
     * @param config  Backend configuration; must not be NULL.
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG for missing required fields.
     */
    esp_err_t (*init)(const ai_config_t *config);

    /**
     * @brief Send a text prompt and block until the response arrives.
     *
     * @param prompt    NUL-terminated prompt string; must not be NULL.
     * @param response  Output structure to populate; must not be NULL.
     *                  On success, response->response_text is heap-allocated.
     * @return ESP_OK on success, ESP_FAIL on HTTP / parse error.
     */
    esp_err_t (*query_text)(const char *prompt, ai_response_t *response);

    /**
     * @brief Release heap memory owned by @p response.
     *
     * Safe to call with a NULL @p response or a response whose response_text
     * is already NULL.
     *
     * @param response  Response to free.
     */
    void (*free_response)(ai_response_t *response);

    /**
     * @brief Tear down the backend and release resources.
     */
    void (*deinit)(void);
} thinkpack_ai_backend_t;

/**
 * @brief Return the backend compiled in via Kconfig (Claude / Gemini / Ollama).
 *
 * The returned pointer is to a statically-allocated object and must not be
 * freed. The pointer is valid for the lifetime of the process.
 *
 * @return Pointer to the active thinkpack_ai_backend_t.
 */
const thinkpack_ai_backend_t *thinkpack_ai_get_current(void);

#endif  // THINKPACK_AI_H
