#ifndef AI_BACKEND_H
#define AI_BACKEND_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Generic AI response structure
 */
typedef struct {
    char* response_text;    /**< The response content from the AI backend. Must be freed by the caller using free_response. */
    size_t response_length; /**< Length of the response text. */
} ai_response_t;

/**
 * @brief Configuration for an AI backend.
 */
typedef struct {
    const char* api_key; /**< API key for the service, can be NULL if not needed. */
    const char* api_url; /**< The API endpoint URL. */
    const char* model;   /**< The model to use. */
} ai_config_t;

/**
 * @brief Interface for an AI backend.
 *
 * This structure defines a generic interface for various AI vision backends.
 */
typedef struct {
    /**
     * @brief Initialize the AI backend.
     * @param config Configuration for the backend.
     * @return ESP_OK on success.
     */
    esp_err_t (*init)(const ai_config_t* config);

    /**
     * @brief Analyze an image.
     * @param image_data Pointer to the image data (JPEG).
     * @param image_size Size of the image data.
     * @param response Pointer to a response structure to be filled.
     * @return ESP_OK on success.
     */
    esp_err_t (*analyze_image)(const uint8_t* image_data, size_t image_size, ai_response_t* response);

    /**
     * @brief Free the resources associated with a response.
     * @param response The response to free.
     */
    void (*free_response)(ai_response_t* response);

    /**
     * @brief Deinitialize the AI backend.
     */
    void (*deinit)(void);
} ai_backend_t;

/**
 * @brief Get the currently configured AI backend.
 * @return A pointer to the active AI backend interface.
 */
const ai_backend_t* ai_backend_get_current(void);

#endif // AI_BACKEND_H
