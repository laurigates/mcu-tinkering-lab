#ifndef LLM_CLIENT_H
#define LLM_CLIENT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// LLM backend types
typedef enum { LLM_BACKEND_OLLAMA = 0, LLM_BACKEND_CLAUDE = 1 } llm_backend_type_t;

// LLM response structure
typedef struct {
    char *text;
    char *movement_command;  // forward, backward, left, right, stop
    char *confidence;        // low, medium, high
    char *objects_detected;  // comma-separated list
    bool has_obstacles;
    int error_code;
} llm_response_t;

// LLM client configuration
typedef struct {
    llm_backend_type_t backend_type;
    char *api_key;
    char *server_url;
    char *model;
    int timeout_ms;
} llm_config_t;

// Initialize LLM client
esp_err_t llm_client_init(llm_config_t *config);

// Analyze image with LLM
esp_err_t llm_analyze_image(const uint8_t *image_data, size_t image_size, const char *prompt,
                            llm_response_t *response);

// Send text query to LLM
esp_err_t llm_query_text(const char *query, llm_response_t *response);

// Process vision for robot control
esp_err_t llm_process_vision(const uint8_t *image_data, size_t image_size,
                             llm_response_t *response);

// Free LLM response
void llm_free_response(llm_response_t *response);

// Cleanup LLM client
void llm_client_cleanup(void);

// Helper to format vision prompt
char *llm_format_vision_prompt(const char *context, const char *additional_instructions);

// Parse movement commands from LLM response
esp_err_t llm_parse_movement(const char *response_text, char **movement, char **reason);

#endif  // LLM_CLIENT_H
