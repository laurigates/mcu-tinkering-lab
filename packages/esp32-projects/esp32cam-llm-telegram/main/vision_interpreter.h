#ifndef VISION_INTERPRETER_H
#define VISION_INTERPRETER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "llm_client.h"
#include "motor_controller.h"

// Vision interpretation result
typedef struct {
    char *scene_description;
    char *objects_detected;
    motor_command_t suggested_action;
    int suggested_speed;
    uint32_t suggested_duration_ms;
    char *reasoning;
    char *confidence_level;
    bool obstacle_detected;
    bool path_clear;
} vision_result_t;

// Vision interpreter context
typedef struct {
    bool is_active;
    uint32_t analysis_count;
    uint32_t successful_analyses;
    uint32_t failed_analyses;
    vision_result_t last_result;
} vision_interpreter_t;

// Initialize vision interpreter
esp_err_t vision_interpreter_init(void);

// Analyze image and generate motor commands
esp_err_t vision_analyze_and_interpret(const uint8_t *image_data, size_t image_size,
                                       vision_result_t *result);

// Convert LLM response to motor commands
esp_err_t vision_parse_llm_response(const llm_response_t *llm_response, vision_result_t *result);

// Generate action from vision result
esp_err_t vision_get_motor_action(const vision_result_t *result, motor_command_t *command,
                                  int *speed, uint32_t *duration_ms);

// Format vision result for Telegram message
char *vision_format_telegram_message(const vision_result_t *result);

// Free vision result resources
void vision_free_result(vision_result_t *result);

// Get interpreter status
vision_interpreter_t vision_get_status(void);

// Cleanup vision interpreter
void vision_interpreter_cleanup(void);

#endif  // VISION_INTERPRETER_H
