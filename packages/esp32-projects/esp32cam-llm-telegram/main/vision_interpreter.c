#include "vision_interpreter.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "VISION_INTERPRETER";

static vision_interpreter_t interpreter = {
    .is_active = false,
    .analysis_count = 0,
    .successful_analyses = 0,
    .failed_analyses = 0,
    .last_result = {0}
};

// Initialize vision interpreter
esp_err_t vision_interpreter_init(void) {
    ESP_LOGI(TAG, "Initializing vision interpreter");
    interpreter.is_active = true;
    return ESP_OK;
}

// Parse keywords from text to determine motor action
static motor_command_t parse_motor_command(const char* text) {
    if (!text) return MOTOR_CMD_STOP;

    // Convert to lowercase for easier parsing (simplified)
    if (strstr(text, "forward") || strstr(text, "straight") || strstr(text, "ahead")) {
        return MOTOR_CMD_FORWARD;
    } else if (strstr(text, "backward") || strstr(text, "reverse") || strstr(text, "back")) {
        return MOTOR_CMD_BACKWARD;
    } else if (strstr(text, "left")) {
        if (strstr(text, "rotate") || strstr(text, "turn around")) {
            return MOTOR_CMD_ROTATE_LEFT;
        }
        return MOTOR_CMD_LEFT;
    } else if (strstr(text, "right")) {
        if (strstr(text, "rotate") || strstr(text, "turn around")) {
            return MOTOR_CMD_ROTATE_RIGHT;
        }
        return MOTOR_CMD_RIGHT;
    } else if (strstr(text, "stop") || strstr(text, "halt") || strstr(text, "wait")) {
        return MOTOR_CMD_STOP;
    }

    return MOTOR_CMD_STOP;
}

// Parse speed from confidence level
static int parse_speed(const char* confidence) {
    if (!confidence) return MOTOR_DEFAULT_SPEED;

    if (strstr(confidence, "high")) {
        return MOTOR_DEFAULT_SPEED;
    } else if (strstr(confidence, "medium")) {
        return MOTOR_DEFAULT_SPEED * 3 / 4;
    } else if (strstr(confidence, "low")) {
        return MOTOR_DEFAULT_SPEED / 2;
    }

    return MOTOR_DEFAULT_SPEED / 2;
}

// Analyze image and generate motor commands
esp_err_t vision_analyze_and_interpret(const uint8_t* image_data, size_t image_size,
                                       vision_result_t* result) {
    if (!interpreter.is_active || !image_data || !result) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Analyzing image (size: %d bytes)", image_size);
    interpreter.analysis_count++;

    // Initialize result
    memset(result, 0, sizeof(vision_result_t));

    // Prepare LLM request
    llm_response_t llm_response = {0};
    esp_err_t err = llm_process_vision(image_data, image_size, &llm_response);

    if (err != ESP_OK || llm_response.error_code != 0) {
        ESP_LOGE(TAG, "Failed to get LLM response");
        interpreter.failed_analyses++;
        return ESP_FAIL;
    }

    // Parse LLM response
    err = vision_parse_llm_response(&llm_response, result);

    // Copy to last result
    if (interpreter.last_result.scene_description) {
        free(interpreter.last_result.scene_description);
    }
    if (interpreter.last_result.objects_detected) {
        free(interpreter.last_result.objects_detected);
    }
    if (interpreter.last_result.reasoning) {
        free(interpreter.last_result.reasoning);
    }
    if (interpreter.last_result.confidence_level) {
        free(interpreter.last_result.confidence_level);
    }

    interpreter.last_result = *result;

    // Duplicate strings for last_result
    if (result->scene_description) {
        interpreter.last_result.scene_description = strdup(result->scene_description);
    }
    if (result->objects_detected) {
        interpreter.last_result.objects_detected = strdup(result->objects_detected);
    }
    if (result->reasoning) {
        interpreter.last_result.reasoning = strdup(result->reasoning);
    }
    if (result->confidence_level) {
        interpreter.last_result.confidence_level = strdup(result->confidence_level);
    }

    // Free LLM response
    llm_free_response(&llm_response);

    if (err == ESP_OK) {
        interpreter.successful_analyses++;
        ESP_LOGI(TAG, "Vision analysis complete: %s with %s confidence",
                 motor_get_command_name(result->suggested_action),
                 result->confidence_level ? result->confidence_level : "unknown");
    } else {
        interpreter.failed_analyses++;
    }

    return err;
}

// Convert LLM response to motor commands
esp_err_t vision_parse_llm_response(const llm_response_t* llm_response,
                                    vision_result_t* result) {
    if (!llm_response || !result) {
        return ESP_ERR_INVALID_ARG;
    }

    // Extract scene description (first sentence or two)
    if (llm_response->text) {
        const char* first_period = strchr(llm_response->text, '.');
        if (first_period) {
            size_t len = first_period - llm_response->text + 1;
            result->scene_description = malloc(len + 1);
            if (result->scene_description) {
                strncpy(result->scene_description, llm_response->text, len);
                result->scene_description[len] = '\0';
            }
        } else {
            result->scene_description = strdup(llm_response->text);
        }
    }

    // Parse objects detected
    if (llm_response->objects_detected) {
        result->objects_detected = strdup(llm_response->objects_detected);
    } else {
        // Try to extract from text
        const char* objects_marker = strstr(llm_response->text, "objects:");
        if (!objects_marker) {
            objects_marker = strstr(llm_response->text, "see:");
        }
        if (objects_marker) {
            const char* end = strchr(objects_marker, '.');
            if (end) {
                size_t len = end - objects_marker;
                result->objects_detected = malloc(len + 1);
                if (result->objects_detected) {
                    strncpy(result->objects_detected, objects_marker, len);
                    result->objects_detected[len] = '\0';
                }
            }
        }
    }

    // Parse suggested action
    if (llm_response->movement_command) {
        result->suggested_action = parse_motor_command(llm_response->movement_command);
    } else {
        result->suggested_action = parse_motor_command(llm_response->text);
    }

    // Parse confidence level
    if (llm_response->confidence) {
        result->confidence_level = strdup(llm_response->confidence);
    } else {
        result->confidence_level = strdup("medium");
    }

    // Set speed based on confidence
    result->suggested_speed = parse_speed(result->confidence_level);

    // Set duration (default 2 seconds for movements)
    result->suggested_duration_ms = (result->suggested_action == MOTOR_CMD_STOP) ? 0 : 2000;

    // Extract reasoning
    const char* because = strstr(llm_response->text, "because");
    if (!because) {
        because = strstr(llm_response->text, "since");
    }
    if (!because) {
        because = strstr(llm_response->text, "due to");
    }
    if (because) {
        result->reasoning = strdup(because);
    } else {
        result->reasoning = strdup("Based on visual analysis");
    }

    // Detect obstacles
    result->obstacle_detected = llm_response->has_obstacles;
    result->path_clear = !llm_response->has_obstacles;

    return ESP_OK;
}

// Generate action from vision result
esp_err_t vision_get_motor_action(const vision_result_t* result,
                                  motor_command_t* command,
                                  int* speed, uint32_t* duration_ms) {
    if (!result || !command || !speed || !duration_ms) {
        return ESP_ERR_INVALID_ARG;
    }

    *command = result->suggested_action;
    *speed = result->suggested_speed;
    *duration_ms = result->suggested_duration_ms;

    // Safety checks
    if (result->obstacle_detected && *command == MOTOR_CMD_FORWARD) {
        ESP_LOGW(TAG, "Obstacle detected, overriding forward command to stop");
        *command = MOTOR_CMD_STOP;
        *duration_ms = 0;
    }

    return ESP_OK;
}

// Format vision result for Telegram message
char* vision_format_telegram_message(const vision_result_t* result) {
    static char message[1024];

    snprintf(message, sizeof(message),
             "*🤖 Vision Analysis Result*\n\n"
             "*Scene:* %s\n"
             "*Objects:* %s\n"
             "*Action:* %s\n"
             "*Confidence:* %s\n"
             "*Path:* %s\n"
             "*Reasoning:* %s",
             result->scene_description ? result->scene_description : "No description",
             result->objects_detected ? result->objects_detected : "None detected",
             motor_get_command_name(result->suggested_action),
             result->confidence_level ? result->confidence_level : "Unknown",
             result->path_clear ? "Clear ✅" : "Blocked ⛔",
             result->reasoning ? result->reasoning : "No reasoning provided");

    return message;
}

// Free vision result resources
void vision_free_result(vision_result_t* result) {
    if (result) {
        if (result->scene_description) {
            free(result->scene_description);
            result->scene_description = NULL;
        }
        if (result->objects_detected) {
            free(result->objects_detected);
            result->objects_detected = NULL;
        }
        if (result->reasoning) {
            free(result->reasoning);
            result->reasoning = NULL;
        }
        if (result->confidence_level) {
            free(result->confidence_level);
            result->confidence_level = NULL;
        }
    }
}

// Get interpreter status
vision_interpreter_t vision_get_status(void) {
    return interpreter;
}

// Cleanup vision interpreter
void vision_interpreter_cleanup(void) {
    interpreter.is_active = false;
    vision_free_result(&interpreter.last_result);
    ESP_LOGI(TAG, "Vision interpreter cleaned up");
}