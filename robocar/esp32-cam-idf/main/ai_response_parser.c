/**
 * @file ai_response_parser.c
 * @brief Parser for Claude AI responses implementation
 */

#include "ai_response_parser.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ai_parser";

const char* movement_to_command(const char* recommendation) {
    if (!recommendation) return "S";
    
    if (strcmp(recommendation, "forward") == 0) return "F";
    if (strcmp(recommendation, "backward") == 0) return "B";
    if (strcmp(recommendation, "left") == 0) return "L";
    if (strcmp(recommendation, "right") == 0) return "R";
    if (strcmp(recommendation, "rotate_cw") == 0) return "C";
    if (strcmp(recommendation, "rotate_ccw") == 0) return "W";
    if (strcmp(recommendation, "stop") == 0) return "S";
    
    return "S"; // Default to stop
}

bool parse_ai_response(const char* response_text, ai_command_t* command) {
    if (!response_text || !command) {
        return false;
    }

    // Initialize command structure
    memset(command, 0, sizeof(ai_command_t));
    strcpy(command->movement_command, "S");
    command->pan_angle = 90;
    command->tilt_angle = 90;

    ESP_LOGI(TAG, "Parsing AI response: %.200s", response_text);

    // Parse outer response structure first
    cJSON *root = cJSON_Parse(response_text);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse main JSON response");
        return false;
    }

    // Extract content from Claude's response format
    cJSON *content_array = cJSON_GetObjectItem(root, "content");
    if (!content_array || !cJSON_IsArray(content_array)) {
        ESP_LOGE(TAG, "No content array found in response");
        cJSON_Delete(root);
        return false;
    }

    cJSON *content_item = cJSON_GetArrayItem(content_array, 0);
    if (!content_item) {
        ESP_LOGE(TAG, "No content item found");
        cJSON_Delete(root);
        return false;
    }

    cJSON *text_item = cJSON_GetObjectItem(content_item, "text");
    if (!text_item || !cJSON_IsString(text_item)) {
        ESP_LOGE(TAG, "No text content found");
        cJSON_Delete(root);
        return false;
    }

    const char *content_text = cJSON_GetStringValue(text_item);
    if (!content_text) {
        ESP_LOGE(TAG, "Empty text content");
        cJSON_Delete(root);
        return false;
    }

    ESP_LOGI(TAG, "Extracted content: %.200s", content_text);

    // Find JSON object within the content
    const char *json_start = strchr(content_text, '{');
    const char *json_end = strrchr(content_text, '}');
    
    if (!json_start || !json_end || json_end <= json_start) {
        ESP_LOGE(TAG, "No JSON object found in content");
        cJSON_Delete(root);
        return false;
    }

    // Extract JSON substring
    size_t json_len = json_end - json_start + 1;
    char *json_str = malloc(json_len + 1);
    if (!json_str) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON parsing");
        cJSON_Delete(root);
        return false;
    }
    
    strncpy(json_str, json_start, json_len);
    json_str[json_len] = '\0';

    ESP_LOGI(TAG, "Extracted JSON: %s", json_str);

    // Parse the extracted JSON
    cJSON *json_obj = cJSON_Parse(json_str);
    free(json_str);
    cJSON_Delete(root);

    if (!json_obj) {
        ESP_LOGE(TAG, "Failed to parse extracted JSON");
        return false;
    }

    bool success = false;

    // Parse movement recommendation
    cJSON *recommendation = cJSON_GetObjectItem(json_obj, "recommendation");
    if (recommendation && cJSON_IsString(recommendation)) {
        const char *rec_str = cJSON_GetStringValue(recommendation);
        const char *cmd = movement_to_command(rec_str);
        strncpy(command->movement_command, cmd, sizeof(command->movement_command) - 1);
        command->has_movement = true;
        ESP_LOGI(TAG, "Movement: %s -> %s", rec_str, cmd);
        success = true;
    }

    // Parse sound command
    cJSON *sound = cJSON_GetObjectItem(json_obj, "sound");
    if (sound && cJSON_IsString(sound)) {
        const char *sound_str = cJSON_GetStringValue(sound);
        
        if (strcmp(sound_str, "beep") == 0) {
            strcpy(command->sound_command, "SB");
        } else if (strcmp(sound_str, "melody") == 0) {
            strcpy(command->sound_command, "SM");
        } else if (strcmp(sound_str, "alert") == 0) {
            strcpy(command->sound_command, "SA");
        } else if (strncmp(sound_str, "custom:", 7) == 0) {
            snprintf(command->sound_command, sizeof(command->sound_command), "SC%s", sound_str + 6);
        } else if (strncmp(sound_str, "morse:", 6) == 0) {
            snprintf(command->sound_command, sizeof(command->sound_command), "MO%s", sound_str + 5);
        } else if (strncmp(sound_str, "rtttl:", 6) == 0) {
            snprintf(command->sound_command, sizeof(command->sound_command), "RT%s", sound_str + 5);
        }
        
        command->has_sound = (strlen(command->sound_command) > 0);
        if (command->has_sound) {
            ESP_LOGI(TAG, "Sound: %s -> %s", sound_str, command->sound_command);
        }
    }

    // Parse pan angle
    cJSON *pan = cJSON_GetObjectItem(json_obj, "pan");
    if (pan && cJSON_IsNumber(pan)) {
        int pan_val = cJSON_GetNumberValue(pan);
        if (pan_val >= 0 && pan_val <= 180) {
            command->pan_angle = pan_val;
            command->has_pan = true;
            ESP_LOGI(TAG, "Pan angle: %d", pan_val);
        }
    }

    // Parse tilt angle
    cJSON *tilt = cJSON_GetObjectItem(json_obj, "tilt");
    if (tilt && cJSON_IsNumber(tilt)) {
        int tilt_val = cJSON_GetNumberValue(tilt);
        if (tilt_val >= 0 && tilt_val <= 180) {
            command->tilt_angle = tilt_val;
            command->has_tilt = true;
            ESP_LOGI(TAG, "Tilt angle: %d", tilt_val);
        }
    }

    // Parse display message
    cJSON *display = cJSON_GetObjectItem(json_obj, "display");
    if (display && cJSON_IsString(display)) {
        const char *display_str = cJSON_GetStringValue(display);
        strncpy(command->display_message, display_str, sizeof(command->display_message) - 1);
        command->display_message[sizeof(command->display_message) - 1] = '\0';
        command->has_display = true;
        ESP_LOGI(TAG, "Display message: %s", display_str);
    }

    // Parse display line (optional, defaults to 6)
    cJSON *display_line = cJSON_GetObjectItem(json_obj, "display_line");
    if (display_line && cJSON_IsNumber(display_line)) {
        int line_val = cJSON_GetNumberValue(display_line);
        if (line_val >= 0 && line_val <= 7) {
            command->display_line = line_val;
        } else {
            command->display_line = 6; // Default line
        }
    } else {
        command->display_line = 6; // Default to line 6 for Claude messages
    }

    cJSON_Delete(json_obj);
    return success;
}