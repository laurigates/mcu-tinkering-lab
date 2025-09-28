#include "llm_client.h"
#include "config.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "LLM_CLIENT";

static llm_config_t llm_config = {0};
static bool is_initialized = false;

// Base64 encode helper
static char* base64_encode(const uint8_t* data, size_t len) {
    size_t output_len = 0;
    mbedtls_base64_encode(NULL, 0, &output_len, data, len);

    char* encoded = malloc(output_len + 1);
    if (encoded) {
        mbedtls_base64_encode((unsigned char*)encoded, output_len, &output_len, data, len);
        encoded[output_len] = '\0';
    }
    return encoded;
}

// HTTP event handler
static esp_err_t http_event_handler(esp_http_client_event_t* evt) {
    static char* output_buffer;
    static int output_len;

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (output_buffer == NULL) {
                    output_buffer = (char*) malloc(esp_http_client_get_content_length(evt->client));
                    output_len = 0;
                    if (output_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
                output_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            if (output_buffer != NULL) {
                evt->data = output_buffer;
                evt->data_len = output_len;
                output_buffer = NULL;
                output_len = 0;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
                output_len = 0;
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

// Send request to Claude API
static esp_err_t send_claude_request(const char* prompt, const char* image_base64,
                                     char** response_text) {
    esp_err_t err = ESP_OK;

    // Build JSON request
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", llm_config.model);
    cJSON_AddNumberToObject(root, "max_tokens", 1024);

    cJSON* messages = cJSON_CreateArray();
    cJSON* message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "role", "user");

    cJSON* content = cJSON_CreateArray();

    // Add text prompt
    cJSON* text_content = cJSON_CreateObject();
    cJSON_AddStringToObject(text_content, "type", "text");
    cJSON_AddStringToObject(text_content, "text", prompt);
    cJSON_AddItemToArray(content, text_content);

    // Add image if provided
    if (image_base64) {
        cJSON* image_content = cJSON_CreateObject();
        cJSON_AddStringToObject(image_content, "type", "image");
        cJSON* source = cJSON_CreateObject();
        cJSON_AddStringToObject(source, "type", "base64");
        cJSON_AddStringToObject(source, "media_type", "image/jpeg");
        cJSON_AddStringToObject(source, "data", image_base64);
        cJSON_AddItemToObject(image_content, "source", source);
        cJSON_AddItemToArray(content, image_content);
    }

    cJSON_AddItemToObject(message, "content", content);
    cJSON_AddItemToArray(messages, message);
    cJSON_AddItemToObject(root, "messages", messages);

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = CLAUDE_API_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = llm_config.timeout_ms,
        .event_handler = http_event_handler,
        .buffer_size = HTTP_BUFFER_SIZE,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // Set headers
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-api-key", llm_config.api_key);
    esp_http_client_set_header(client, "anthropic-version", "2023-06-01");

    esp_http_client_set_post_field(client, json_str, strlen(json_str));

    // Perform request
    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            int content_length = esp_http_client_get_content_length(client);
            char* buffer = malloc(content_length + 1);

            if (buffer) {
                esp_http_client_read(client, buffer, content_length);
                buffer[content_length] = '\0';

                // Parse response
                cJSON* response = cJSON_Parse(buffer);
                if (response) {
                    cJSON* content_item = cJSON_GetObjectItem(response, "content");
                    if (content_item && cJSON_IsArray(content_item)) {
                        cJSON* first = cJSON_GetArrayItem(content_item, 0);
                        if (first) {
                            cJSON* text = cJSON_GetObjectItem(first, "text");
                            if (text && cJSON_IsString(text)) {
                                *response_text = strdup(text->valuestring);
                            }
                        }
                    }
                    cJSON_Delete(response);
                }
                free(buffer);
            }
        } else {
            ESP_LOGE(TAG, "HTTP request failed with status: %d", status_code);
            err = ESP_FAIL;
        }
    }

    free(json_str);
    esp_http_client_cleanup(client);

    return err;
}

// Send request to Ollama API
static esp_err_t send_ollama_request(const char* prompt, const char* image_base64,
                                     char** response_text) {
    esp_err_t err = ESP_OK;

    // Build JSON request
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", llm_config.model);
    cJSON_AddStringToObject(root, "prompt", prompt);

    if (image_base64) {
        cJSON* images = cJSON_CreateArray();
        cJSON_AddItemToArray(images, cJSON_CreateString(image_base64));
        cJSON_AddItemToObject(root, "images", images);
    }

    cJSON_AddBoolToObject(root, "stream", false);

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // Build URL
    char url[256];
    snprintf(url, sizeof(url), "%s/api/generate", llm_config.server_url);

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = llm_config.timeout_ms,
        .buffer_size = HTTP_BUFFER_SIZE,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_str, strlen(json_str));

    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        char* buffer = malloc(content_length + 1);

        if (buffer) {
            esp_http_client_read(client, buffer, content_length);
            buffer[content_length] = '\0';

            // Parse response
            cJSON* response = cJSON_Parse(buffer);
            if (response) {
                cJSON* response_field = cJSON_GetObjectItem(response, "response");
                if (response_field && cJSON_IsString(response_field)) {
                    *response_text = strdup(response_field->valuestring);
                }
                cJSON_Delete(response);
            }
            free(buffer);
        }
    }

    free(json_str);
    esp_http_client_cleanup(client);

    return err;
}

// Initialize LLM client
esp_err_t llm_client_init(llm_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&llm_config, config, sizeof(llm_config_t));

    if (config->api_key) {
        llm_config.api_key = strdup(config->api_key);
    }
    if (config->server_url) {
        llm_config.server_url = strdup(config->server_url);
    }
    if (config->model) {
        llm_config.model = strdup(config->model);
    }

    if (llm_config.timeout_ms == 0) {
        llm_config.timeout_ms = 30000;  // Default 30 seconds
    }

    is_initialized = true;
    ESP_LOGI(TAG, "LLM client initialized with backend: %s",
             llm_config.backend_type == LLM_BACKEND_CLAUDE ? "Claude" : "Ollama");

    return ESP_OK;
}

// Analyze image with LLM
esp_err_t llm_analyze_image(const uint8_t* image_data, size_t image_size,
                            const char* prompt, llm_response_t* response) {
    if (!is_initialized || !image_data || !prompt || !response) {
        return ESP_ERR_INVALID_ARG;
    }

    // Base64 encode the image
    char* image_base64 = base64_encode(image_data, image_size);
    if (!image_base64) {
        ESP_LOGE(TAG, "Failed to encode image");
        return ESP_ERR_NO_MEM;
    }

    char* response_text = NULL;
    esp_err_t err;

    if (llm_config.backend_type == LLM_BACKEND_CLAUDE) {
        err = send_claude_request(prompt, image_base64, &response_text);
    } else {
        err = send_ollama_request(prompt, image_base64, &response_text);
    }

    free(image_base64);

    if (err == ESP_OK && response_text) {
        response->text = response_text;

        // Parse response for movement and confidence
        // This is simplified - in production, use more robust parsing
        if (strstr(response_text, "forward")) {
            response->movement_command = strdup("forward");
        } else if (strstr(response_text, "backward")) {
            response->movement_command = strdup("backward");
        } else if (strstr(response_text, "left")) {
            response->movement_command = strdup("left");
        } else if (strstr(response_text, "right")) {
            response->movement_command = strdup("right");
        } else {
            response->movement_command = strdup("stop");
        }

        if (strstr(response_text, "high")) {
            response->confidence = strdup("high");
        } else if (strstr(response_text, "medium")) {
            response->confidence = strdup("medium");
        } else {
            response->confidence = strdup("low");
        }

        response->has_obstacles = (strstr(response_text, "obstacle") != NULL ||
                                  strstr(response_text, "blocked") != NULL);

        response->error_code = 0;
    } else {
        response->error_code = -1;
    }

    return err;
}

// Send text query to LLM
esp_err_t llm_query_text(const char* query, llm_response_t* response) {
    if (!is_initialized || !query || !response) {
        return ESP_ERR_INVALID_ARG;
    }

    char* response_text = NULL;
    esp_err_t err;

    if (llm_config.backend_type == LLM_BACKEND_CLAUDE) {
        err = send_claude_request(query, NULL, &response_text);
    } else {
        err = send_ollama_request(query, NULL, &response_text);
    }

    if (err == ESP_OK && response_text) {
        response->text = response_text;
        response->error_code = 0;
    } else {
        response->error_code = -1;
    }

    return err;
}

// Process vision for robot control
esp_err_t llm_process_vision(const uint8_t* image_data, size_t image_size,
                             llm_response_t* response) {
    return llm_analyze_image(image_data, image_size, VISION_PROMPT_PREFIX, response);
}

// Free LLM response
void llm_free_response(llm_response_t* response) {
    if (response) {
        if (response->text) {
            free(response->text);
            response->text = NULL;
        }
        if (response->movement_command) {
            free(response->movement_command);
            response->movement_command = NULL;
        }
        if (response->confidence) {
            free(response->confidence);
            response->confidence = NULL;
        }
        if (response->objects_detected) {
            free(response->objects_detected);
            response->objects_detected = NULL;
        }
    }
}

// Cleanup LLM client
void llm_client_cleanup(void) {
    if (llm_config.api_key) {
        free(llm_config.api_key);
    }
    if (llm_config.server_url) {
        free(llm_config.server_url);
    }
    if (llm_config.model) {
        free(llm_config.model);
    }
    memset(&llm_config, 0, sizeof(llm_config_t));
    is_initialized = false;
}

// Helper to format vision prompt
char* llm_format_vision_prompt(const char* context, const char* additional_instructions) {
    size_t len = strlen(VISION_PROMPT_PREFIX) +
                (context ? strlen(context) : 0) +
                (additional_instructions ? strlen(additional_instructions) : 0) + 10;

    char* prompt = malloc(len);
    if (prompt) {
        strcpy(prompt, VISION_PROMPT_PREFIX);
        if (context) {
            strcat(prompt, "\nContext: ");
            strcat(prompt, context);
        }
        if (additional_instructions) {
            strcat(prompt, "\n");
            strcat(prompt, additional_instructions);
        }
    }
    return prompt;
}

// Parse movement commands from LLM response
esp_err_t llm_parse_movement(const char* response_text, char** movement, char** reason) {
    if (!response_text) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple keyword-based parsing
    if (strstr(response_text, "move forward") || strstr(response_text, "go forward")) {
        *movement = strdup("forward");
    } else if (strstr(response_text, "move backward") || strstr(response_text, "go back")) {
        *movement = strdup("backward");
    } else if (strstr(response_text, "turn left")) {
        *movement = strdup("left");
    } else if (strstr(response_text, "turn right")) {
        *movement = strdup("right");
    } else if (strstr(response_text, "stop") || strstr(response_text, "halt")) {
        *movement = strdup("stop");
    } else {
        *movement = strdup("stop");
    }

    // Extract reason (simplified)
    const char* because = strstr(response_text, "because");
    if (because) {
        *reason = strdup(because);
    } else {
        *reason = strdup(response_text);
    }

    return ESP_OK;
}