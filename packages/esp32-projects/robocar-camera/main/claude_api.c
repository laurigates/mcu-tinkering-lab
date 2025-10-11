/**
 * @file claude_backend.c
 * @brief Claude backend for the AI interface
 */

#include "claude_backend.h"
#include "ai_backend.h"
#include "base64.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "claude_backend";
static const ai_config_t* s_config = NULL;
static char* s_response_buffer = NULL;
static size_t s_response_len = 0;

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (s_response_buffer && evt->data && evt->data_len > 0) {
                // Ensure we don't overflow the buffer (4096 bytes total, -1 for null terminator)
                size_t available_space = 4095 - s_response_len;
                size_t bytes_to_copy = (evt->data_len <= available_space) ? evt->data_len : available_space;
                
                if (bytes_to_copy > 0) {
                    memcpy(s_response_buffer + s_response_len, evt->data, bytes_to_copy);
                    s_response_len += bytes_to_copy;
                    s_response_buffer[s_response_len] = '\0';
                    
                    if (bytes_to_copy < evt->data_len) {
                        ESP_LOGW(TAG, "HTTP response truncated: received %d bytes, only %zu bytes available", 
                                evt->data_len, available_space);
                    }
                } else {
                    ESP_LOGW(TAG, "HTTP response buffer full, discarding %d bytes", evt->data_len);
                }
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

static esp_err_t claude_init(const ai_config_t* config) {
    if (!config || !config->api_key || !config->api_url || !config->model) {
        ESP_LOGE(TAG, "Claude API key, URL, and model are required");
        return ESP_ERR_INVALID_ARG;
    }
    s_config = config;

    s_response_buffer = malloc(4096);
    if (!s_response_buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Claude backend initialized with model %s", s_config->model);
    return ESP_OK;
}

static esp_err_t claude_analyze_image(const uint8_t* image_data, size_t image_size, ai_response_t* response) {
    if (!image_data || !response || !s_config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Analyzing image with Claude (size: %zu bytes)", image_size);

    // Encode image to base64
    char* base64_image = base64_encode_alloc(image_data, image_size);
    if (!base64_image) {
        ESP_LOGE(TAG, "Failed to encode image to base64");
        return ESP_ERR_NO_MEM;
    }

    // Create JSON request
    cJSON *json = cJSON_CreateObject();
    cJSON *model = cJSON_CreateString(s_config->model);
    cJSON *max_tokens = cJSON_CreateNumber(512);
    cJSON *messages = cJSON_CreateArray();
    cJSON *message = cJSON_CreateObject();
    cJSON *role = cJSON_CreateString("user");
    cJSON *content = cJSON_CreateArray();
    
    // Add text content
    cJSON *text_content = cJSON_CreateObject();
    cJSON *text_type = cJSON_CreateString("text");
    cJSON *text_value = cJSON_CreateString(
        "You control a robot with a camera. Analyze this image and help the robot navigate its environment. "
        "Identify objects and their positions (left, center, right) in the scene. "
        "Determine if there are obstacles to avoid or clear paths. "
        "Respond with ONLY a simple JSON object with these keys:\n"
        "- objects: array of detected objects with name and position (e.g., [{\"name\":\"cup\", \"position\":\"left\"}])\n"
        "- recommendation: one of [forward, backward, left, right, rotate_cw, rotate_ccw, stop]\n"
        "- sound: optional sound command [beep, melody, alert, custom:freq:duration, morse:TEXT:message, rtttl:name]\n"
        "- pan: optional camera pan angle (0-180, default 90)\n"
        "- tilt: optional camera tilt angle (0-180, default 90)\n"
        "Respond ONLY with the JSON object, no explanations."
    );
    cJSON_AddItemToObject(text_content, "type", text_type);
    cJSON_AddItemToObject(text_content, "text", text_value);
    cJSON_AddItemToArray(content, text_content);
    
    // Add image content
    cJSON *image_content = cJSON_CreateObject();
    cJSON *image_type = cJSON_CreateString("image");
    cJSON *source = cJSON_CreateObject();
    cJSON *source_type = cJSON_CreateString("base64");
    cJSON *media_type = cJSON_CreateString("image/jpeg");
    cJSON *data = cJSON_CreateString(base64_image);
    
    cJSON_AddItemToObject(source, "type", source_type);
    cJSON_AddItemToObject(source, "media_type", media_type);
    cJSON_AddItemToObject(source, "data", data);
    cJSON_AddItemToObject(image_content, "type", image_type);
    cJSON_AddItemToObject(image_content, "source", source);
    cJSON_AddItemToArray(content, image_content);
    
    cJSON_AddItemToObject(message, "role", role);
    cJSON_AddItemToObject(message, "content", content);
    cJSON_AddItemToArray(messages, message);
    
    cJSON_AddItemToObject(json, "model", model);
    cJSON_AddItemToObject(json, "max_tokens", max_tokens);
    cJSON_AddItemToObject(json, "messages", messages);
    
    char *json_string = cJSON_Print(json);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to create JSON request");
        free(base64_image);
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "JSON request size: %zu bytes", strlen(json_string));

    // Configure HTTP client
    esp_http_client_config_t config = {
        .url = s_config->api_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 30000,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        free(base64_image);
        free(json_string);
        cJSON_Delete(json);
        return ESP_FAIL;
    }

    // Set headers
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-api-key", s_config->api_key);
    esp_http_client_set_header(client, "anthropic-version", "2023-06-01");

    // Reset response buffer
    s_response_len = 0;
    if (s_response_buffer) {
        s_response_buffer[0] = '\0';
    }

    // Send request
    esp_http_client_set_post_field(client, json_string, strlen(json_string));
    esp_err_t err = esp_http_client_perform(client);
    
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP Status: %d, Response length: %zu", status_code, s_response_len);

    if (err == ESP_OK && status_code == 200) {
        cJSON* root = cJSON_Parse(s_response_buffer);
        if (root) {
            cJSON* content_array = cJSON_GetObjectItem(root, "content");
            if (cJSON_IsArray(content_array) && cJSON_GetArraySize(content_array) > 0) {
                cJSON* first_item = cJSON_GetArrayItem(content_array, 0);
                cJSON* text_item = cJSON_GetObjectItem(first_item, "text");
                if (cJSON_IsString(text_item)) {
                    const char* response_str = cJSON_GetStringValue(text_item);
                    size_t len = strlen(response_str);
                    response->response_text = malloc(len + 1);
                    if (response->response_text) {
                        strcpy(response->response_text, response_str);
                        response->response_length = len;
                        ESP_LOGI(TAG, "Successfully parsed Claude response");
                    } else {
                        err = ESP_ERR_NO_MEM;
                    }
                }
            }
            cJSON_Delete(root);
        } else {
            ESP_LOGE(TAG, "Failed to parse Claude JSON response");
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s, status: %d", esp_err_to_name(err), status_code);
        if (s_response_len > 0) {
            ESP_LOGE(TAG, "Error response: %.*s", (int)s_response_len, s_response_buffer);
        }
        err = ESP_FAIL;
    }

    // Cleanup
    esp_http_client_cleanup(client);
    free(base64_image);
    free(json_string);
    cJSON_Delete(json);

    return err;
}

static void claude_free_response(ai_response_t* response) {
    if (response && response->response_text) {
        free(response->response_text);
        response->response_text = NULL;
        response->response_length = 0;
    }
}

static void claude_deinit(void) {
    s_config = NULL;
    if (s_response_buffer) {
        free(s_response_buffer);
        s_response_buffer = NULL;
    }
    ESP_LOGI(TAG, "Claude backend deinitialized");
}

static const ai_backend_t s_claude_backend = {
    .init = claude_init,
    .analyze_image = claude_analyze_image,
    .free_response = claude_free_response,
    .deinit = claude_deinit,
};

const ai_backend_t* claude_backend_get(void) {
    return &s_claude_backend;
}