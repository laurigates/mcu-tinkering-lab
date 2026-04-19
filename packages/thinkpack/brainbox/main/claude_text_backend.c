/**
 * @file claude_text_backend.c
 * @brief Claude API text-query backend for ThinkPack Brainbox.
 *
 * POSTs a text-only message to https://api.anthropic.com/v1/messages and
 * extracts response.content[0].text from the JSON reply.
 *
 * NOTE: s_response_buffer and s_response_len are accessed only from the single
 * HTTP event handler callback, which is invoked synchronously during
 * esp_http_client_perform(). Do not call claude_query_text() from multiple
 * tasks concurrently.
 */

#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "thinkpack_ai.h"

static const char *TAG = "claude_text";

#define CLAUDE_DEFAULT_API_URL "https://api.anthropic.com/v1/messages"
#define CLAUDE_DEFAULT_MODEL "claude-sonnet-4-6"
#define CLAUDE_MAX_TOKENS 1024
#define CLAUDE_RESPONSE_BUFFER_SIZE 8192
#define CLAUDE_TIMEOUT_MS 30000

static const ai_config_t *s_config = NULL;
static char *s_response_buffer = NULL;
static size_t s_response_len = 0;

/* Filled from config on init; points into s_url_buf when api_url is NULL. */
static char s_url_buf[128];
static char s_model_buf[64];
static const char *s_api_url = NULL;
static const char *s_model = NULL;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
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
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key,
                     evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (s_response_buffer && evt->data && evt->data_len > 0) {
                size_t available = (CLAUDE_RESPONSE_BUFFER_SIZE - 1) - s_response_len;
                size_t to_copy =
                    ((size_t)evt->data_len <= available) ? (size_t)evt->data_len : available;
                if (to_copy > 0) {
                    memcpy(s_response_buffer + s_response_len, evt->data, to_copy);
                    s_response_len += to_copy;
                    s_response_buffer[s_response_len] = '\0';
                    // cppcheck-suppress knownConditionTrueFalse // true when buffer fills partially
                    if (to_copy < (size_t)evt->data_len) {
                        ESP_LOGW(TAG, "Response truncated: %d bytes received, %zu available",
                                 evt->data_len, available);
                    }
                } else {
                    ESP_LOGW(TAG, "Response buffer full, discarding %d bytes", evt->data_len);
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

static esp_err_t claude_init(const ai_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "config must not be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!config->api_key || strlen(config->api_key) == 0) {
        ESP_LOGE(TAG, "Claude API key is required");
        return ESP_ERR_INVALID_ARG;
    }

    s_config = config;

    /* Resolve URL — fall back to built-in default when caller passes NULL. */
    if (config->api_url && strlen(config->api_url) > 0) {
        s_api_url = config->api_url;
    } else {
        strncpy(s_url_buf, CLAUDE_DEFAULT_API_URL, sizeof(s_url_buf) - 1);
        s_url_buf[sizeof(s_url_buf) - 1] = '\0';
        s_api_url = s_url_buf;
    }

    /* Resolve model. */
    if (config->model && strlen(config->model) > 0) {
        s_model = config->model;
    } else {
        strncpy(s_model_buf, CLAUDE_DEFAULT_MODEL, sizeof(s_model_buf) - 1);
        s_model_buf[sizeof(s_model_buf) - 1] = '\0';
        s_model = s_model_buf;
    }

    s_response_buffer = malloc(CLAUDE_RESPONSE_BUFFER_SIZE);
    if (!s_response_buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Claude text backend initialised (model=%s, url=%s)", s_model, s_api_url);
    return ESP_OK;
}

static esp_err_t claude_query_text(const char *prompt, ai_response_t *response)
{
    if (!prompt || !response || !s_config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending text query to Claude (%zu chars)", strlen(prompt));

    /* Build JSON body:
     * {
     *   "model": "...",
     *   "max_tokens": 1024,
     *   "messages": [{"role": "user", "content": "<prompt>"}]
     * }
     */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", s_model);
    cJSON_AddNumberToObject(root, "max_tokens", CLAUDE_MAX_TOKENS);

    cJSON *messages = cJSON_AddArrayToObject(root, "messages");
    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "role", "user");
    cJSON_AddStringToObject(msg, "content", prompt);
    cJSON_AddItemToArray(messages, msg);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to serialise JSON request");
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t http_cfg = {
        .url = s_api_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = CLAUDE_TIMEOUT_MS,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        free(json_string);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-api-key", s_config->api_key);
    esp_http_client_set_header(client, "anthropic-version", "2023-06-01");

    s_response_len = 0;
    s_response_buffer[0] = '\0';

    esp_http_client_set_post_field(client, json_string, strlen(json_string));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP status=%d, response_len=%zu", status, s_response_len);

    if (err == ESP_OK && status == 200) {
        /* Parse: {"content":[{"type":"text","text":"..."}], ...} */
        cJSON *parsed = cJSON_Parse(s_response_buffer);
        if (parsed) {
            cJSON *content_arr = cJSON_GetObjectItem(parsed, "content");
            if (cJSON_IsArray(content_arr) && cJSON_GetArraySize(content_arr) > 0) {
                cJSON *first = cJSON_GetArrayItem(content_arr, 0);
                cJSON *text_item = cJSON_GetObjectItem(first, "text");
                if (cJSON_IsString(text_item)) {
                    const char *text = cJSON_GetStringValue(text_item);
                    if (text) {
                        size_t len = strlen(text);
                        response->response_text = malloc(len + 1);
                        if (response->response_text) {
                            memcpy(response->response_text, text, len + 1);
                            response->response_length = len;
                            ESP_LOGI(TAG, "Response parsed successfully (%zu chars)", len);
                        } else {
                            ESP_LOGE(TAG, "Failed to allocate response text");
                            err = ESP_ERR_NO_MEM;
                        }
                    } else {
                        ESP_LOGE(TAG, "Null string in content[0].text");
                        err = ESP_FAIL;
                    }
                } else {
                    ESP_LOGE(TAG, "content[0].text field missing or not a string");
                    err = ESP_FAIL;
                }
            } else {
                ESP_LOGE(TAG, "content array missing or empty in Claude response");
                err = ESP_FAIL;
            }
            cJSON_Delete(parsed);
        } else {
            ESP_LOGE(TAG, "Failed to parse Claude JSON response");
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s, status=%d", esp_err_to_name(err), status);
        if (s_response_len > 0) {
            ESP_LOGE(TAG, "Error body: %.*s", (int)s_response_len, s_response_buffer);
        }
        err = ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    free(json_string);
    return err;
}

static void claude_free_response(ai_response_t *response)
{
    if (response && response->response_text) {
        free(response->response_text);
        response->response_text = NULL;
        response->response_length = 0;
    }
}

static void claude_deinit(void)
{
    s_config = NULL;
    s_api_url = NULL;
    s_model = NULL;
    if (s_response_buffer) {
        free(s_response_buffer);
        s_response_buffer = NULL;
    }
    s_response_len = 0;
    ESP_LOGI(TAG, "Claude text backend deinitialized");
}

const thinkpack_ai_backend_t claude_text_backend = {
    .init = claude_init,
    .query_text = claude_query_text,
    .free_response = claude_free_response,
    .deinit = claude_deinit,
};
