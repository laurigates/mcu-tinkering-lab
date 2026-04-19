/**
 * @file gemini_text_backend.c
 * @brief Google Gemini text-query backend for ThinkPack Brainbox.
 *
 * POSTs a text-only generateContent request to the Gemini REST API and
 * extracts candidates[0].content.parts[0].text from the JSON reply.
 *
 * NOTE: s_response_buffer and s_response_len are accessed only from the single
 * HTTP event handler callback, which is invoked synchronously during
 * esp_http_client_perform(). Do not call gemini_query_text() from multiple
 * tasks concurrently.
 */

#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "thinkpack_ai.h"

static const char *TAG = "gemini_text";

/* Base path; full URL is assembled as <base>/<model>:generateContent?key=<key>. */
#define GEMINI_DEFAULT_API_BASE "https://generativelanguage.googleapis.com/v1beta/models"
#define GEMINI_DEFAULT_MODEL "gemini-2.5-flash"
#define GEMINI_RESPONSE_BUFFER_SIZE 8192
#define GEMINI_TIMEOUT_MS 30000

static const ai_config_t *s_config = NULL;
static char *s_response_buffer = NULL;
static size_t s_response_len = 0;
static char s_model_buf[64];
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
                size_t available = (GEMINI_RESPONSE_BUFFER_SIZE - 1) - s_response_len;
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

/**
 * @brief Build the full Gemini generateContent URL with API key as query parameter.
 *
 * Format: <base>/<model>:generateContent?key=<apiKey>
 * When config->api_url is non-NULL, treat it as the base path instead.
 *
 * @param out        Output buffer.
 * @param out_size   Size of output buffer.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if buffer too small.
 */
static esp_err_t build_full_url(char *out, size_t out_size)
{
    const char *base = (s_config->api_url && strlen(s_config->api_url) > 0)
                           ? s_config->api_url
                           : GEMINI_DEFAULT_API_BASE;
    int ret =
        snprintf(out, out_size, "%s/%s:generateContent?key=%s", base, s_model, s_config->api_key);
    if (ret < 0 || (size_t)ret >= out_size) {
        ESP_LOGE(TAG, "Full Gemini URL too long for buffer");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t gemini_init(const ai_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "config must not be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!config->api_key || strlen(config->api_key) == 0) {
        ESP_LOGE(TAG, "Gemini API key is required");
        return ESP_ERR_INVALID_ARG;
    }

    s_config = config;

    if (config->model && strlen(config->model) > 0) {
        s_model = config->model;
    } else {
        strncpy(s_model_buf, GEMINI_DEFAULT_MODEL, sizeof(s_model_buf) - 1);
        s_model_buf[sizeof(s_model_buf) - 1] = '\0';
        s_model = s_model_buf;
    }

    s_response_buffer = malloc(GEMINI_RESPONSE_BUFFER_SIZE);
    if (!s_response_buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Gemini text backend initialised (model=%s)", s_model);
    return ESP_OK;
}

static esp_err_t gemini_query_text(const char *prompt, ai_response_t *response)
{
    if (!prompt || !response || !s_config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending text query to Gemini (%zu chars)", strlen(prompt));

    /* Build JSON body:
     * {
     *   "contents": [{"parts": [{"text": "<prompt>"}]}],
     *   "generationConfig": {"temperature": 0.5}
     * }
     */
    cJSON *root = cJSON_CreateObject();
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content_item = cJSON_CreateObject();
    cJSON *parts = cJSON_AddArrayToObject(content_item, "parts");
    cJSON *text_part = cJSON_CreateObject();
    cJSON_AddStringToObject(text_part, "text", prompt);
    cJSON_AddItemToArray(parts, text_part);
    cJSON_AddItemToArray(contents, content_item);

    cJSON *gen_config = cJSON_CreateObject();
    cJSON_AddNumberToObject(gen_config, "temperature", 0.5);
    cJSON_AddItemToObject(root, "generationConfig", gen_config);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to serialise JSON request");
        return ESP_ERR_NO_MEM;
    }

    /* Build URL into a stack buffer; max realistic length ~256 chars. */
    char full_url[512];
    esp_err_t url_err = build_full_url(full_url, sizeof(full_url));
    if (url_err != ESP_OK) {
        free(json_string);
        return url_err;
    }

    esp_http_client_config_t http_cfg = {
        .url = full_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = GEMINI_TIMEOUT_MS,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        free(json_string);
        return ESP_FAIL;
    }

    /* Gemini authenticates via the ?key= query parameter, not a header. */
    esp_http_client_set_header(client, "Content-Type", "application/json");

    s_response_len = 0;
    s_response_buffer[0] = '\0';

    esp_http_client_set_post_field(client, json_string, strlen(json_string));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP status=%d, response_len=%zu", status, s_response_len);

    if (err == ESP_OK && status == 200) {
        /* Parse: {"candidates":[{"content":{"parts":[{"text":"..."}]}}]} */
        cJSON *parsed = cJSON_Parse(s_response_buffer);
        if (parsed) {
            cJSON *candidates = cJSON_GetObjectItem(parsed, "candidates");
            if (cJSON_IsArray(candidates) && cJSON_GetArraySize(candidates) > 0) {
                cJSON *candidate = cJSON_GetArrayItem(candidates, 0);
                cJSON *content = cJSON_GetObjectItem(candidate, "content");
                cJSON *cparts = cJSON_GetObjectItem(content, "parts");
                if (cJSON_IsArray(cparts) && cJSON_GetArraySize(cparts) > 0) {
                    cJSON *first_part = cJSON_GetArrayItem(cparts, 0);
                    cJSON *text_item = cJSON_GetObjectItem(first_part, "text");
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
                            ESP_LOGE(TAG, "Null string in parts[0].text");
                            err = ESP_FAIL;
                        }
                    } else {
                        ESP_LOGE(TAG, "parts[0].text field missing or not a string");
                        err = ESP_FAIL;
                    }
                } else {
                    ESP_LOGE(TAG, "parts array missing in Gemini candidate content");
                    err = ESP_FAIL;
                }
            } else {
                ESP_LOGE(TAG, "candidates array missing or empty in Gemini response");
                err = ESP_FAIL;
            }
            cJSON_Delete(parsed);
        } else {
            ESP_LOGE(TAG, "Failed to parse Gemini JSON response");
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

static void gemini_free_response(ai_response_t *response)
{
    if (response && response->response_text) {
        free(response->response_text);
        response->response_text = NULL;
        response->response_length = 0;
    }
}

static void gemini_deinit(void)
{
    s_config = NULL;
    s_model = NULL;
    if (s_response_buffer) {
        free(s_response_buffer);
        s_response_buffer = NULL;
    }
    s_response_len = 0;
    ESP_LOGI(TAG, "Gemini text backend deinitialized");
}

const thinkpack_ai_backend_t gemini_text_backend = {
    .init = gemini_init,
    .query_text = gemini_query_text,
    .free_response = gemini_free_response,
    .deinit = gemini_deinit,
};
