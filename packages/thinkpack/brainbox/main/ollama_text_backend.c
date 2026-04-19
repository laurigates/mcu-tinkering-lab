/**
 * @file ollama_text_backend.c
 * @brief Ollama text-query backend for ThinkPack Brainbox.
 *
 * POSTs a text prompt to {base_url}/api/generate with stream:false and
 * extracts the "response" field from the JSON reply.
 *
 * URL resolution order:
 *   1. config->api_url (non-NULL and non-empty)
 *   2. OLLAMA_BASE_URL macro from credentials.h (non-empty)
 *   3. mDNS discovery via ollama_discovery_find_server()
 *
 * NOTE: s_response_buffer and s_response_len are accessed only from the
 * single HTTP event handler callback, which is invoked synchronously during
 * esp_http_client_perform(). Do not call ollama_query_text() from multiple
 * tasks concurrently.
 */

#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "ollama_discovery.h"
#include "thinkpack_ai.h"

/* credentials.h supplies OLLAMA_BASE_URL (may be an empty string literal). */
#ifdef __has_include
#if __has_include("credentials.h")
#include "credentials.h"
#endif
#endif

#ifndef OLLAMA_BASE_URL
#define OLLAMA_BASE_URL ""
#endif

static const char *TAG = "ollama_text";

#define OLLAMA_DEFAULT_MODEL "llama3.2:3b"
#define OLLAMA_RESPONSE_BUFFER_SIZE 8192
#define OLLAMA_TIMEOUT_MS 45000
/* Max length for "http://host:port/api/generate\0" */
#define OLLAMA_URL_BUF_SIZE 512

static char *s_response_buffer = NULL;
static size_t s_response_len = 0;
static char s_api_url[OLLAMA_URL_BUF_SIZE];
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
                size_t available = (OLLAMA_RESPONSE_BUFFER_SIZE - 1) - s_response_len;
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
 * @brief Resolve the Ollama /api/generate URL and store it in s_api_url.
 *
 * Tries config->api_url, then OLLAMA_BASE_URL, then mDNS discovery.
 * Returns ESP_OK with s_api_url populated, or ESP_FAIL if all sources fail.
 */
static esp_err_t resolve_api_url(const ai_config_t *config)
{
    /* 1. Caller-supplied URL. */
    if (config->api_url && strlen(config->api_url) > 0) {
        int r = snprintf(s_api_url, sizeof(s_api_url), "%s/api/generate", config->api_url);
        if (r > 0 && (size_t)r < sizeof(s_api_url)) {
            ESP_LOGI(TAG, "Using config api_url: %s", s_api_url);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "config->api_url is too long");
        return ESP_ERR_INVALID_ARG;
    }

    /* 2. Compile-time OLLAMA_BASE_URL from credentials.h. */
    if (strlen(OLLAMA_BASE_URL) > 0) {
        int r = snprintf(s_api_url, sizeof(s_api_url), "%s/api/generate", OLLAMA_BASE_URL);
        if (r > 0 && (size_t)r < sizeof(s_api_url)) {
            ESP_LOGI(TAG, "Using OLLAMA_BASE_URL from credentials.h: %s", s_api_url);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "OLLAMA_BASE_URL is too long");
        return ESP_ERR_INVALID_ARG;
    }

    /* 3. mDNS discovery. */
    ESP_LOGI(TAG, "No static URL configured, attempting mDNS discovery for _ollama._tcp.local");
    ollama_discovery_config_t disc_cfg = {
        .srv_record = "_ollama._tcp.local",
        .fallback_url = NULL,
        .timeout_ms = 5000,
        .use_mdns = true,
    };
    esp_err_t err = ollama_discovery_init(&disc_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ollama_discovery_init failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    char base_url[256];
    err = ollama_discovery_find_server(base_url, sizeof(base_url));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS discovery found no Ollama server: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    int r = snprintf(s_api_url, sizeof(s_api_url), "%s/api/generate", base_url);
    if (r < 0 || (size_t)r >= sizeof(s_api_url)) {
        ESP_LOGE(TAG, "Discovered URL too long for buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Discovered Ollama server: %s", s_api_url);
    return ESP_OK;
}

static esp_err_t ollama_init(const ai_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "config must not be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->model && strlen(config->model) > 0) {
        s_model = config->model;
    } else {
        strncpy(s_model_buf, OLLAMA_DEFAULT_MODEL, sizeof(s_model_buf) - 1);
        s_model_buf[sizeof(s_model_buf) - 1] = '\0';
        s_model = s_model_buf;
    }

    esp_err_t err = resolve_api_url(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to resolve Ollama API URL");
        return err;
    }

    s_response_buffer = malloc(OLLAMA_RESPONSE_BUFFER_SIZE);
    if (!s_response_buffer) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Ollama text backend initialised (model=%s, url=%s)", s_model, s_api_url);
    return ESP_OK;
}

static esp_err_t ollama_query_text(const char *prompt, ai_response_t *response)
{
    if (!prompt || !response || strlen(s_api_url) == 0 || !s_model) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Sending text query to Ollama (%zu chars)", strlen(prompt));

    /* Build JSON body: {"model":"...","prompt":"...","stream":false} */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", s_model);
    cJSON_AddStringToObject(root, "prompt", prompt);
    cJSON_AddFalseToObject(root, "stream");

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to serialise JSON request");
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t http_cfg = {
        .url = s_api_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = OLLAMA_TIMEOUT_MS,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        free(json_string);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    s_response_len = 0;
    s_response_buffer[0] = '\0';

    esp_http_client_set_post_field(client, json_string, strlen(json_string));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP status=%d, response_len=%zu", status, s_response_len);

    if (err == ESP_OK && status == 200) {
        /* Parse: {"model":"...","response":"...","done":true,...} */
        cJSON *parsed = cJSON_Parse(s_response_buffer);
        if (parsed) {
            cJSON *resp_item = cJSON_GetObjectItem(parsed, "response");
            if (cJSON_IsString(resp_item)) {
                const char *text = cJSON_GetStringValue(resp_item);
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
                    ESP_LOGE(TAG, "Null string in Ollama response field");
                    err = ESP_FAIL;
                }
            } else {
                ESP_LOGE(TAG, "response field missing or not a string in Ollama reply");
                err = ESP_FAIL;
            }
            cJSON_Delete(parsed);
        } else {
            ESP_LOGE(TAG, "Failed to parse Ollama JSON response");
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

static void ollama_free_response(ai_response_t *response)
{
    if (response && response->response_text) {
        free(response->response_text);
        response->response_text = NULL;
        response->response_length = 0;
    }
}

static void ollama_deinit(void)
{
    memset(s_api_url, 0, sizeof(s_api_url));
    s_model = NULL;
    if (s_response_buffer) {
        free(s_response_buffer);
        s_response_buffer = NULL;
    }
    s_response_len = 0;
    ESP_LOGI(TAG, "Ollama text backend deinitialized");
}

const thinkpack_ai_backend_t ollama_text_backend = {
    .init = ollama_init,
    .query_text = ollama_query_text,
    .free_response = ollama_free_response,
    .deinit = ollama_deinit,
};
