/**
 * @file gemini_client.c
 * @brief HTTPS POST to Gemini Robotics-ER 1.5 with JSON response mode.
 *
 * The single-threaded assumption (a single consumer task) holds because this
 * function is only called from one FreeRTOS task (the Gemini worker) — never
 * invoke it concurrently.
 */

#include "gemini_client.h"

#include <stdlib.h>
#include <string.h>

#include "base64.h"
#include "cJSON.h"
#include "credentials.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gemini";

#define GEMINI_MODEL "gemini-robotics-er-1.5-preview"
#define GEMINI_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" GEMINI_MODEL ":generateContent"

#define GEMINI_RESPONSE_BUF_SIZE (16 * 1024)
#define GEMINI_TIMEOUT_MS 30000

// Using a thinking budget of 0 minimises latency; flip to >0 for higher
// accuracy at the cost of several seconds more.
#define GEMINI_THINKING_BUDGET 0

static const char *PROMPT =
    "Detect all prominent objects in this image. Return ONLY a JSON array. "
    "Each element must be an object with keys: \"label\" (short English noun) "
    "and \"box_2d\" ([ymin, xmin, ymax, xmax] integers normalized 0-1000). "
    "Return at most 15 objects. No prose, no markdown.";

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} response_accumulator_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        response_accumulator_t *acc = (response_accumulator_t *)evt->user_data;
        if (!acc || !acc->buf || !evt->data || evt->data_len <= 0) {
            return ESP_OK;
        }
        size_t avail = acc->cap - 1 - acc->len;
        size_t to_copy = ((size_t)evt->data_len < avail) ? (size_t)evt->data_len : avail;
        if (to_copy > 0) {
            memcpy(acc->buf + acc->len, evt->data, to_copy);
            acc->len += to_copy;
            acc->buf[acc->len] = '\0';
        }
        if (to_copy < (size_t)evt->data_len) {
            ESP_LOGW(TAG, "response truncated (discarded %d bytes)",
                     (int)((size_t)evt->data_len - to_copy));
        }
    }
    return ESP_OK;
}

static void write_error(char *out, size_t out_size, const char *msg)
{
    if (!out || out_size < 2) {
        return;
    }
    snprintf(out, out_size, "{\"error\":\"%s\"}", msg ? msg : "unknown");
}

esp_err_t gemini_detect(const uint8_t *jpeg, size_t jpeg_len, char *out_json, size_t out_json_size,
                        uint32_t *out_latency_ms)
{
    if (!jpeg || !jpeg_len || !out_json || out_json_size < 16) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(GEMINI_API_KEY) == 0) {
        write_error(out_json, out_json_size, "GEMINI_API_KEY unset");
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t t_start = esp_timer_get_time();

    // ---- Base64 encode ----
    char *b64 = base64_encode_alloc(jpeg, jpeg_len);
    if (!b64) {
        write_error(out_json, out_json_size, "base64 alloc failed");
        return ESP_ERR_NO_MEM;
    }

    // ---- Build request JSON ----
    cJSON *root = cJSON_CreateObject();
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");

    cJSON *text_part = cJSON_CreateObject();
    cJSON_AddStringToObject(text_part, "text", PROMPT);
    cJSON_AddItemToArray(parts, text_part);

    cJSON *image_part = cJSON_CreateObject();
    cJSON *inline_data = cJSON_AddObjectToObject(image_part, "inlineData");
    cJSON_AddStringToObject(inline_data, "mimeType", "image/jpeg");
    cJSON_AddStringToObject(inline_data, "data", b64);
    cJSON_AddItemToArray(parts, image_part);

    cJSON_AddItemToArray(contents, content);

    cJSON *gen_config = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON_AddStringToObject(gen_config, "responseMimeType", "application/json");
    cJSON *thinking = cJSON_AddObjectToObject(gen_config, "thinkingConfig");
    cJSON_AddNumberToObject(thinking, "thinkingBudget", GEMINI_THINKING_BUDGET);

    char *request_body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(b64);

    if (!request_body) {
        write_error(out_json, out_json_size, "request build failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "POST %s (body=%zu bytes, image=%zu bytes)", GEMINI_URL, strlen(request_body),
             jpeg_len);

    // ---- HTTP client ----
    response_accumulator_t acc = {
        .buf = malloc(GEMINI_RESPONSE_BUF_SIZE),
        .len = 0,
        .cap = GEMINI_RESPONSE_BUF_SIZE,
    };
    if (!acc.buf) {
        free(request_body);
        write_error(out_json, out_json_size, "response buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    acc.buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url = GEMINI_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = GEMINI_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .user_data = &acc,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_err_t err = ESP_FAIL;
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        write_error(out_json, out_json_size, "http client init failed");
        goto cleanup_noclient;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-goog-api-key", GEMINI_API_KEY);
    esp_http_client_set_post_field(client, request_body, strlen(request_body));

    err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    const int64_t t_end = esp_timer_get_time();
    const uint32_t latency_ms = (uint32_t)((t_end - t_start) / 1000);
    if (out_latency_ms) {
        *out_latency_ms = latency_ms;
    }
    ESP_LOGI(TAG, "HTTP %d in %u ms, body=%zu bytes", status, (unsigned)latency_ms, acc.len);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "Gemini request failed: err=%s status=%d", esp_err_to_name(err), status);
        if (acc.len > 0) {
            ESP_LOGE(TAG, "response: %.*s", (int)acc.len, acc.buf);
        }
        snprintf(out_json, out_json_size, "{\"error\":\"http %d\"}", status);
        err = ESP_FAIL;
        goto cleanup;
    }

    // ---- Parse response: candidates[0].content.parts[0].text is the JSON array ----
    cJSON *resp = cJSON_Parse(acc.buf);
    if (!resp) {
        ESP_LOGE(TAG, "response JSON parse failed");
        write_error(out_json, out_json_size, "response parse failed");
        err = ESP_FAIL;
        goto cleanup;
    }
    cJSON *candidates = cJSON_GetObjectItem(resp, "candidates");
    cJSON *cand0 = candidates ? cJSON_GetArrayItem(candidates, 0) : NULL;
    cJSON *cand_content = cand0 ? cJSON_GetObjectItem(cand0, "content") : NULL;
    cJSON *cand_parts = cand_content ? cJSON_GetObjectItem(cand_content, "parts") : NULL;
    cJSON *part0 = cand_parts ? cJSON_GetArrayItem(cand_parts, 0) : NULL;
    cJSON *text = part0 ? cJSON_GetObjectItem(part0, "text") : NULL;

    if (!cJSON_IsString(text) || !text->valuestring) {
        ESP_LOGE(TAG, "no candidates[0].content.parts[0].text");
        write_error(out_json, out_json_size, "missing text field");
        cJSON_Delete(resp);
        err = ESP_FAIL;
        goto cleanup;
    }

    size_t needed = strlen(text->valuestring);
    if (needed + 1 > out_json_size) {
        ESP_LOGW(TAG, "detections JSON (%zu) truncated to output buffer (%zu)", needed,
                 out_json_size);
        needed = out_json_size - 1;
    }
    memcpy(out_json, text->valuestring, needed);
    out_json[needed] = '\0';

    // Log Gemini usage metadata for cost observability
    cJSON *usage = cJSON_GetObjectItem(resp, "usageMetadata");
    if (usage) {
        cJSON *prompt_tokens = cJSON_GetObjectItem(usage, "promptTokenCount");
        cJSON *output_tokens = cJSON_GetObjectItem(usage, "candidatesTokenCount");
        cJSON *total_tokens = cJSON_GetObjectItem(usage, "totalTokenCount");
        ESP_LOGI(TAG, "tokens: prompt=%d output=%d total=%d",
                 cJSON_IsNumber(prompt_tokens) ? prompt_tokens->valueint : -1,
                 cJSON_IsNumber(output_tokens) ? output_tokens->valueint : -1,
                 cJSON_IsNumber(total_tokens) ? total_tokens->valueint : -1);
    }

    cJSON_Delete(resp);
    err = ESP_OK;

cleanup:
    esp_http_client_cleanup(client);
cleanup_noclient:
    free(request_body);
    free(acc.buf);
    return err;
}
