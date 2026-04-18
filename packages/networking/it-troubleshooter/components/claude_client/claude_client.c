/**
 * @file claude_client.c
 * @brief Claude API client for IT Troubleshooter (Phase 3).
 *
 * Sends operator-pasted command output to the Anthropic Claude API and
 * returns the suggested next diagnostic command.
 *
 * TLS certificate validation uses the bundled cert store
 * (CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y in sdkconfig.defaults).
 */
#include <string.h>

#include "cJSON.h"
#include "claude_client.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"

#define CLAUDE_API_URL "https://api.anthropic.com/v1/messages"
#define ANTHROPIC_VERSION "2023-06-01"
#define MAX_OUTPUT_LEN 1024
#define RESPONSE_BUF_LEN 4096
#define MAX_TOKENS 256
#define REQUEST_TIMEOUT_MS 30000

static const char *TAG = "claude_client";

static const char *s_api_key = NULL;
static const char *s_model = NULL;

static const char *SYSTEM_PROMPT =
    "You are a network troubleshooting assistant running on an embedded USB device. "
    "Analyze the command output from a target computer and suggest the single next "
    "diagnostic command to run. Reply with ONLY the command — no explanation, no "
    "markdown, no code block. If troubleshooting is complete or you need more context, "
    "reply with exactly: DONE";

typedef struct {
    char buf[RESPONSE_BUF_LEN];
    int offset;
} response_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    response_ctx_t *ctx = (response_ctx_t *)evt->user_data;
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (ctx->offset + evt->data_len < RESPONSE_BUF_LEN - 1) {
                memcpy(ctx->buf + ctx->offset, evt->data, evt->data_len);
                ctx->offset += evt->data_len;
            } else {
                ESP_LOGW(TAG, "Response buffer overflow — truncating at %d bytes", ctx->offset);
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

esp_err_t claude_client_init(const char *api_key, const char *model)
{
    if (!api_key || !model) {
        return ESP_ERR_INVALID_ARG;
    }
    s_api_key = api_key;
    s_model = model;
    ESP_LOGI(TAG, "Initialized (model=%s)", model);
    return ESP_OK;
}

void claude_client_deinit(void)
{
    s_api_key = NULL;
    s_model = NULL;
}

esp_err_t claude_client_analyze(const char *output, char *next_cmd, size_t next_cmd_len)
{
    if (!s_api_key || !s_model) {
        ESP_LOGE(TAG, "Client not initialized — call claude_client_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    if (!output || !next_cmd || next_cmd_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Truncate output to avoid oversized payloads */
    char truncated_output[MAX_OUTPUT_LEN + 1];
    size_t out_len = strnlen(output, MAX_OUTPUT_LEN);
    memcpy(truncated_output, output, out_len);
    truncated_output[out_len] = '\0';
    if (out_len == MAX_OUTPUT_LEN) {
        ESP_LOGD(TAG, "Output truncated to %d bytes", MAX_OUTPUT_LEN);
    }

    /* Build request JSON via cJSON — handles escaping of special chars */
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddStringToObject(root, "model", s_model);
    cJSON_AddNumberToObject(root, "max_tokens", MAX_TOKENS);
    cJSON_AddStringToObject(root, "system", SYSTEM_PROMPT);

    cJSON *messages = cJSON_CreateArray();
    cJSON *msg = cJSON_CreateObject();
    cJSON_AddStringToObject(msg, "role", "user");
    cJSON_AddStringToObject(msg, "content", truncated_output);
    cJSON_AddItemToArray(messages, msg);
    cJSON_AddItemToObject(root, "messages", messages);

    char *request_body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!request_body) {
        return ESP_ERR_NO_MEM;
    }

    /* HTTP client + TLS setup */
    response_ctx_t ctx = {.offset = 0};
    memset(ctx.buf, 0, sizeof(ctx.buf));

    esp_http_client_config_t config = {
        .url = CLAUDE_API_URL,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .timeout_ms = REQUEST_TIMEOUT_MS,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        cJSON_free(request_body);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "x-api-key", s_api_key);
    esp_http_client_set_header(client, "anthropic-version", ANTHROPIC_VERSION);
    esp_http_client_set_header(client, "content-type", "application/json");
    esp_http_client_set_post_field(client, request_body, (int)strlen(request_body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    cJSON_free(request_body);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        return err;
    }
    if (status != 200) {
        ESP_LOGE(TAG, "API returned HTTP %d: %.200s", status, ctx.buf);
        return ESP_FAIL;
    }

    /* Parse response: extract content[0].text */
    cJSON *response = cJSON_Parse(ctx.buf);
    if (!response) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    cJSON *content = cJSON_GetObjectItem(response, "content");
    if (cJSON_IsArray(content) && cJSON_GetArraySize(content) > 0) {
        cJSON *item = cJSON_GetArrayItem(content, 0);
        cJSON *text = cJSON_GetObjectItem(item, "text");
        if (cJSON_IsString(text) && text->valuestring) {
            strncpy(next_cmd, text->valuestring, next_cmd_len - 1);
            next_cmd[next_cmd_len - 1] = '\0';
            ESP_LOGI(TAG, "Claude suggests: %s", next_cmd);
            ret = ESP_OK;
        }
    }

    cJSON_Delete(response);
    return ret;
}
