#include "gemini_tts.h"

#include <stdlib.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "secrets.h"

static const char *TAG = "gemini_tts";

#define GEMINI_MODEL "gemini-2.5-flash-preview-tts"
#define GEMINI_URL                                                          \
    "https://generativelanguage.googleapis.com/v1beta/models/" GEMINI_MODEL \
    ":generateContent?key=" GEMINI_API_KEY
#define GEMINI_VOICE "Kore"

/* PSRAM-allocated response buffer */
#define MAX_RESPONSE_SIZE (1024 * 1024)  // 1MB max response

/*
 * JSON request template for Gemini TTS.
 * The %s placeholder is replaced with the prompt text.
 */
static const char *JSON_TEMPLATE =
    "{"
    "\"contents\":[{\"role\":\"user\",\"parts\":[{\"text\":\"%s\"}]}],"
    "\"generationConfig\":{"
    "\"responseModalities\":[\"AUDIO\"],"
    "\"speechConfig\":{\"voiceConfig\":{\"prebuiltVoiceConfig\":{\"voiceName\":\"" GEMINI_VOICE
    "\"}}}"
    "}"
    "}";

/* HTTP event handler — accumulates response into PSRAM buffer */
typedef struct {
    char *buffer;
    size_t len;
    size_t capacity;
} response_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    response_ctx_t *ctx = (response_ctx_t *)evt->user_data;

    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (ctx->len + evt->data_len > ctx->capacity) {
            ESP_LOGE(TAG, "Response too large (%u + %u > %u)", (unsigned)ctx->len,
                     (unsigned)evt->data_len, (unsigned)ctx->capacity);
            return ESP_FAIL;
        }
        memcpy(ctx->buffer + ctx->len, evt->data, evt->data_len);
        ctx->len += evt->data_len;
    }

    return ESP_OK;
}

/* Base64 decoding — standard alphabet, handles padding */
static const uint8_t B64_DECODE_TABLE[256] = {
    // clang-format off
    ['A'] = 0,  ['B'] = 1,  ['C'] = 2,  ['D'] = 3,  ['E'] = 4,  ['F'] = 5,
    ['G'] = 6,  ['H'] = 7,  ['I'] = 8,  ['J'] = 9,  ['K'] = 10, ['L'] = 11,
    ['M'] = 12, ['N'] = 13, ['O'] = 14, ['P'] = 15, ['Q'] = 16, ['R'] = 17,
    ['S'] = 18, ['T'] = 19, ['U'] = 20, ['V'] = 21, ['W'] = 22, ['X'] = 23,
    ['Y'] = 24, ['Z'] = 25, ['a'] = 26, ['b'] = 27, ['c'] = 28, ['d'] = 29,
    ['e'] = 30, ['f'] = 31, ['g'] = 32, ['h'] = 33, ['i'] = 34, ['j'] = 35,
    ['k'] = 36, ['l'] = 37, ['m'] = 38, ['n'] = 39, ['o'] = 40, ['p'] = 41,
    ['q'] = 42, ['r'] = 43, ['s'] = 44, ['t'] = 45, ['u'] = 46, ['v'] = 47,
    ['w'] = 48, ['x'] = 49, ['y'] = 50, ['z'] = 51, ['0'] = 52, ['1'] = 53,
    ['2'] = 54, ['3'] = 55, ['4'] = 56, ['5'] = 57, ['6'] = 58, ['7'] = 59,
    ['8'] = 60, ['9'] = 61, ['+'] = 62, ['/'] = 63,
    // clang-format on
};

static size_t base64_decode(const char *src, size_t src_len, uint8_t *dst)
{
    size_t out = 0;
    uint32_t accum = 0;
    int bits = 0;

    for (size_t i = 0; i < src_len; i++) {
        char c = src[i];
        if (c == '=' || c == '\n' || c == '\r' || c == ' ') {
            continue;
        }
        accum = (accum << 6) | B64_DECODE_TABLE[(uint8_t)c];
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            dst[out++] = (uint8_t)(accum >> bits);
        }
    }

    return out;
}

/*
 * Find the base64 audio data in the JSON response.
 * Looks for "data":" and extracts until the closing ".
 * Returns pointer into the response buffer (not a copy).
 */
static esp_err_t find_audio_data(const char *json, size_t json_len, const char **data_start,
                                 size_t *data_len)
{
    const char *needle = "\"data\":\"";
    size_t needle_len = strlen(needle);

    const char *pos = NULL;
    for (size_t i = 0; i + needle_len < json_len; i++) {
        if (memcmp(&json[i], needle, needle_len) == 0) {
            pos = &json[i + needle_len];
            break;
        }
    }

    if (!pos) {
        /* Try alternate spacing: "data" : " */
        needle = "\"data\": \"";
        needle_len = strlen(needle);
        for (size_t i = 0; i + needle_len < json_len; i++) {
            if (memcmp(&json[i], needle, needle_len) == 0) {
                pos = &json[i + needle_len];
                break;
            }
        }
    }

    if (!pos) {
        ESP_LOGE(TAG, "Could not find audio data in response");
        return ESP_ERR_NOT_FOUND;
    }

    /* Find closing quote */
    const char *end = pos;
    const char *json_end = json + json_len;
    while (end < json_end && *end != '"') {
        end++;
    }

    if (end >= json_end) {
        ESP_LOGE(TAG, "Unterminated base64 data in response");
        return ESP_ERR_INVALID_RESPONSE;
    }

    *data_start = pos;
    *data_len = end - pos;

    ESP_LOGI(TAG, "Found base64 audio data: %u bytes", (unsigned)*data_len);
    return ESP_OK;
}

static char *build_request_json(const char *prompt)
{
    /* Escape special chars in prompt for JSON embedding */
    size_t prompt_len = strlen(prompt);
    size_t escaped_capacity = prompt_len * 2 + 1;
    char *escaped = malloc(escaped_capacity);
    if (!escaped) {
        return NULL;
    }

    size_t j = 0;
    for (size_t i = 0; i < prompt_len && j < escaped_capacity - 2; i++) {
        char c = prompt[i];
        if (c == '"' || c == '\\') {
            escaped[j++] = '\\';
        } else if (c == '\n') {
            escaped[j++] = '\\';
            c = 'n';
        }
        escaped[j++] = c;
    }
    escaped[j] = '\0';

    size_t template_len = strlen(JSON_TEMPLATE);
    size_t json_len = template_len + j + 64;
    char *json = malloc(json_len);
    if (!json) {
        free(escaped);
        return NULL;
    }

    snprintf(json, json_len, JSON_TEMPLATE, escaped);
    free(escaped);
    return json;
}

esp_err_t gemini_tts_request(const char *prompt, tts_result_t *result)
{
    memset(result, 0, sizeof(*result));

    char *request_json = build_request_json(prompt);
    if (!request_json) {
        return ESP_ERR_NO_MEM;
    }

    /* Allocate response buffer in PSRAM */
    response_ctx_t ctx = {
        .buffer = heap_caps_malloc(MAX_RESPONSE_SIZE, MALLOC_CAP_SPIRAM),
        .len = 0,
        .capacity = MAX_RESPONSE_SIZE,
    };

    if (!ctx.buffer) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM response buffer");
        free(request_json);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_config_t config = {
        .url = GEMINI_URL,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        free(request_json);
        heap_caps_free(ctx.buffer);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, request_json, strlen(request_json));

    ESP_LOGI(TAG, "Sending TTS request (%u bytes)...", (unsigned)strlen(request_json));

    esp_err_t ret = esp_http_client_perform(client);
    free(request_json);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        heap_caps_free(ctx.buffer);
        return ret;
    }

    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGE(TAG, "API returned HTTP %d", status);
        /* Log first 500 chars of response for debugging */
        size_t log_len = ctx.len < 500 ? ctx.len : 500;
        ctx.buffer[log_len] = '\0';
        ESP_LOGE(TAG, "Response: %s", ctx.buffer);
        heap_caps_free(ctx.buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Response received: %u bytes", (unsigned)ctx.len);

    /* Find and decode base64 audio data */
    const char *b64_start;
    size_t b64_len;
    ret = find_audio_data(ctx.buffer, ctx.len, &b64_start, &b64_len);
    if (ret != ESP_OK) {
        heap_caps_free(ctx.buffer);
        return ret;
    }

    /* Allocate PCM buffer in PSRAM: base64 decodes to ~75% of encoded size */
    size_t max_pcm = (b64_len * 3) / 4 + 4;
    int16_t *pcm = heap_caps_malloc(max_pcm, MALLOC_CAP_SPIRAM);
    if (!pcm) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM PCM buffer (%u bytes)", (unsigned)max_pcm);
        heap_caps_free(ctx.buffer);
        return ESP_ERR_NO_MEM;
    }

    size_t pcm_bytes = base64_decode(b64_start, b64_len, (uint8_t *)pcm);
    heap_caps_free(ctx.buffer);  // Done with the JSON response

    ESP_LOGI(TAG, "Decoded %u bytes of PCM audio (%.1f seconds @ 24kHz)", (unsigned)pcm_bytes,
             (float)pcm_bytes / (24000.0f * 2));

    result->pcm_data = pcm;
    result->pcm_len = pcm_bytes;
    result->sample_rate = 24000;

    return ESP_OK;
}
