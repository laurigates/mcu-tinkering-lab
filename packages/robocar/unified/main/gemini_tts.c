/**
 * @file gemini_tts.c
 * @brief Gemini TTS fetch task. See gemini_tts.h for the design.
 */

#include "gemini_tts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_player.h"
#include "base64.h"
#include "cJSON.h"
#include "credentials_loader.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"
#include "speech_queue.h"

static const char *TAG = "gemini_tts";

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

/** Speech-generation model. Distinct from the Robotics-ER planner model.
 *  Returns raw 24 kHz 16-bit mono PCM with NO WAV header, base64-encoded in
 *  candidates[0].content.parts[N].inlineData.data. */
#define TTS_MODEL "gemini-3.1-flash-tts-preview"
#define TTS_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" TTS_MODEL ":generateContent"

/** Prebuilt voice name. */
#define TTS_VOICE "Kore"

/** Longer than the planner's budget: synthesis plus transferring a few
 *  hundred kB of base64 is slower than a function-call response. */
#define TTS_TIMEOUT_MS 20000

/** Max block when the playback ring is full. Exceeding this means playback
 *  has stalled (not merely lagged), so the utterance is abandoned. */
#define TTS_RING_TIMEOUT_MS 4000

/** Cap on captured error-response text. */
#define TTS_ERR_BUF_SIZE 256

/* -------------------------------------------------------------------------- */
/* Streaming context                                                           */
/* -------------------------------------------------------------------------- */

typedef struct {
    base64_stream_t b64;
    size_t pcm_bytes;
    bool ring_stalled;
    bool http_error; /**< non-200: capture body, decode nothing */
    char err[TTS_ERR_BUF_SIZE];
    size_t err_len;
} tts_ctx_t;

/**
 * Sink for decoded PCM.
 *
 * Blocking here is intentional flow control: when the ring is full this stalls
 * the HTTP event handler, which stops draining the socket and lets TCP
 * backpressure throttle the download to real-time playback speed. Without it a
 * long utterance would need unbounded PSRAM.
 */
static bool pcm_sink(const uint8_t *data, size_t len, void *ctx)
{
    tts_ctx_t *tc = (tts_ctx_t *)ctx;

    if (audio_player_write(data, len, TTS_RING_TIMEOUT_MS) != ESP_OK) {
        tc->ring_stalled = true;
        return false;  // abort the decode
    }

    tc->pcm_bytes += len;
    return true;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    tts_ctx_t *tc = (tts_ctx_t *)evt->user_data;

    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA: {
            if (!tc) {
                break;
            }

            // Status is known by the time body data arrives. On an error the
            // body is a JSON error object with no "data" key, so the decoder
            // would harmlessly find nothing — but capturing it gives a usable
            // log line instead of a silent failure.
            if (esp_http_client_get_status_code(evt->client) != 200) {
                tc->http_error = true;
                const size_t room = TTS_ERR_BUF_SIZE - 1 - tc->err_len;
                const size_t n = (evt->data_len < (int)room) ? (size_t)evt->data_len : room;
                if (n > 0) {
                    memcpy(tc->err + tc->err_len, evt->data, n);
                    tc->err_len += n;
                    tc->err[tc->err_len] = '\0';
                }
                break;
            }

            base64_stream_feed(&tc->b64, (const char *)evt->data, (size_t)evt->data_len, pcm_sink,
                               tc);
            break;
        }

        case HTTP_EVENT_ERROR:
            ESP_LOGW(TAG, "HTTP transport error");
            break;

        default:
            break;
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Request                                                                     */
/* -------------------------------------------------------------------------- */

static char *build_request_json(const char *text)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");
    cJSON *part = cJSON_CreateObject();
    cJSON_AddStringToObject(part, "text", text);
    cJSON_AddItemToArray(parts, part);
    cJSON_AddItemToArray(contents, content);

    cJSON *gen_cfg = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON *modalities = cJSON_AddArrayToObject(gen_cfg, "responseModalities");
    cJSON_AddItemToArray(modalities, cJSON_CreateString("AUDIO"));

    cJSON *speech_cfg = cJSON_AddObjectToObject(gen_cfg, "speechConfig");
    cJSON *voice_cfg = cJSON_AddObjectToObject(speech_cfg, "voiceConfig");
    cJSON *prebuilt = cJSON_AddObjectToObject(voice_cfg, "prebuiltVoiceConfig");
    cJSON_AddStringToObject(prebuilt, "voiceName", TTS_VOICE);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static void speak(const char *text)
{
    const char *api_key = get_gemini_api_key();
    if (!api_key || api_key[0] == '\0') {
        ESP_LOGE(TAG, "API key unavailable — cannot speak");
        return;
    }

    char *body = build_request_json(text);
    if (!body) {
        ESP_LOGE(TAG, "request JSON build failed");
        return;
    }

    tts_ctx_t ctx = {0};
    base64_stream_init(&ctx.b64);

    esp_http_client_config_t cfg = {
        .url = TTS_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = TTS_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "esp_http_client_init() failed");
        free(body);
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-goog-api-key", api_key);
    esp_http_client_set_post_field(client, body, strlen(body));

    const int64_t t_start = esp_timer_get_time();
    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    const uint32_t latency_ms = (uint32_t)((esp_timer_get_time() - t_start) / 1000);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "TTS request failed: %s status=%d%s%s", esp_err_to_name(err), status,
                 ctx.err_len ? " body=" : "", ctx.err_len ? ctx.err : "");
    } else if (ctx.ring_stalled) {
        ESP_LOGW(TAG, "playback stalled — utterance truncated at %u bytes",
                 (unsigned)ctx.pcm_bytes);
    } else if (ctx.pcm_bytes == 0) {
        ESP_LOGW(TAG, "HTTP 200 but no audio payload found in response");
    } else {
        ESP_LOGI(TAG, "spoke %u bytes (%u ms audio) in %u ms: \"%s\"", (unsigned)ctx.pcm_bytes,
                 (unsigned)(ctx.pcm_bytes * 1000 / (AUDIO_SAMPLE_RATE_HZ * sizeof(int16_t))),
                 (unsigned)latency_ms, text);
    }

    audio_player_end_utterance();

    esp_http_client_cleanup(client);
    free(body);
}

/* -------------------------------------------------------------------------- */
/* Task                                                                        */
/* -------------------------------------------------------------------------- */

static void tts_task(void *arg)
{
    (void)arg;
    speech_request_t req;

    for (;;) {
        if (speech_queue_receive(&req, UINT32_MAX) == ESP_OK) {
            speak(req.text);
        }
    }
}

esp_err_t gemini_tts_start(void)
{
    const BaseType_t ok =
        xTaskCreatePinnedToCore(tts_task, "gemini_tts", TTS_FETCH_TASK_STACK_SIZE, NULL,
                                TTS_FETCH_TASK_PRIORITY, NULL, TTS_FETCH_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create TTS task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TTS task started (model=%s voice=%s)", TTS_MODEL, TTS_VOICE);
    return ESP_OK;
}
