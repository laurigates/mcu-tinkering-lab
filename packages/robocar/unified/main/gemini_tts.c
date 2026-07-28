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
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_http.h"
#include "pin_config.h"
#include "speech_queue.h"
#include "voice_persona.h"

static const char *TAG = "gemini_tts";

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

/** Speech-generation model. Distinct from the Robotics-ER planner model.
 *  Returns raw 24 kHz 16-bit mono PCM with NO WAV header, base64-encoded in
 *  candidates[0].content.parts[N].inlineData.data. */
#define TTS_MODEL "gemini-3.1-flash-tts-preview"

/** Streaming endpoint, not `:generateContent`.
 *
 *  `:generateContent` synthesises the *whole* utterance before sending a byte,
 *  so the ring-buffer overlap this module is built around had almost nothing to
 *  overlap with. Measured on one Finnish line (2026-07):
 *
 *      :generateContent               TTFB 6.42 s   total 7.42 s   1 response
 *      :streamGenerateContent?alt=sse TTFB 1.16 s   total 3.78 s   233 events
 *
 *  Same request body, same model — only the endpoint differs. `alt=sse` frames
 *  each event as `data: <json>\n\n`, and every event carries its own
 *  `inlineData.data` chunk of the same 24 kHz PCM, so the streaming decoder
 *  consumes it unchanged (see base64.h on multi-payload bodies). */
#define TTS_URL                                                          \
    "https://generativelanguage.googleapis.com/v1beta/models/" TTS_MODEL \
    ":streamGenerateContent?alt=sse"

/* Voice name and language now come from the active persona (voice_persona.h),
 * so they can be switched at runtime rather than pinned here. */

/** Longer than the planner's budget: synthesis plus transferring a few
 *  hundred kB of base64 is slower than a function-call response.
 *
 *  It must also clear the *playback* time, not just the synthesis time.
 *  pcm_sink blocks when the ring is full, so the transfer cannot finish faster
 *  than the utterance drains once the ring saturates; at SPEECH_TEXT_MAX (320
 *  chars, ~20 s of speech) a 20 s budget would tear the socket down
 *  mid-sentence. The larger ring loosens the coupling, and this covers the
 *  rest. */
#define TTS_TIMEOUT_MS 40000

/** Bytes of decoded PCM accumulated before handing them to the player.
 *
 *  The decoder emits exactly 3 bytes per base64 quartet — measured over the
 *  live stream, 66560/66560 sink calls for a 4 s utterance were 3 bytes, which
 *  is structural (every payload is a whole number of quartets). Passing each
 *  one straight to audio_player_write() meant ~100 000 xRingbufferSend calls
 *  per utterance, and ESP-IDF's prvSendGeneric ends each with
 *  portYIELD_WITHIN_API() when a reader is waiting — so the fetch task was
 *  preempted by the higher-priority player roughly 25 000 times a second, each
 *  time for a 4-byte I2S write, precisely while it was trying to get ahead of
 *  real time. Batching cuts that by ~700x. */
#define TTS_PCM_BATCH_BYTES 2048

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
    /** Accumulator so the player is fed in TTS_PCM_BATCH_BYTES runs rather than
     *  in the decoder's 3-byte quartets. */
    uint8_t batch[TTS_PCM_BATCH_BYTES];
    size_t batch_len;
    size_t pcm_bytes;
    /** When the first decoded sample reached the ring. This — not the total
     *  request time — is what the listener perceives as the robot's delay, and
     *  it is the number the streaming endpoint exists to shrink. Logged so a
     *  regression back to whole-response synthesis is visible in the monitor. */
    int64_t first_pcm_us;
    bool ring_stalled;
    bool http_error; /**< non-200: capture body, decode nothing */
    char err[TTS_ERR_BUF_SIZE];
    size_t err_len;
} tts_ctx_t;

/**
 * Hand the accumulated batch to the player.
 *
 * Blocking here is intentional flow control: when the ring is full this stalls
 * the HTTP event handler, which stops draining the socket and lets TCP
 * backpressure throttle the download to real-time playback speed. Without it a
 * long utterance would need unbounded PSRAM.
 */
static bool batch_flush(tts_ctx_t *tc)
{
    if (tc->batch_len == 0) {
        return true;
    }

    if (tc->first_pcm_us == 0) {
        tc->first_pcm_us = esp_timer_get_time();
    }

    if (audio_player_write(tc->batch, tc->batch_len, TTS_RING_TIMEOUT_MS) != ESP_OK) {
        tc->ring_stalled = true;
        return false;  // abort the decode
    }

    tc->pcm_bytes += tc->batch_len;
    tc->batch_len = 0;
    return true;
}

/**
 * Sink for decoded PCM. Called once per base64 quartet, i.e. with 3 bytes;
 * accumulates into tc->batch and flushes a batch at a time — see
 * TTS_PCM_BATCH_BYTES for why the granularity matters.
 */
static bool pcm_sink(const uint8_t *data, size_t len, void *ctx)
{
    tts_ctx_t *tc = (tts_ctx_t *)ctx;

    while (len > 0) {
        const size_t room = sizeof(tc->batch) - tc->batch_len;
        const size_t n = (len < room) ? len : room;
        memcpy(tc->batch + tc->batch_len, data, n);
        tc->batch_len += n;
        data += n;
        len -= n;

        if (tc->batch_len == sizeof(tc->batch) && !batch_flush(tc)) {
            return false;
        }
    }
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

            /* Once the playback ring has stalled, stop feeding the decoder.
             * pcm_sink()'s abort return only ends the *current* feed call — it
             * does not advance the decoder state — so without this guard the
             * next chunk resumes decoding, hits the same full ring, and blocks
             * another full TTS_RING_TIMEOUT_MS. Repeated for every remaining
             * chunk of a response that is hundreds of kB. */
            if (tc->ring_stalled) {
                return ESP_FAIL;  // tear the transfer down
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

            if (base64_stream_feed(&tc->b64, (const char *)evt->data, (size_t)evt->data_len,
                                   pcm_sink, tc) != 0) {
                ESP_LOGW(TAG, "PCM decode aborted (ring stalled) — ending transfer");
                return ESP_FAIL;
            }
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

    const voice_persona_t *persona = voice_persona_get();

    /* Delivery style is prompted, not parameterised — Gemini has no style/accent
     * field, so the documented form is "<style directive>: <text to speak>".
     * The directive is interpreted rather than read aloud (measured: a ~40-word
     * directive, 12+ s if spoken, added 0.36 s of audio). */
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");
    cJSON *part = cJSON_CreateObject();
    char styled[SPEECH_TEXT_MAX + 256];
    snprintf(styled, sizeof(styled), "%s: %s", persona->tts_style, text);
    cJSON_AddStringToObject(part, "text", styled);
    cJSON_AddItemToArray(parts, part);
    cJSON_AddItemToArray(contents, content);

    cJSON *gen_cfg = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON *modalities = cJSON_AddArrayToObject(gen_cfg, "responseModalities");
    cJSON_AddItemToArray(modalities, cJSON_CreateString("AUDIO"));

    cJSON *speech_cfg = cJSON_AddObjectToObject(gen_cfg, "speechConfig");
    cJSON_AddStringToObject(speech_cfg, "languageCode", persona->language_code);
    cJSON *voice_cfg = cJSON_AddObjectToObject(speech_cfg, "voiceConfig");
    cJSON *prebuilt = cJSON_AddObjectToObject(voice_cfg, "prebuiltVoiceConfig");
    char voice[VOICE_PERSONA_VOICE_MAX];
    voice_persona_effective_voice(voice, sizeof(voice));
    cJSON_AddStringToObject(prebuilt, "voiceName", voice);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

/* File scope rather than a speak() local: the batch accumulator makes
 * tts_ctx_t ~2 kB, which does not belong on the 8 kB stack of a task that also
 * runs a TLS handshake. Only the single TTS task ever touches it. */
static tts_ctx_t s_tts_ctx;

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

    tts_ctx_t *ctx = &s_tts_ctx;
    memset(ctx, 0, sizeof(*ctx));
    base64_stream_init(&ctx->b64);

    /* Opens the preroll window: nothing plays until AUDIO_PREROLL_BYTES have
     * banked or this request completes. Paired with the unconditional
     * audio_player_end_utterance() below, which closes it on every exit path. */
    audio_player_begin_utterance();

    const int64_t t_start = esp_timer_get_time();
    int status = 0;
    const esp_err_t err =
        gemini_http_post(TTS_URL, api_key, body, TTS_TIMEOUT_MS, http_event_handler, ctx, &status);

    /* Whatever the decoder left in the accumulator is still real audio — the
     * tail of the utterance. Flush before judging the result, so pcm_bytes and
     * the "no audio payload" branch below see the true total. */
    if (!ctx->ring_stalled) {
        batch_flush(ctx);
    }

    const uint32_t latency_ms = (uint32_t)((esp_timer_get_time() - t_start) / 1000);

    /* Checked before the transport error: a stall now deliberately fails the
     * perform() (the handler returns ESP_FAIL to tear the socket down), so the
     * generic error branch would otherwise mask the real cause. */
    if (ctx->ring_stalled) {
        ESP_LOGW(TAG, "playback stalled — utterance truncated at %u bytes",
                 (unsigned)ctx->pcm_bytes);
        /* Drop the partial utterance rather than leaving a fragment to drain.
         * abort() already closes the preroll gate and clears the in-flight
         * flag, so end_utterance() is skipped below — calling it here would
         * re-open the gate on an empty ring and let the NEXT utterance start
         * playing with nothing banked. */
        audio_player_abort();
    } else if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "TTS request failed: %s status=%d%s%s", esp_err_to_name(err), status,
                 ctx->err_len ? " body=" : "", ctx->err_len ? ctx->err : "");
    } else if (ctx->pcm_bytes == 0) {
        ESP_LOGW(TAG, "HTTP 200 but no audio payload found in response");
    } else {
        const uint32_t audio_ms =
            (uint32_t)(ctx->pcm_bytes * 1000 / (AUDIO_SAMPLE_RATE_HZ * sizeof(int16_t)));
        const uint32_t first_ms =
            ctx->first_pcm_us ? (uint32_t)((ctx->first_pcm_us - t_start) / 1000) : 0;
        /* rtf = audio produced per unit wallclock after first byte. Below 1.00
         * means Gemini generated slower than the 48 kB/s playback rate, i.e.
         * this utterance would have underrun without the preroll gate — the
         * single number that says whether the voice path is healthy. */
        const uint32_t stream_ms = (latency_ms > first_ms) ? (latency_ms - first_ms) : 1;
        ESP_LOGI(TAG, "spoke %u bytes (%u ms audio) first=%u ms total=%u ms rtf=%u.%02u: \"%s\"",
                 (unsigned)ctx->pcm_bytes, (unsigned)audio_ms, (unsigned)first_ms,
                 (unsigned)latency_ms, (unsigned)(audio_ms / stream_ms),
                 (unsigned)((audio_ms * 100 / stream_ms) % 100), text);
    }

    if (!ctx->ring_stalled) {
        audio_player_end_utterance();
    }

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

    char voice[VOICE_PERSONA_VOICE_MAX];
    voice_persona_effective_voice(voice, sizeof(voice));
    ESP_LOGI(TAG, "TTS task started (model=%s persona=%s lang=%s voice=%s)", TTS_MODEL,
             voice_persona_get()->slug, voice_persona_get()->language_code, voice);
    return ESP_OK;
}
