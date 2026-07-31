/**
 * @file voice_turn.c
 * @brief Push-to-talk: record a clip, ask Gemini, speak the reply.
 *
 * See voice_turn.h for the design. The ordering inside run_turn() is the part
 * worth reading — it is what keeps the transient allocation at roughly 430 kB
 * rather than 800 kB, on a device where the camera framebuffers and the 512 kB
 * TTS ring are already spoken for.
 */

#include "voice_turn.h"

#include <stdlib.h>
#include <string.h>

#include "ambient_audio.h"
#include "audio_clip.h"
#include "audio_player.h"
#include "base64.h"
#include "buzzer.h"
#include "cJSON.h"
#include "credentials_loader.h"
#include "dialogue_style.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gemini_http.h"
#include "gemini_parse.h"
#include "mic_pdm.h"
#include "pin_config.h"
#include "speech_budget.h"
#include "speech_queue.h"
#include "voice_persona.h"

static const char *TAG = "voice_turn";

/** A flash model, NOT the planner's Robotics-ER: quota is per model, and the
 *  planner already saturates ER's 5 req/min. Verified reachable and verified to
 *  accept an inline audio part (2026-07). */
#define VOICE_TURN_MODEL "gemini-flash-latest"
#define VOICE_TURN_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" VOICE_TURN_MODEL ":generateContent"

/** 30 s, not the planner's 15 s: a ~170 kB upload over a domestic uplink can
 *  spend most of a planner timeout still sending. */
#define VOICE_TURN_TIMEOUT_MS 30000

/** Own response buffer, never gemini_backend.c's shared s_response_buf —
 *  gemini_backend_plan() is documented single-task and this runs on a different
 *  one. A one-sentence reply is tiny; 4 kB also absorbs an error body. */
#define VOICE_TURN_RESPONSE_BUF_SIZE (4 * 1024)

/** Combined thinking + reply budget (thinking is spent first, and this model
 *  always thinks). Measured elsewhere in this firmware at 487–745 thought tokens
 *  for a one-sentence answer, so 512 truncates every time and 1024 has no
 *  margin. Raising it is nearly free: thinking tokens are billed either way, and
 *  the cap only decides whether the sentence survives. */
#define VOICE_TURN_MAX_OUTPUT_TOKENS 2048

/** Settle time after the start beep. The buzzer is on GPIO2, a different
 *  peripheral from the I2S amplifier, so audio_player_is_active() does NOT cover
 *  it — without this the first fraction of every clip is the robot's own beep,
 *  and the model politely transcribes it. */
#define VOICE_TURN_BEEP_SETTLE_MS 120

typedef struct {
    uint32_t window_ms;
} voice_turn_req_t;

static QueueHandle_t s_queue;
static volatile bool s_busy;

/* Per-turn state at FILE scope, not on the 8 kB stack — the same reason
 * gemini_tts.c keeps its context static. The response buffer alone would be
 * half the stack. */
static char s_response[VOICE_TURN_RESPONSE_BUF_SIZE];
static char s_reply[SPEECH_TEXT_MAX];

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} response_acc_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id != HTTP_EVENT_ON_DATA) {
        return ESP_OK;
    }
    response_acc_t *acc = (response_acc_t *)evt->user_data;
    if (!acc || !acc->buf || !evt->data || evt->data_len <= 0) {
        return ESP_OK;
    }
    const size_t avail = acc->cap - 1 - acc->len;
    const size_t to_copy = ((size_t)evt->data_len < avail) ? (size_t)evt->data_len : avail;
    if (to_copy > 0) {
        memcpy(acc->buf + acc->len, evt->data, to_copy);
        acc->len += to_copy;
        acc->buf[acc->len] = '\0';
    }
    return ESP_OK;
}

/**
 * @brief Record @p pcm_bytes of audio, holding the microphone for the window.
 *
 * The ambient listener simply misses these frames, and that is correct rather
 * than merely tolerable: the noise floor must not learn from a conversation, or
 * a chat with the robot would raise the floor enough to deafen the gate
 * afterwards.
 */
static esp_err_t record_clip(int16_t *pcm, size_t samples, size_t *out_samples)
{
    if (mic_pdm_lock(1000) != ESP_OK) {
        ESP_LOGW(TAG, "microphone busy");
        return ESP_ERR_INVALID_STATE;
    }

    /* Drop whatever the DMA accumulated while we were beeping and settling. */
    mic_pdm_flush();

    size_t filled = 0;
    esp_err_t err = ESP_OK;
    while (filled < samples) {
        size_t got = 0;
        err = mic_pdm_read(pcm + filled, samples - filled, &got, 1000);
        if (err != ESP_OK || got == 0) {
            break;
        }
        filled += got;
    }
    mic_pdm_unlock();

    *out_samples = filled;
    if (filled == 0) {
        return (err == ESP_OK) ? ESP_FAIL : err;
    }
    return ESP_OK;
}

/** Build the request body, taking ownership of nothing and freeing nothing. */
static char *build_body(const char *b64_wav)
{
    const voice_persona_t *persona = voice_persona_get();

    cJSON *root = cJSON_CreateObject();
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *turn = cJSON_CreateObject();
    cJSON_AddItemToArray(contents, turn);
    cJSON *parts = cJSON_AddArrayToObject(turn, "parts");

    cJSON *text_part = cJSON_CreateObject();
    cJSON_AddItemToArray(parts, text_part);
    cJSON_AddStringToObject(
        text_part, "text",
        "You are a small wheeled robot. Someone has just spoken to you; the audio follows. "
        "Answer them in ONE short spoken sentence. Reply with the sentence itself and nothing "
        "else — no preamble, no quotation marks, no stage directions, no markdown. If the audio "
        "contains no intelligible speech, say so briefly in the same voice.");

    cJSON *audio_part = cJSON_CreateObject();
    cJSON_AddItemToArray(parts, audio_part);
    cJSON *inline_data = cJSON_AddObjectToObject(audio_part, "inlineData");
    /* audio/wav, never audio/pcm: the latter is rejected with a 400 that names
     * no field. See audio_clip.h for the probe that established this. */
    cJSON_AddStringToObject(inline_data, "mimeType", "audio/wav");
    cJSON_AddStringToObject(inline_data, "data", b64_wav);

    cJSON *sys = cJSON_AddObjectToObject(root, "systemInstruction");
    cJSON *sys_parts = cJSON_AddArrayToObject(sys, "parts");
    cJSON *sys_text = cJSON_CreateObject();
    cJSON_AddItemToArray(sys_parts, sys_text);
    cJSON_AddStringToObject(sys_text, "text",
                            persona && persona->text_brief ? persona->text_brief : "Be brief.");

    cJSON *gen = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON_AddNumberToObject(gen, "maxOutputTokens", VOICE_TURN_MAX_OUTPUT_TOKENS);
    cJSON *thinking = cJSON_AddObjectToObject(gen, "thinkingConfig");
    /* thinkingLevel, NOT thinkingBudget: the numeric form is rejected by
     * Gemini 3-era models with a bare 400 that names no field. */
    cJSON_AddStringToObject(thinking, "thinkingLevel", "low");

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body; /* caller frees */
}

static void run_turn(uint32_t window_ms)
{
    const uint32_t t_start = (uint32_t)(esp_timer_get_time() / 1000);

    const size_t pcm_bytes = audio_clip_pcm_bytes(window_ms, MIC_SAMPLE_RATE_HZ);
    if (pcm_bytes == 0) {
        ESP_LOGE(TAG, "window %u ms rejected by clip sizing", (unsigned)window_ms);
        return;
    }
    const size_t samples = pcm_bytes / sizeof(int16_t);

    const char *api_key = get_gemini_api_key();
    if (!api_key || api_key[0] == '\0') {
        ESP_LOGE(TAG, "no Gemini API key — cannot run a voice turn");
        return;
    }

    /* PSRAM: this is hundreds of kB and internal RAM is the scarce pool the
     * camera and the TLS handshake compete for. */
    int16_t *pcm = heap_caps_malloc(pcm_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) {
        ESP_LOGE(TAG, "no PSRAM for a %u-byte clip", (unsigned)pcm_bytes);
        return;
    }

    /* Record-start feedback. buzzer_beep() blocks for its full duration, which
     * is exactly the "wait for it to finish" this needs; the settle delay covers
     * the piezo ringing down afterwards. */
    buzzer_beep();
    vTaskDelay(pdMS_TO_TICKS(VOICE_TURN_BEEP_SETTLE_MS));

    size_t got = 0;
    if (record_clip(pcm, samples, &got) != ESP_OK) {
        ESP_LOGE(TAG, "recording failed");
        heap_caps_free(pcm);
        return;
    }

    audio_clip_stats_t st;
    audio_clip_normalise(pcm, got, &st);

    const size_t got_bytes = got * sizeof(int16_t);
    const size_t wav_bytes = got_bytes + AUDIO_CLIP_WAV_HEADER_BYTES;

    /* Build the WAV in its own buffer so the header and payload are contiguous
     * for a single base64 pass. */
    uint8_t *wav = heap_caps_malloc(wav_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav || !audio_clip_wav_header(wav, got_bytes, MIC_SAMPLE_RATE_HZ, 1, 16)) {
        ESP_LOGE(TAG, "WAV framing failed");
        heap_caps_free(wav);
        heap_caps_free(pcm);
        return;
    }
    memcpy(wav + AUDIO_CLIP_WAV_HEADER_BYTES, pcm, got_bytes);
    heap_caps_free(pcm); /* the raw PCM is now redundant — release it before the
                          * base64 copy exists, not after */
    pcm = NULL;

    char *b64 = base64_encode_alloc(wav, wav_bytes);
    heap_caps_free(wav); /* likewise: the encoder has its own copy now */
    wav = NULL;
    if (!b64) {
        ESP_LOGE(TAG, "base64 encode failed");
        return;
    }

    char *body = build_body(b64);
    /* cJSON duplicated the string when it was added; holding both is a pure
     * waste of ~170 kB at the exact moment the body is also live. */
    free(b64);
    b64 = NULL;
    if (!body) {
        ESP_LOGE(TAG, "request build failed");
        return;
    }

    const size_t body_len = strlen(body);
    ESP_LOGI(TAG,
             "listen: window=%u ms samples=%u peak=%d clipped=%u dc=%d | upload=%u B | free "
             "PSRAM=%u B",
             (unsigned)window_ms, (unsigned)got, (int)st.peak, (unsigned)st.clipped, (int)st.dc,
             (unsigned)body_len, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    response_acc_t acc = {.buf = s_response, .len = 0, .cap = sizeof(s_response)};
    s_response[0] = '\0';
    int status = 0;
    const esp_err_t err = gemini_http_post(VOICE_TURN_URL, api_key, body, VOICE_TURN_TIMEOUT_MS,
                                           http_event_handler, &acc, &status);
    free(body);

    if (err != ESP_OK) {
        /* ERROR, never DEBUG. A Gemini 400 names no field, so the body IS the
         * diagnosis — hiding it at debug level is what made an earlier call site
         * fail opaquely for an entire debugging round. */
        ESP_LOGE(TAG, "voice turn HTTP failed (status %d): %s", status,
                 acc.len ? s_response : "(empty body)");
        return;
    }

    if (gemini_parse_text(s_response, s_reply, sizeof(s_reply)) != ESP_OK) {
        ESP_LOGE(TAG, "no usable text in reply: %s", acc.len ? s_response : "(empty body)");
        return;
    }

    const uint32_t latency_ms = (uint32_t)(esp_timer_get_time() / 1000) - t_start;
    ESP_LOGI(TAG, "listen: latency=%u ms reply=\"%s\"", (unsigned)latency_ms, s_reply);

    if (speech_queue_post(s_reply) != ESP_OK) {
        ESP_LOGW(TAG, "speech queue full — reply dropped");
        return;
    }

    /* Post-speech bookkeeping. Each of these is a deliberate ruling:
     *
     *  - note_spoken: so the planner does not parrot the answer back at its next
     *    cycle, having no idea the robot just said it.
     *  - budget_note: the robot just talked. A spontaneous remark three seconds
     *    later is exactly the chattering the minimum gap exists to prevent.
     *  - ambient mark_spoken: the human's voice is what armed the audio latch.
     *    Having answered it, the robot must not then volunteer "I heard
     *    something" on the next planner cycle.
     *
     * Deliberately NOT done:
     *  - scene_change_mark_spoken(): answering a question is not remarking on
     *    the view, and consuming the visual evidence would silence a genuine
     *    observation the robot had not yet made.
     *  - dialogue_style_is_repetitive(): a direct answer is allowed to repeat.
     *    Same reasoning that exempts `voice say` and the self-report — screening
     *    it would turn a working command into one that silently does nothing
     *    whenever someone asks the same question twice. */
    dialogue_style_note_spoken(s_reply);
    speech_budget_note((uint32_t)(esp_timer_get_time() / 1000));
    ambient_audio_mark_spoken();
}

static void voice_turn_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Voice turn task started (model %s)", VOICE_TURN_MODEL);

    for (;;) {
        voice_turn_req_t req;
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        s_busy = true;
        run_turn(req.window_ms);
        s_busy = false;
    }
}

esp_err_t voice_turn_start(void)
{
    if (s_queue) {
        return ESP_OK;
    }
    /* Depth 1: a second request while a turn is in flight must be REJECTED, not
     * buffered. Buffering would leave someone waiting through a turn they have
     * already forgotten about, and would let two turns fight for the mic. */
    s_queue = xQueueCreate(1, sizeof(voice_turn_req_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t ok =
        xTaskCreatePinnedToCore(voice_turn_task, "voice_turn", VOICE_TURN_TASK_STACK_SIZE, NULL,
                                VOICE_TURN_TASK_PRIORITY, NULL, VOICE_TURN_TASK_CORE);
    if (ok != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t voice_turn_request(uint32_t window_ms)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_clip_pcm_bytes(window_ms, MIC_SAMPLE_RATE_HZ) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mic_pdm_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Refuse while the robot is talking rather than recording it. The check is
     * here, at request time, so the console can say so immediately instead of
     * the turn failing silently a second later. */
    if (audio_player_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    const voice_turn_req_t req = {.window_ms = window_ms};
    return (xQueueSend(s_queue, &req, 0) == pdTRUE) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool voice_turn_is_busy(void)
{
    return s_busy;
}
