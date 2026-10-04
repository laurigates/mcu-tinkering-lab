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
#include "ambient_listener.h"
#include "audio_clip.h"
#include "audio_player.h"
#include "base64.h"
#include "buzzer.h"
#include "cJSON.h"
#include "camera.h"
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
#include "reactive_controller.h"
#include "speech_budget.h"
#include "speech_queue.h"
#include "voice_endpoint.h"
#include "voice_history.h"
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

/** Ceiling on a VAD turn's whole clip, pre-roll included (issue #616).
 *
 *  Endpointing normally ends the turn well before this; it is the memory bound
 *  for someone who keeps talking. 6 s keeps the PCM buffer at 192 kB, below the
 *  8 s `listen` ceiling the PSRAM budget in audio_clip.h was sized for, so a VAD
 *  turn can never be the one that breaks it. */
#define VOICE_TURN_VAD_MAX_MS 6000u

typedef struct {
    uint32_t window_ms; /**< Fixed window (`listen`), or the ceiling for a VAD turn. */
    bool vad;           /**< Pre-roll + end-of-speech instead of a fixed window. */
} voice_turn_req_t;

/** What one recording produced, for the per-turn log line. */
typedef struct {
    size_t samples;         /**< Total samples in the clip, pre-roll included. */
    size_t preroll_samples; /**< How many of them came from the pre-roll ring. */
    voice_endpoint_verdict_t end;
    uint32_t speech_frames;
    int16_t floor_db;
} record_result_t;

static QueueHandle_t s_queue;
static volatile bool s_busy;

/** Hands-free listening is ON at boot (issue #617). It was off while the
 *  trigger was a broadband loudness excursion, because a slam or the motors
 *  would start turns; the speech-shaped trigger (speech_trigger.h) is selective
 *  enough to leave on. `voice vad off` still disables it, until the next boot. */
static bool s_vad_enabled = true;

/** True from just before the start beep until its settle time has passed. The
 *  ambient listener reads it to keep the beep out of the pre-roll (as silence of
 *  the same length — see voice_preroll.h) and out of the ambient gate (see
 *  ambient_gate_accepts()). */
static volatile bool s_cue_active;

/** Endpointing knobs (`voice endpoint`). Not persisted, like every other voice
 *  threshold: a boot comes up at the documented defaults. max_ms is not used
 *  from here; it is set per turn from the buffer actually allocated. */
static voice_endpoint_cfg_t s_endpoint_cfg = {
    .min_ms = VOICE_ENDPOINT_MIN_MS_DEFAULT,
    .max_ms = VOICE_TURN_VAD_MAX_MS,
    .silence_ms = VOICE_ENDPOINT_SILENCE_MS_DEFAULT,
    .margin_db = VOICE_ENDPOINT_MARGIN_DB_DEFAULT,
};

/* Per-turn state at FILE scope, not on the 8 kB stack — the same reason
 * gemini_tts.c keeps its context static. The response buffer alone would be
 * half the stack. */
static char s_response[VOICE_TURN_RESPONSE_BUF_SIZE];
static char s_reply[SPEECH_TEXT_MAX];
static char s_sys_prompt[1536];

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

/** Read exactly @p n samples, absorbing short reads. Returns samples read. */
static size_t read_exact(int16_t *dst, size_t n, esp_err_t *err)
{
    size_t filled = 0;
    *err = ESP_OK;
    while (filled < n) {
        size_t got = 0;
        *err = mic_pdm_read(dst + filled, n - filled, &got, 1000);
        if (*err != ESP_OK || got == 0) {
            break;
        }
        filled += got;
    }
    return filled;
}

/**
 * @brief Record a VAD turn: the pre-roll, then frames until the speaker stops.
 *
 * Starts with the listener's pre-roll — the speech that set off the trigger —
 * and does NOT flush: the listener released the lock straight after offering its
 * last frame, so what the DMA holds now is the continuation of the ring, and the
 * clip is one unbroken stretch of audio. Then records listener-sized frames
 * until voice_endpoint says the speaker has stopped, or the buffer is full.
 * The caller holds the microphone lock.
 */
static size_t record_vad(int16_t *pcm, size_t samples, record_result_t *res, esp_err_t *err)
{
    size_t filled = ambient_listener_take_preroll(pcm, samples);
    res->preroll_samples = filled;

    voice_endpoint_cfg_t cfg = s_endpoint_cfg;
    cfg.max_ms = (uint32_t)(((uint64_t)(samples - filled) * 1000u) / MIC_SAMPLE_RATE_HZ);
    /* The floor as it stood before the turn. The listener is locked out for the
     * duration, so this is the room before anyone spoke — the reference speech
     * should be measured against, never a floor that has risen to meet it. */
    res->floor_db = ambient_audio_floor_db();

    /* The clock is derived from samples recorded rather than read from
     * esp_timer: it is the audio's own time, unaffected by the DMA backlog or by
     * this task being descheduled. Only the origin comes from the wall clock. */
    const uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &cfg, t0);

    size_t recorded = 0;
    *err = ESP_OK;
    while (filled < samples) {
        size_t want = samples - filled;
        if (want > AMBIENT_FRAME_SAMPLES) {
            want = AMBIENT_FRAME_SAMPLES;
        }
        const size_t got = read_exact(pcm + filled, want, err);
        if (got == 0) {
            break;
        }
        ambient_fingerprint_t fp;
        ambient_fingerprint_from_pcm(pcm + filled, got, &fp);
        filled += got;
        recorded += got;
        if (got < want || !fp.valid) {
            break; /* a failed read, or a tail too short to measure: the buffer is full */
        }
        const uint32_t now = t0 + (uint32_t)(((uint64_t)recorded * 1000u) / MIC_SAMPLE_RATE_HZ);
        res->end = voice_endpoint_update(&ep, fp.level_db, res->floor_db, now);
        if (res->end != VOICE_ENDPOINT_CONTINUE) {
            break;
        }
    }
    if (*err != ESP_OK) {
        ESP_LOGW(TAG, "listen: mic read failed %u ms into a VAD turn: %s",
                 (unsigned)((recorded * 1000u) / MIC_SAMPLE_RATE_HZ), esp_err_to_name(*err));
    }
    /* Leaving the loop without a verdict means the buffer filled on a tail too
     * short to measure, or a read failed (logged above). Either way the clip
     * ends where the memory did, so the log line must not read end=continue. */
    if (res->end == VOICE_ENDPOINT_CONTINUE) {
        res->end = VOICE_ENDPOINT_END_MAX;
    }
    res->speech_frames = ep.speech_frames;
    return filled;
}

/**
 * @brief Record into @p pcm, holding the microphone for the whole window.
 *
 * The ambient listener simply misses these frames, and that is correct rather
 * than merely tolerable: the noise floor must not learn from a conversation, or
 * a chat with the robot would raise the floor enough to deafen the gate
 * afterwards.
 *
 * A fixed-window turn (`listen`) flushes the DMA and records @p samples after
 * the beep, as it always has. A VAD turn keeps the triggering speech and stops
 * on silence instead (record_vad(), issue #616).
 *
 * Either way the pre-roll ring is emptied here. Once this function holds the
 * lock the listener stops reading, so anything left in the ring would be from
 * before this turn and would open the next one.
 */
static esp_err_t record_clip(int16_t *pcm, size_t samples, bool vad, record_result_t *res)
{
    memset(res, 0, sizeof(*res));
    res->end = VOICE_ENDPOINT_END_MAX;

    if (mic_pdm_lock(1000) != ESP_OK) {
        ESP_LOGW(TAG, "microphone busy");
        return ESP_ERR_INVALID_STATE;
    }

    size_t filled = 0;
    esp_err_t err = ESP_OK;
    if (vad) {
        filled = record_vad(pcm, samples, res, &err);
    } else {
        ambient_listener_take_preroll(NULL, 0); /* discard */
        /* Drop whatever the DMA accumulated while we were beeping and settling. */
        mic_pdm_flush();
        filled = read_exact(pcm, samples, &err);
    }
    mic_pdm_unlock();

    res->samples = filled;
    if (filled == 0) {
        return (err == ESP_OK) ? ESP_FAIL : err;
    }
    return ESP_OK;
}

/** Build the request body, taking ownership of nothing and freeing nothing. */
static char *build_body(const char *b64_wav, const char *b64_jpeg, const reactive_telemetry_t *tele,
                        bool has_telemetry, uint32_t now_ms)
{
    const voice_persona_t *persona = voice_persona_get();
    const char *name =
        (persona && persona->name && persona->name[0] != '\0') ? persona->name : "Robocar";

    char brief[512];
    if (persona && persona->text_brief) {
        if (persona->tag_brief && persona->tag_brief[0] != '\0') {
            snprintf(brief, sizeof(brief), "%s %s", persona->text_brief, persona->tag_brief);
        } else {
            snprintf(brief, sizeof(brief), "%s", persona->text_brief);
        }
    } else {
        snprintf(brief, sizeof(brief), "Be brief.");
    }

    snprintf(s_sys_prompt, sizeof(s_sys_prompt),
             "%s\n"
             "Listen to the audio.\n"
             "- If someone speaks to you (addresses you as %s or 'robotti', asks you a question, "
             "or makes a comment directed at you), answer them in ONE short spoken sentence in "
             "your persona.\n"
             "- If people are talking in the room and you are idle, you may occasionally chime in "
             "with a brief, dry witty remark.\n"
             "- If the audio contains only background noise, coughs, typing, or speech not "
             "directed at you, reply with '__IGNORE__'.",
             brief, name);

    return voice_history_build_request_body(name, s_sys_prompt, tele, has_telemetry, b64_jpeg,
                                            b64_wav, now_ms);
}

static void run_turn(uint32_t window_ms, bool vad)
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
    s_cue_active = true;
    buzzer_beep();
    vTaskDelay(pdMS_TO_TICKS(VOICE_TURN_BEEP_SETTLE_MS));
    s_cue_active = false;

    record_result_t rec;
    if (record_clip(pcm, samples, vad, &rec) != ESP_OK) {
        ESP_LOGE(TAG, "recording failed");
        heap_caps_free(pcm);
        return;
    }
    const size_t got = rec.samples;

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

    /* Camera multimodal context: capture JPEG frame if available. */
    char *b64_jpeg = NULL;
    camera_fb_t *fb = camera_capture();
    if (fb) {
        b64_jpeg = base64_encode_alloc(fb->buf, fb->len);
        camera_return_fb(fb);
        fb = NULL;
        if (!b64_jpeg) {
            ESP_LOGW(TAG, "camera JPEG base64 encode failed — continuing without image");
        }
    }

    /* Physical telemetry context: obstacle distance and reflex state. */
    reactive_telemetry_t tele = {0};
    const bool has_telemetry = (reactive_controller_get_telemetry(&tele) == ESP_OK);

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    char *body = build_body(b64, b64_jpeg, &tele, has_telemetry, now_ms);

    /* Free base64 buffers immediately after adding to request body to preserve PSRAM. */
    free(b64);
    b64 = NULL;
    if (b64_jpeg) {
        free(b64_jpeg);
        b64_jpeg = NULL;
    }

    if (!body) {
        ESP_LOGE(TAG, "request build failed");
        return;
    }

    const size_t body_len = strlen(body);
    if (vad) {
        /* clip= tracking the utterance is the bench check for issue #616: a short
         * question should end on `silence` well under the ceiling, and preroll=
         * near 1500 ms (the whole ring) shows the trigger's own words were kept. */
        ESP_LOGI(TAG, "listen: vad clip=%u ms preroll=%u ms end=%s speech=%u frames floor=%d dB",
                 (unsigned)((got * 1000u) / MIC_SAMPLE_RATE_HZ),
                 (unsigned)((rec.preroll_samples * 1000u) / MIC_SAMPLE_RATE_HZ),
                 voice_endpoint_verdict_name(rec.end), (unsigned)rec.speech_frames,
                 (int)rec.floor_db);
    }
    ESP_LOGI(TAG,
             "listen: window=%u ms samples=%u raw_peak=%d gain=%.1fx peak=%d clipped=%u dc=%d | "
             "upload=%u B | free PSRAM=%u B",
             (unsigned)window_ms, (unsigned)got, (int)st.raw_peak, (double)st.gain_q8 / 256.0,
             (int)st.peak, (unsigned)st.clipped, (int)st.dc, (unsigned)body_len,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    response_acc_t acc = {.buf = s_response, .len = 0, .cap = sizeof(s_response)};
    s_response[0] = '\0';
    int status = 0;
    const esp_err_t err =
        gemini_http_post(ACTIVITY_EP_VOICE_TURN, VOICE_TURN_URL, api_key, body,
                         VOICE_TURN_TIMEOUT_MS, http_event_handler, &acc, &status);
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

    if (voice_history_is_ignore(s_reply)) {
        ESP_LOGI(TAG, "listen: ignored (not addressed to robot)");
        return;
    }

    /* Record reply in conversational history ring */
    const uint32_t reply_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    voice_history_record(s_reply, reply_now_ms);

    const uint32_t latency_ms = reply_now_ms - t_start;
    ESP_LOGI(TAG, "listen: latency=%u ms reply=\"%s\"", (unsigned)latency_ms, s_reply);

    if (speech_queue_post(s_reply) != ESP_OK) {
        ESP_LOGW(TAG, "speech queue full — reply dropped");
        return;
    }

    const uint32_t post_speak_ms = (uint32_t)(esp_timer_get_time() / 1000);
    voice_history_mark_conversation_active(post_speak_ms);

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
        run_turn(req.window_ms, req.vad);
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

static esp_err_t enqueue(const voice_turn_req_t *req)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (audio_clip_pcm_bytes(req->window_ms, MIC_SAMPLE_RATE_HZ) == 0) {
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
    return (xQueueSend(s_queue, req, 0) == pdTRUE) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t voice_turn_request(uint32_t window_ms)
{
    const voice_turn_req_t req = {.window_ms = window_ms, .vad = false};
    return enqueue(&req);
}

esp_err_t voice_turn_request_vad(void)
{
    const voice_turn_req_t req = {.window_ms = VOICE_TURN_VAD_MAX_MS, .vad = true};
    return enqueue(&req);
}

bool voice_turn_cue_active(void)
{
    return s_cue_active;
}

void voice_turn_set_endpoint(uint32_t silence_ms, uint8_t margin_db)
{
    s_endpoint_cfg.silence_ms = silence_ms;
    s_endpoint_cfg.margin_db = margin_db;
}

void voice_turn_get_endpoint(uint32_t *silence_ms, uint8_t *margin_db, uint32_t *min_ms,
                             uint32_t *max_ms)
{
    if (silence_ms) {
        *silence_ms = s_endpoint_cfg.silence_ms;
    }
    if (margin_db) {
        *margin_db = s_endpoint_cfg.margin_db;
    }
    if (min_ms) {
        *min_ms = s_endpoint_cfg.min_ms;
    }
    if (max_ms) {
        *max_ms = VOICE_TURN_VAD_MAX_MS;
    }
}

bool voice_turn_is_busy(void)
{
    return s_busy;
}

void voice_turn_reset_history(void)
{
    voice_history_reset();
}

bool voice_turn_in_conversation(void)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return voice_history_in_conversation(now);
}

void voice_turn_set_vad(bool enabled)
{
    s_vad_enabled = enabled;
}

bool voice_turn_get_vad(void)
{
    return s_vad_enabled;
}
