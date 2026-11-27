/**
 * @file live_session.c
 * @brief Gemini Live session: WebSocket, microphone up, reply audio down.
 *
 * See live_session.h for what it does and ADR-032 for why. Every decision
 * (mute, close, turns, reassembly, gain) is in live_session_core.c, host-tested;
 * this file is the I/O around it.
 *
 * Two tasks touch a session:
 *
 *   - the voice-turn task (live_session_run()): connects, owns the mic lock,
 *     reads 100 ms chunks, sends them, sends tool responses, does the post-turn
 *     bookkeeping, and closes;
 *   - the WebSocket client's own task (ws_event()): reassembles server
 *     messages, decodes reply audio straight into the player, and records what
 *     each message said in the shared core state.
 *
 * The core state is shared under s_lock, held only for the core's short calls.
 * Reply audio is written to the player outside the lock: the write can block
 * while the 512 kB ring is full, and the voice-turn task must keep draining the
 * mic meanwhile. The downstream context (batch, utterance flag) belongs to the
 * WebSocket task alone until the client is destroyed.
 */

#include "live_session.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ambient_audio.h"
#include "ambient_listener.h"
#include "audio_clip.h"
#include "audio_player.h"
#include "dialogue_style.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "live_proto.h"
#include "live_session_core.h"
#include "mic_pdm.h"
#include "speech_budget.h"
#include "voice_budget.h"
#include "voice_history.h"
#include "voice_persona.h"
#include "voice_preroll.h"
#include "voice_turn.h"

static const char *TAG = "live";

/** 100 ms at 16 kHz, the probe's chunk: 3.2 kB of PCM, ~4.3 kB as JSON. */
#define LIVE_CHUNK_SAMPLES (LIVE_PROTO_INPUT_RATE_HZ / 10)

/** Connect + TLS + setupComplete. The planner's handshake has taken >5 s of
 *  software ECDHE on this chip (#948), so this is generous. */
#define LIVE_CONNECT_TIMEOUT_MS 15000u

/** Mic audio kept while the session connects, pre-roll included: the person
 *  keeps talking through the handshake, and all of it is sent once the setup
 *  is confirmed. 8 s is 256 kB of PSRAM; past it the oldest is kept and the
 *  rest dropped (logged). */
#define LIVE_BACKLOG_MS 8000u
#define LIVE_BACKLOG_SAMPLES ((LIVE_BACKLOG_MS * LIVE_PROTO_INPUT_RATE_HZ) / 1000u)

/** The WebSocket task does the TLS handshake: 8 kB like every TLS task here
 *  (.claude/rules/freertos-task-gotchas.md §2), plus cJSON and the decoder. */
#define LIVE_WS_TASK_STACK 8192
#define LIVE_WS_TASK_PRIO 4
/** Core 1 with the rest of the network and AI work; core 0 is the motors'. */
#define LIVE_WS_TASK_CORE 1
#define LIVE_WS_BUFFER_SIZE 4096

#define LIVE_SEND_TIMEOUT_MS 2000u
#define LIVE_RING_TIMEOUT_MS 4000u
#define LIVE_PCM_BATCH_BYTES 2048
#define LIVE_MIC_READ_TIMEOUT_MS 1000u

#define EV_CONNECTED BIT0
#define EV_SETUP_DONE BIT1
#define EV_FAILED BIT2

/* ----------------------------------------------------------------- state */

static bool s_enabled;
static uint32_t s_connect_failures;
static uint32_t s_idle_ms = LIVE_SESSION_IDLE_MS_DEFAULT;
static uint32_t s_max_ms = LIVE_SESSION_MAX_MS_DEFAULT;

static uint32_t s_total_sessions;
static uint32_t s_total_failures;
static uint32_t s_total_turns;
static uint32_t s_total_waits;

static volatile bool s_open;
static volatile bool s_stop_requested;

static SemaphoreHandle_t s_lock;
static EventGroupHandle_t s_events;
static live_session_t s_core;
static live_rx_t s_rx;

/** A finished spoken turn, handed from the WebSocket task to the voice-turn
 *  task for bookkeeping. Guarded by s_lock. */
static bool s_turn_done;
static char s_turn_done_text[LIVE_SESSION_TURN_TEXT_MAX];

/** Downstream: the WebSocket task's alone (see the file comment). */
typedef struct {
    uint8_t batch[LIVE_PCM_BATCH_BYTES];
    size_t batch_len;
    bool utterance_open;
    bool ring_stalled;
    int64_t first_audio_us;
} downstream_t;
static downstream_t s_down;

/** Mic chunks at file scope, not on the voice-turn task's 8 kB stack, which
 *  the TLS write inside every send also needs. One session at a time. */
static int16_t s_chunk[LIVE_CHUNK_SAMPLES];
static char s_said[LIVE_SESSION_TURN_TEXT_MAX];

/** Per-session timing for the summary line. */
static int64_t s_t_start_us;
static int64_t s_t_setup_us;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ------------------------------------------------------------ downstream */

static bool down_flush(downstream_t *d)
{
    if (d->batch_len == 0) {
        return true;
    }
    if (audio_player_write(d->batch, d->batch_len, LIVE_RING_TIMEOUT_MS) != ESP_OK) {
        d->ring_stalled = true;
        d->batch_len = 0;
        return false;
    }
    d->batch_len = 0;
    return true;
}

/** Decoded reply PCM: open the utterance on the first byte, batch like TTS
 *  (TTS_PCM_BATCH_BYTES in gemini_tts.c says why the batching matters). */
static bool pcm_sink(const uint8_t *data, size_t len, void *ctx)
{
    downstream_t *d = ctx;
    if (d->ring_stalled) {
        return false;
    }
    if (!d->utterance_open) {
        /* Opens the preroll gate's window: nothing plays until
         * AUDIO_PREROLL_BYTES have banked or the utterance ends. */
        audio_player_begin_utterance();
        d->utterance_open = true;
        if (d->first_audio_us == 0) {
            d->first_audio_us = esp_timer_get_time();
        }
    }
    while (len > 0) {
        const size_t room = sizeof(d->batch) - d->batch_len;
        const size_t n = (len < room) ? len : room;
        memcpy(d->batch + d->batch_len, data, n);
        d->batch_len += n;
        data += n;
        len -= n;
        if (d->batch_len == sizeof(d->batch) && !down_flush(d)) {
            return false;
        }
    }
    return true;
}

static void down_end_utterance(downstream_t *d)
{
    if (!d->utterance_open) {
        return;
    }
    if (d->ring_stalled) {
        /* abort() closes the gate itself; end_utterance() on top would reopen
         * it on an empty ring (gemini_tts.c speak() has the same rule). */
        audio_player_abort();
    } else {
        down_flush(d);
        audio_player_end_utterance();
    }
    d->utterance_open = false;
    d->ring_stalled = false;
}

static void down_abort(downstream_t *d)
{
    if (d->utterance_open) {
        audio_player_abort();
    }
    d->utterance_open = false;
    d->ring_stalled = false;
    d->batch_len = 0;
}

/* ------------------------------------------------------- WebSocket task */

static void handle_message(const char *json, size_t len)
{
    live_server_msg_t m;
    if (live_proto_parse_server(json, len, &m, pcm_sink, &s_down) != 0) {
        ESP_LOGW(TAG, "unparseable server message (%u B): %.120s", (unsigned)len, json);
        return;
    }
    if (m.audio_parts_skipped) {
        ESP_LOGW(TAG, "skipped %u reply part(s) not in %s", (unsigned)m.audio_parts_skipped,
                 LIVE_PROTO_OUTPUT_MIME);
    }
    if (m.sink_aborted) {
        ESP_LOGW(TAG, "playback ring stalled — reply truncated");
    }
    if (m.input_text[0]) {
        ESP_LOGI(TAG, "heard: \"%s\"", m.input_text);
    }
    if (m.go_away) {
        ESP_LOGW(TAG, "goAway: %u ms left — closing", (unsigned)m.go_away_ms);
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const unsigned ev = live_session_note_server(&s_core, &m, now_ms());
    if (ev & LIVE_EV_TURN_END) {
        memcpy(s_turn_done_text, s_core.turn_text, sizeof(s_turn_done_text));
        s_turn_done = true;
    }
    xSemaphoreGive(s_lock);

    if (m.setup_complete) {
        xEventGroupSetBits(s_events, EV_SETUP_DONE);
    }
    if (ev & LIVE_EV_TURN_END) {
        down_end_utterance(&s_down);
    }
    if (ev & LIVE_EV_INTERRUPTED) {
        down_abort(&s_down);
    }
}

static void log_close_frame(const esp_websocket_event_data_t *d)
{
    /* The close code and reason are the server's whole diagnosis (1007
     * "invalid argument" named the rejected setup field on 2026-11-16), so they
     * go out at ERROR, like every Gemini error body. */
    if (d->data_len >= 2) {
        const uint8_t *p = (const uint8_t *)d->data_ptr;
        const unsigned code = ((unsigned)p[0] << 8) | p[1];
        ESP_LOGE(TAG, "server closed: %u %.*s", code, d->data_len - 2, (const char *)p + 2);
    } else {
        ESP_LOGE(TAG, "server closed (no code)");
    }
}

static void ws_event(void *arg, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)arg;
    (void)base;
    const esp_websocket_event_data_t *d = event_data;
    switch (id) {
        case WEBSOCKET_EVENT_CONNECTED:
            xEventGroupSetBits(s_events, EV_CONNECTED);
            break;
        case WEBSOCKET_EVENT_DATA:
            if (d->op_code == 0x08) {
                log_close_frame(d);
                break;
            }
            /* Text (1), binary (2) and continuation (0) carry the JSON; the
             * server sends its messages as binary frames. */
            if (d->op_code > 0x02) {
                break;
            }
            {
                const int r =
                    live_rx_feed(&s_rx, d->data_ptr, (size_t)d->data_len, (size_t)d->payload_offset,
                                 (size_t)d->payload_len, d->fin);
                if (r == 1) {
                    handle_message(s_rx.buf, s_rx.len);
                } else if (r < 0) {
                    ESP_LOGW(TAG, "server message over %u B dropped", (unsigned)LIVE_RX_MAX_BYTES);
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
        case WEBSOCKET_EVENT_DISCONNECTED:
        case WEBSOCKET_EVENT_CLOSED:
            xSemaphoreTake(s_lock, portMAX_DELAY);
            live_session_fail(&s_core);
            xSemaphoreGive(s_lock);
            xEventGroupSetBits(s_events, EV_FAILED);
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------- voice-turn task */

static size_t read_chunk(int16_t *dst, size_t n)
{
    size_t filled = 0;
    while (filled < n) {
        size_t got = 0;
        if (mic_pdm_read(dst + filled, n - filled, &got, LIVE_MIC_READ_TIMEOUT_MS) != ESP_OK ||
            got == 0) {
            break;
        }
        filled += got;
    }
    return filled;
}

static bool send_text(esp_websocket_client_handle_t c, const char *msg, size_t len)
{
    const int r =
        esp_websocket_client_send_text(c, msg, (int)len, pdMS_TO_TICKS(LIVE_SEND_TIMEOUT_MS));
    return r == (int)len;
}

/** Send @p samples of PCM as 100 ms messages, back to back. */
static bool send_pcm(esp_websocket_client_handle_t c, const int16_t *pcm, size_t samples, char *msg,
                     size_t msg_cap)
{
    for (size_t off = 0; off < samples; off += LIVE_CHUNK_SAMPLES) {
        const size_t n = (samples - off < LIVE_CHUNK_SAMPLES) ? samples - off : LIVE_CHUNK_SAMPLES;
        const size_t len = live_proto_build_audio(pcm + off, n, msg, msg_cap);
        if (len == 0 || !send_text(c, msg, len)) {
            return false;
        }
    }
    return true;
}

/** True when a raw chunk is speech by the VAD turn's rule: its level above the
 *  room floor by the endpoint margin (voice_endpoint.h). */
static bool chunk_is_speech(const int16_t *pcm, size_t n, int16_t floor_db, uint8_t margin_db)
{
    ambient_fingerprint_t fp;
    ambient_fingerprint_from_pcm(pcm, n, &fp);
    return fp.valid && margin_db > 0 && fp.level_db >= floor_db + (int16_t)margin_db;
}

/** After a spoken turn: the same rulings as voice_turn.c's run_turn(). */
static void turn_bookkeeping(const char *text)
{
    const uint32_t t = now_ms();
    ESP_LOGI(TAG, "said: \"%s\"", text);
    if (text[0] != '\0') {
        voice_history_record(text, t);
        dialogue_style_note_spoken(text);
    }
    voice_history_mark_conversation_active(t);
    speech_budget_note(t);
    ambient_audio_mark_spoken();
}

static char *build_setup(void)
{
    const voice_persona_t *p = voice_persona_get();
    char voice[VOICE_PERSONA_VOICE_MAX];
    voice_persona_effective_voice(voice, sizeof(voice));
    char *instruction =
        live_proto_build_instruction((p && p->name) ? p->name : "Robocar", p ? p->text_brief : NULL,
                                     p ? p->tts_style : NULL, p ? p->language_code : NULL);
    if (!instruction) {
        return NULL;
    }
    char *setup = live_proto_build_setup(LIVE_PROTO_MODEL, voice, instruction);
    free(instruction);
    return setup;
}

static esp_websocket_client_handle_t connect_client(const char *api_key, char *headers,
                                                    size_t headers_cap)
{
    snprintf(headers, headers_cap, "x-goog-api-key: %s\r\n", api_key);
    const esp_websocket_client_config_t cfg = {
        .uri = LIVE_PROTO_URI,
        .headers = headers,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .disable_auto_reconnect = true,
        .task_stack = LIVE_WS_TASK_STACK,
        .task_prio = LIVE_WS_TASK_PRIO,
        .task_core_id_set = true,
        .task_core_id = LIVE_WS_TASK_CORE,
        .task_name = "live_ws",
        .buffer_size = LIVE_WS_BUFFER_SIZE,
        .network_timeout_ms = 10000,
    };
    esp_websocket_client_handle_t c = esp_websocket_client_init(&cfg);
    if (!c) {
        return NULL;
    }
    esp_websocket_register_events(c, WEBSOCKET_EVENT_ANY, ws_event, NULL);
    if (esp_websocket_client_start(c) != ESP_OK) {
        esp_websocket_client_destroy(c);
        return NULL;
    }
    return c;
}

static void note_connect_failure(const char *why)
{
    s_total_failures++;
    s_connect_failures++;
    ESP_LOGE(TAG, "session did not open: %s (%u in a row)", why, (unsigned)s_connect_failures);
    if (s_connect_failures >= LIVE_SESSION_MAX_CONNECT_FAILURES) {
        s_enabled = false;
        ESP_LOGE(TAG,
                 "%u failed opens in a row — live mode off, hands-free turns use the old path "
                 "until `voice live on`",
                 (unsigned)s_connect_failures);
    }
}

esp_err_t live_session_run(const char *api_key)
{
    if (!api_key || api_key[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        s_events = xEventGroupCreate();
        if (!s_lock || !s_events) {
            return ESP_ERR_NO_MEM;
        }
    }

    const size_t msg_cap = live_proto_audio_msg_size(LIVE_CHUNK_SAMPLES * sizeof(int16_t));
    char *msg = heap_caps_malloc(msg_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int16_t *backlog = heap_caps_malloc(LIVE_BACKLOG_SAMPLES * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    char *setup = build_setup();
    if (!msg || !backlog || !setup) {
        ESP_LOGE(TAG, "no memory for a session");
        heap_caps_free(msg);
        heap_caps_free(backlog);
        free(setup);
        return ESP_ERR_NO_MEM;
    }

    const size_t psram_start = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    s_t_start_us = esp_timer_get_time();
    s_t_setup_us = 0;
    memset(&s_down, 0, sizeof(s_down));
    s_turn_done = false;
    s_stop_requested = false;
    live_rx_init(&s_rx, LIVE_RX_MAX_BYTES);
    const live_session_cfg_t cfg = {
        .idle_ms = s_idle_ms,
        .max_ms = s_max_ms,
        .hangover_ms = AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT,
    };
    live_session_begin(&s_core, &cfg, now_ms());
    xEventGroupClearBits(s_events, EV_CONNECTED | EV_SETUP_DONE | EV_FAILED);
    s_open = true;

    /* Start the handshake first: it runs on the client's task while this one
     * keeps the microphone, so nothing said during it is lost. */
    char headers[160];
    esp_websocket_client_handle_t client = connect_client(api_key, headers, sizeof(headers));
    esp_err_t result = ESP_OK;
    bool mic_locked = false;
    size_t backlog_n = 0;
    size_t backlog_dropped = 0;
    audio_clip_stats_t gain = {.gain_q8 = 256};
    int16_t floor_db = ambient_audio_floor_db();
    uint8_t margin_db = 0;
    voice_turn_get_endpoint(NULL, &margin_db, NULL, NULL);
    live_end_t end = LIVE_END_ERROR;
    uint32_t speech_chunks = 0;

    if (!client) {
        note_connect_failure("client start failed");
        result = ESP_FAIL;
        goto out;
    }

    if (mic_pdm_lock(1000) != ESP_OK) {
        ESP_LOGW(TAG, "microphone busy");
        result = ESP_ERR_INVALID_STATE;
        goto out;
    }
    mic_locked = true;

    /* The words that set off the trigger, then the DMA's continuation of them
     * (no flush, as in a VAD turn: voice_turn.c record_vad()). */
    backlog_n = ambient_listener_take_preroll(backlog, LIVE_BACKLOG_SAMPLES);
    /* One gain for the session, from the speech that opened it, so the level
     * does not pump between chunks (live_session_core.h). */
    if (backlog_n > 0) {
        audio_clip_normalise(backlog, backlog_n, &gain);
    }

    while (true) {
        const EventBits_t bits = xEventGroupGetBits(s_events);
        if (bits & (EV_SETUP_DONE | EV_FAILED)) {
            break;
        }
        if (bits & EV_CONNECTED) {
            /* Once per session: the bit is cleared after the send. */
            xEventGroupClearBits(s_events, EV_CONNECTED);
            if (!send_text(client, setup, strlen(setup))) {
                note_connect_failure("setup send failed");
                result = ESP_FAIL;
                goto out;
            }
        }
        if ((uint32_t)((esp_timer_get_time() - s_t_start_us) / 1000) > LIVE_CONNECT_TIMEOUT_MS) {
            note_connect_failure("timed out before setupComplete");
            result = ESP_ERR_TIMEOUT;
            goto out;
        }
        int16_t *dst = backlog + backlog_n;
        const bool room = backlog_n + LIVE_CHUNK_SAMPLES <= LIVE_BACKLOG_SAMPLES;
        const size_t got = read_chunk(room ? dst : s_chunk, LIVE_CHUNK_SAMPLES);
        if (room) {
            if (backlog_n == 0 && gain.samples == 0) {
                audio_clip_normalise(dst, got, &gain);
            } else {
                live_gain_apply(dst, got, gain.dc, gain.gain_q8);
            }
            backlog_n += got;
        } else {
            backlog_dropped += got;
        }
    }

    if (!(xEventGroupGetBits(s_events) & EV_SETUP_DONE)) {
        note_connect_failure("closed before setupComplete");
        result = ESP_FAIL;
        goto out;
    }

    s_t_setup_us = esp_timer_get_time();
    s_connect_failures = 0;
    s_total_sessions++;
    /* Charged once per session, when the server has accepted it: one request
     * against the hands-free ration and the per-boot ceiling (ADR-032 §4). */
    voice_budget_note_request(now_ms(), true);
    ESP_LOGI(TAG,
             "session open: setup %u ms, backlog %u ms (pre-roll included, %u ms dropped), "
             "gain %u.%02ux",
             (unsigned)((s_t_setup_us - s_t_start_us) / 1000),
             (unsigned)((backlog_n * 1000u) / LIVE_PROTO_INPUT_RATE_HZ),
             (unsigned)((backlog_dropped * 1000u) / LIVE_PROTO_INPUT_RATE_HZ),
             (unsigned)(gain.gain_q8 / 256u), (unsigned)((gain.gain_q8 % 256u) * 100u / 256u));

    if (backlog_n > 0 && !send_pcm(client, backlog, backlog_n, msg, msg_cap)) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        live_session_fail(&s_core);
        xSemaphoreGive(s_lock);
    }

    bool was_allowed = true;
    int16_t *const chunk = s_chunk;
    char *const said = s_said;
    for (;;) {
        const size_t got = read_chunk(chunk, LIVE_CHUNK_SAMPLES);
        const uint32_t t = now_ms();
        const bool speech = got > 0 && chunk_is_speech(chunk, got, floor_db, margin_db);
        if (got > 0) {
            live_gain_apply(chunk, got, gain.dc, gain.gain_q8);
        }
        const bool playing = audio_player_is_active();

        char tool_id[LIVE_PROTO_TOOL_ID_MAX] = {0};
        char tool_name[LIVE_PROTO_TOOL_NAME_MAX] = {0};
        bool turn_done = false;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_stop_requested) {
            live_session_stop(&s_core);
        }
        if (speech) {
            live_session_note_speech(&s_core, t);
            speech_chunks++;
        }
        const bool allowed = got > 0 && live_session_upstream_allowed(&s_core, playing, t);
        if (s_core.tool_pending) {
            memcpy(tool_id, s_core.tool_id, sizeof(tool_id));
            memcpy(tool_name, s_core.tool_name, sizeof(tool_name));
            live_session_tool_sent(&s_core);
        }
        if (s_turn_done) {
            memcpy(said, s_turn_done_text, sizeof(s_said));
            s_turn_done = false;
            turn_done = true;
        }
        end = live_session_should_end(&s_core, playing, t);
        xSemaphoreGive(s_lock);

        bool ok = true;
        if (allowed) {
            const size_t len = live_proto_build_audio(chunk, got, msg, msg_cap);
            ok = len > 0 && send_text(client, msg, len);
        } else if (was_allowed) {
            /* The stream pauses for the reply: tell the server, so its VAD
             * closes the turn on what it has rather than waiting for more. */
            const char *eos = live_proto_audio_stream_end();
            ok = send_text(client, eos, strlen(eos));
        }
        was_allowed = allowed;

        if (ok && tool_id[0] != '\0') {
            char *resp = live_proto_build_tool_response(tool_id, tool_name);
            ok = resp && send_text(client, resp, strlen(resp));
            free(resp);
            if (strcmp(tool_name, LIVE_PROTO_WAIT_TOOL) == 0) {
                /* A normal outcome, not a misfire: the model judged that no
                 * reaction was needed. Counted on the `live:` line only, not as
                 * an ignored voice turn, whose count means "trigger too loose". */
                ESP_LOGI(TAG, "wait (no reaction needed)");
            }
        }
        if (turn_done) {
            turn_bookkeeping(said);
        }
        if (!ok) {
            ESP_LOGW(TAG, "send failed — closing");
            xSemaphoreTake(s_lock, portMAX_DELAY);
            live_session_fail(&s_core);
            xSemaphoreGive(s_lock);
            end = LIVE_END_ERROR;
        }
        if (end != LIVE_END_NONE) {
            break;
        }
    }

out:
    if (mic_locked) {
        mic_pdm_unlock();
    }
    if (client) {
        if (esp_websocket_client_is_connected(client)) {
            esp_websocket_client_close(client, pdMS_TO_TICKS(2000));
        }
        esp_websocket_client_destroy(client); /* joins the client's task */
    }
    memset(headers, 0, sizeof(headers)); /* the key */
    /* The client's task is gone, so the downstream context is this task's. */
    down_end_utterance(&s_down);
    live_rx_free(&s_rx);

    if (result == ESP_OK) {
        const uint32_t secs = (uint32_t)((esp_timer_get_time() - s_t_setup_us) / 1000000);
        s_total_turns += s_core.turns;
        s_total_waits += s_core.waits;
        ESP_LOGI(
            TAG,
            "session closed: %s after %u s | turns=%u waits=%u interrupts=%u | "
            "chunks sent=%u muted=%u speech=%u | reply %u ms audio | tokens=%u | "
            "first audio %d ms after open | PSRAM low %u B (start %u)",
            live_end_name(end), (unsigned)secs, (unsigned)s_core.turns, (unsigned)s_core.waits,
            (unsigned)s_core.interrupts, (unsigned)s_core.chunks_sent,
            (unsigned)s_core.chunks_muted, (unsigned)speech_chunks,
            (unsigned)(s_core.audio_bytes * 1000u / (LIVE_PROTO_OUTPUT_RATE_HZ * sizeof(int16_t))),
            (unsigned)s_core.total_tokens,
            s_down.first_audio_us ? (int)((s_down.first_audio_us - s_t_setup_us) / 1000) : -1,
            (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM), (unsigned)psram_start);
    }
    /* The words heard have been dealt with, by the session or by its failure;
     * the latch they armed must not make the planner remark on them. */
    ambient_audio_mark_spoken();

    heap_caps_free(msg);
    heap_caps_free(backlog);
    free(setup);
    s_open = false;
    return result;
}

/* ---------------------------------------------------------------- knobs */

void live_session_set_enabled(bool enabled)
{
    s_enabled = enabled;
    s_connect_failures = 0;
}

bool live_session_enabled(void)
{
    return s_enabled;
}

void live_session_configure(uint32_t idle_ms, uint32_t max_ms)
{
    if (idle_ms) {
        s_idle_ms = idle_ms;
    }
    if (max_ms) {
        s_max_ms = max_ms;
    }
}

void live_session_get_config(uint32_t *idle_ms, uint32_t *max_ms)
{
    if (idle_ms) {
        *idle_ms = s_idle_ms;
    }
    if (max_ms) {
        *max_ms = s_max_ms;
    }
}

bool live_session_is_open(void)
{
    return s_open;
}

void live_session_request_stop(void)
{
    s_stop_requested = true;
}

void live_session_stats(uint32_t *sessions, uint32_t *failures, uint32_t *turns, uint32_t *waits)
{
    if (sessions) {
        *sessions = s_total_sessions;
    }
    if (failures) {
        *failures = s_total_failures;
    }
    if (turns) {
        *turns = s_total_turns;
    }
    if (waits) {
        *waits = s_total_waits;
    }
}
