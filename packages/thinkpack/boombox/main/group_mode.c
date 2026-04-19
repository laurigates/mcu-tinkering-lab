/**
 * @file group_mode.c
 * @brief Mesh event handling: sync-pulse beat alignment, hello chime,
 *        PLAY_MELODY via command_executor, leader sync-pulse broadcast timer.
 *
 * The hello chime (C5 E5 G5, 80 ms each) is executed note-by-note in
 * group_mode_tick() using a small state machine so it never blocks the
 * mesh receive task.
 *
 * CMD_PLAY_MELODY is registered with command_executor in group_mode_init;
 * the executor unpacks the envelope and invokes play_melody_handler,
 * which installs a temporary pattern override for N beats.
 *
 * The sync-pulse broadcast timer is a FreeRTOS timer created on
 * BECAME_LEADER and deleted on BECAME_FOLLOWER. Timer period is
 * derived from the current tempo pot reading at the moment of election
 * and updated lazily — sufficient for the musical-round use case.
 */

#include "group_mode.h"

#include <stdint.h>
#include <string.h>

#include "command_executor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "led_single.h"
#include "melody_gen.h"
#include "pot_reader.h"
#include "standalone_mode.h"
#include "thinkpack_commands.h"
#include "thinkpack_protocol.h"
#include "tone_engine.h"

static const char *TAG = "group_mode";

/* ------------------------------------------------------------------ */
/* Hello chime                                                         */
/* ------------------------------------------------------------------ */

#define CHIME_NOTE_COUNT 3
#define CHIME_NOTE_MS 80u

static const note_t s_chime_notes[CHIME_NOTE_COUNT] = {NOTE_C5, NOTE_E5, NOTE_G5};

typedef enum {
    CHIME_IDLE = 0,
    CHIME_PLAYING,
    CHIME_GAP,
} chime_state_t;

static chime_state_t s_chime_state;
static uint8_t s_chime_step;
static uint32_t s_chime_deadline_ms;

/* ------------------------------------------------------------------ */
/* Temporary pattern override (CMD_PLAY_MELODY)                        */
/* ------------------------------------------------------------------ */

static bool s_override_active;
static melody_pattern_t s_saved_pattern;
static uint32_t s_override_end_beat;

/* ------------------------------------------------------------------ */
/* Leader sync-pulse timer                                             */
/* ------------------------------------------------------------------ */

static TimerHandle_t s_sync_timer;

static void sync_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* Broadcast a sync pulse so followers can align their beat index */
    uint16_t bpm = pot_tempo_to_bpm(pot_reader_get_raw(POT_TEMPO));
    uint32_t period_ms = 60000u / bpm;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint8_t phase = (uint8_t)(standalone_mode_get_beat_index() & 0xFFu);

    thinkpack_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t local_mac[6];
    thinkpack_mesh_get_mac(local_mac);
    thinkpack_prepare_sync_pulse(&pkt, 0, local_mac, now_ms, phase);

    esp_err_t err = thinkpack_mesh_send(NULL, &pkt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Sync pulse send failed: %s", esp_err_to_name(err));
    }

    /* Re-arm at tempo-relative period (quarter-beat = period/4) */
    uint32_t timer_ms = period_ms / 4u;
    if (timer_ms < 50u)
        timer_ms = 50u;
    xTimerChangePeriod(s_sync_timer, pdMS_TO_TICKS(timer_ms), 0);
}

/* ------------------------------------------------------------------ */
/* Command handlers                                                    */
/* ------------------------------------------------------------------ */

/** Handler for CMD_PLAY_MELODY — registered with command_executor. */
static void play_melody_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                                void *user_ctx)
{
    (void)command_id;
    (void)user_ctx;

    if (length < sizeof(cmd_play_melody_payload_t)) {
        ESP_LOGW(TAG, "CMD_PLAY_MELODY: payload too short (%u bytes)", length);
        return;
    }

    cmd_play_melody_payload_t p;
    memcpy(&p, payload, sizeof(p));

    melody_pattern_t new_pattern = (melody_pattern_t)p.pattern_id;
    if (new_pattern >= MELODY_COUNT || p.repeat_count == 0) {
        ESP_LOGW(TAG, "CMD_PLAY_MELODY: bad args pattern=%u repeat=%u", p.pattern_id,
                 p.repeat_count);
        return;
    }

    s_saved_pattern = melody_gen_get_pattern();
    melody_gen_set_pattern(new_pattern);
    s_override_end_beat = standalone_mode_get_beat_index() + (uint32_t)p.repeat_count;
    s_override_active = true;
    ESP_LOGI(TAG, "PLAY_MELODY: pattern %d for %u beats", (int)new_pattern, p.repeat_count);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void group_mode_init(void)
{
    s_chime_state = CHIME_IDLE;
    s_chime_step = 0;
    s_chime_deadline_ms = 0;
    s_override_active = false;
    s_saved_pattern = MELODY_MARCH;
    s_override_end_beat = 0;
    s_sync_timer = NULL;

    esp_err_t err = command_executor_register(CMD_PLAY_MELODY, play_melody_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CMD_PLAY_MELODY handler: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Group mode initialized");
}

void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;

    switch (event->type) {
        case THINKPACK_EVENT_SYNC_PULSE: {
            /* Align beat index to leader's phase */
            const thinkpack_sync_pulse_data_t *pulse =
                (const thinkpack_sync_pulse_data_t *)event->packet->data;
            uint32_t leader_beat = (uint32_t)pulse->phase;
            standalone_mode_set_beat_index(leader_beat);
            ESP_LOGD(TAG, "Sync pulse: aligned beat -> %lu", (unsigned long)leader_beat);
            break;
        }

        case THINKPACK_EVENT_PEER_DISCOVERED:
            /* Trigger hello chime — does not disturb beat index */
            if (s_chime_state == CHIME_IDLE) {
                s_chime_step = 0;
                s_chime_state = CHIME_PLAYING;
                s_chime_deadline_ms = 0; /* fire immediately in next tick */
            }
            ESP_LOGI(TAG, "Peer discovered: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_COMMAND_RECEIVED:
            if (event->packet != NULL) {
                command_executor_dispatch(event->packet);
            }
            break;

        case THINKPACK_EVENT_BECAME_LEADER: {
            ESP_LOGI(TAG, "Became leader — starting sync-pulse timer");
            uint16_t bpm = pot_tempo_to_bpm(pot_reader_get_raw(POT_TEMPO));
            uint32_t period_ms = 60000u / bpm;
            uint32_t timer_ms = period_ms / 4u;
            if (timer_ms < 50u)
                timer_ms = 50u;

            if (s_sync_timer == NULL) {
                s_sync_timer = xTimerCreate("sync_pulse", pdMS_TO_TICKS(timer_ms), pdTRUE, NULL,
                                            sync_timer_cb);
            }
            if (s_sync_timer != NULL) {
                xTimerChangePeriod(s_sync_timer, pdMS_TO_TICKS(timer_ms), 0);
                xTimerStart(s_sync_timer, 0);
            }
            led_single_set(0, 255, 128); /* teal = leader */
            break;
        }

        case THINKPACK_EVENT_BECAME_FOLLOWER:
            ESP_LOGI(TAG, "Became follower — stopping sync-pulse timer");
            if (s_sync_timer != NULL) {
                xTimerStop(s_sync_timer, 0);
            }
            led_single_clear();
            break;

        case THINKPACK_EVENT_PEER_LOST:
            ESP_LOGW(TAG, "Peer lost: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_ELECTED:
            ESP_LOGI(TAG, "Leader elected: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_LOST:
            ESP_LOGW(TAG, "Leader lost — re-election in progress");
            break;

        default:
            break;
    }
}

void group_mode_tick(uint32_t now_ms)
{
    /* Restore pattern after temporary override */
    if (s_override_active) {
        if (standalone_mode_get_beat_index() >= s_override_end_beat) {
            melody_gen_set_pattern(s_saved_pattern);
            s_override_active = false;
            ESP_LOGI(TAG, "PLAY_MELODY override ended, restored pattern %d", (int)s_saved_pattern);
        }
    }

    /* Hello chime state machine */
    switch (s_chime_state) {
        case CHIME_IDLE:
            break;

        case CHIME_PLAYING:
            if (now_ms >= s_chime_deadline_ms) {
                tone_engine_play(s_chime_notes[s_chime_step], 0);
                led_single_flash(0, 200, 255, CHIME_NOTE_MS);
                s_chime_deadline_ms = now_ms + CHIME_NOTE_MS;
                s_chime_state = CHIME_GAP;
            }
            break;

        case CHIME_GAP:
            if (now_ms >= s_chime_deadline_ms) {
                tone_engine_stop();
                s_chime_step++;
                if (s_chime_step >= CHIME_NOTE_COUNT) {
                    s_chime_state = CHIME_IDLE;
                } else {
                    s_chime_deadline_ms = now_ms + 10u; /* 10 ms gap between notes */
                    s_chime_state = CHIME_PLAYING;
                }
            }
            break;

        default:
            s_chime_state = CHIME_IDLE;
            break;
    }
}
