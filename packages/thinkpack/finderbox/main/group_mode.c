/**
 * @file group_mode.c
 * @brief Mesh-aware group behaviour for Finderbox.
 *
 * SEEK (Hot-Cold):
 *   When a SEEK tag is scanned locally, all subsequent peer beacons are
 *   sampled for RSSI (via group_manager_find) at ~10 Hz by a small task
 *   that feeds hot_cold_update and repaints the LED ring.  A partner
 *   Glowbug (if any) mirrors the current colour via CMD_LED_PATTERN.
 *
 * STORY (NFC Story Sounds):
 *   When a STORY tag is scanned, Finderbox unicasts an MSG_LLM_REQUEST to
 *   the mesh leader (or broadcasts if none known) carrying the tag UID
 *   and label as the prompt hint.  The Brainbox replies with a short JSON
 *   sound sequence via MSG_LLM_RESPONSE (possibly fragmented); on receipt
 *   we parse it and dispatch via piezo + MSG_AUDIO_CLIP_BROADCAST relays
 *   to Chatterbox peers.
 *
 * All mesh sends happen off the receive-task critical path — either on
 * the SEEK polling task or on the standalone scan task (via
 * group_mode_on_local_scan).
 */

#include "group_mode.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "command_dispatcher.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_manager.h"
#include "hot_cold.h"
#include "led_ring.h"
#include "nfc_story_sounds.h"
#include "piezo.h"
#include "thinkpack_commands.h"
#include "thinkpack_protocol.h"

static const char *TAG = "group_mode";

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */

/** LED repaint cadence while SEEK is active. */
#define SEEK_POLL_INTERVAL_MS 100
/** Stale threshold after which SEEK reverts to "cold" if no beacon. */
#define SEEK_STALE_MS 3000
/** Piezo frequency for STORY-sequence tone steps. */
#define STORY_TONE_DEFAULT_HZ 880
/** Minimum param clamp for tone steps (Hz). */
#define STORY_TONE_MIN_HZ 200
/** Maximum param clamp for tone steps (Hz). */
#define STORY_TONE_MAX_HZ 6000
/** Duration of a single story tone step (ms). */
#define STORY_TONE_DURATION_MS 150

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static thinkpack_nfc_registry_t *s_registry = NULL;
static hot_cold_state_t s_hot_cold;
static bool s_seek_active = false;
static TaskHandle_t s_seek_task = NULL;
static uint8_t s_leader_mac[6];
static bool s_have_leader = false;
static uint8_t s_seq = 0;

/* Most-recently scanned STORY tag context — retained for the LLM reply. */
static uint8_t s_story_uid[THINKPACK_NFC_UID_MAX_LEN];
static uint8_t s_story_uid_len = 0;
static bool s_story_pending = false;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Fill out_mac with local WiFi MAC; used as src for our own packets. */
static void fill_own_mac(uint8_t out_mac[6])
{
    if (esp_read_mac(out_mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        memset(out_mac, 0, 6);
    }
}

/* Mirror the current hot-cold colour to any partner Glowbug.  Sent as
 * CMD_LED_PATTERN SOLID so the standard command_executor handles it. */
static void broadcast_color_mirror(uint8_t r, uint8_t g, uint8_t b)
{
    cmd_led_pattern_payload_t p = {
        .r = r,
        .g = g,
        .b = b,
        .pattern = (uint8_t)LED_PATTERN_SOLID,
    };
    uint8_t own_mac[6];
    fill_own_mac(own_mac);

    thinkpack_packet_t pkt;
    command_build_led_pattern(&pkt, s_seq++, own_mac, &p);
    (void)thinkpack_mesh_send(NULL, &pkt); /* broadcast */
}

/* ------------------------------------------------------------------ */
/* SEEK polling task                                                   */
/* ------------------------------------------------------------------ */

static void seek_task(void *arg)
{
    (void)arg;

    uint32_t last_beacon_ms = 0;

    for (;;) {
        if (!s_seek_active) {
            /* Task parks itself when SEEK is cleared. */
            vTaskDelay(pdMS_TO_TICKS(SEEK_POLL_INTERVAL_MS));
            continue;
        }

        /* Pick the "nearest-looking" peer: highest RSSI among current peers. */
        size_t pc = group_manager_count();
        int8_t best_rssi = INT8_MIN;
        bool have_any = false;

        for (size_t i = 0; i < pc; i++) {
            const thinkpack_peer_t *peer = group_manager_get(i);
            if (peer == NULL) {
                continue;
            }
            if (!have_any || peer->rssi > best_rssi) {
                best_rssi = peer->rssi;
                have_any = true;
                last_beacon_ms = peer->last_seen_ms;
            }
        }

        uint32_t t = now_ms();
        if (have_any && (t - last_beacon_ms) < SEEK_STALE_MS) {
            hot_cold_update(&s_hot_cold, best_rssi);
        } else {
            /* Stale — fade toward cold by pushing the minimum RSSI in. */
            hot_cold_update(&s_hot_cold, (int8_t)HOT_COLD_RSSI_MIN);
        }

        uint8_t r, g, b;
        hot_cold_get_color(&s_hot_cold, &r, &g, &b);
        led_ring_fill(r, g, b);
        led_ring_show();
        broadcast_color_mirror(r, g, b);

        vTaskDelay(pdMS_TO_TICKS(SEEK_POLL_INTERVAL_MS));
    }
}

/* ------------------------------------------------------------------ */
/* STORY sequence dispatch                                             */
/* ------------------------------------------------------------------ */

/* clamp helper for tone frequencies. */
static uint32_t clamp_freq(int32_t hz)
{
    if (hz < STORY_TONE_MIN_HZ) {
        return (uint32_t)STORY_TONE_MIN_HZ;
    }
    if (hz > STORY_TONE_MAX_HZ) {
        return (uint32_t)STORY_TONE_MAX_HZ;
    }
    return (uint32_t)hz;
}

/* Relay a clip broadcast to any Chatterbox peers that might be in-mesh.
 * This is a metadata-only packet — the Chatterbox leader streams the PCM
 * separately (outside PR E's scope). */
static void relay_clip(uint8_t clip_id)
{
    audio_clip_broadcast_payload_t payload = {
        .clip_id = clip_id,
        .sample_count = 0,
        .flags = 0,
    };
    memset(payload.per_peer_semitone_shift, 0, sizeof(payload.per_peer_semitone_shift));

    uint8_t own_mac[6];
    fill_own_mac(own_mac);

    thinkpack_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.msg_type = (uint8_t)MSG_AUDIO_CLIP_BROADCAST;
    pkt.sequence_number = s_seq++;
    memcpy(pkt.src_mac, own_mac, 6);
    pkt.data_length = (uint8_t)sizeof(payload);
    memcpy(pkt.data, &payload, sizeof(payload));
    thinkpack_finalize(&pkt);
    (void)thinkpack_mesh_send(NULL, &pkt);
}

/* Dispatcher passed to nfc_story_sounds_execute. */
static int story_step_dispatch(uint8_t kind, int32_t param, void *user_ctx)
{
    (void)user_ctx;

    switch ((nfc_story_kind_t)kind) {
        case NFC_STORY_KIND_TONE:
            piezo_play_tone(clamp_freq(param), STORY_TONE_DURATION_MS);
            vTaskDelay(pdMS_TO_TICKS(STORY_TONE_DURATION_MS));
            break;
        case NFC_STORY_KIND_WAIT:
            if (param > 0 && param < 10000) {
                vTaskDelay(pdMS_TO_TICKS((uint32_t)param));
            }
            break;
        case NFC_STORY_KIND_CLIP:
            if (param >= 0 && param < 256) {
                relay_clip((uint8_t)param);
            }
            break;
        default:
            ESP_LOGW(TAG, "Story: unknown kind %u — skipping", (unsigned)kind);
            break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* LLM request / reply                                                 */
/* ------------------------------------------------------------------ */

/* Build a small ASCII prompt hint from the tag's UID + label and unicast
 * it to the leader.  If we haven't seen a leader yet, broadcast it. */
static void send_story_request(const thinkpack_nfc_entry_t *entry)
{
    char hint[96];
    int n = snprintf(hint, sizeof(hint), "story:uid=");
    for (uint8_t i = 0; i < entry->uid_len && n > 0 && (size_t)n + 3 < sizeof(hint); i++) {
        n += snprintf(hint + n, sizeof(hint) - (size_t)n, "%02X", entry->uid[i]);
    }
    if (n > 0 && (size_t)n + 1 < sizeof(hint)) {
        n += snprintf(hint + n, sizeof(hint) - (size_t)n, " label=\"%s\" param=%u", entry->label,
                      (unsigned)entry->param);
    }
    if (n < 0) {
        n = 0;
    }
    size_t hint_len = (size_t)n;
    if (hint_len > sizeof(hint)) {
        hint_len = sizeof(hint);
    }

    uint8_t own_mac[6];
    fill_own_mac(own_mac);

    thinkpack_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.msg_type = (uint8_t)MSG_LLM_REQUEST;
    pkt.sequence_number = s_seq++;
    memcpy(pkt.src_mac, own_mac, 6);
    pkt.data_length =
        (uint8_t)(hint_len > THINKPACK_MAX_DATA_LEN ? THINKPACK_MAX_DATA_LEN : hint_len);
    memcpy(pkt.data, hint, pkt.data_length);
    thinkpack_finalize(&pkt);

    const uint8_t *dest = s_have_leader ? s_leader_mac : NULL;
    (void)thinkpack_mesh_send(dest, &pkt);
    ESP_LOGI(TAG, "STORY request sent (%u bytes, leader=%s)", (unsigned)pkt.data_length,
             s_have_leader ? "unicast" : "broadcast");
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void group_mode_init(thinkpack_nfc_registry_t *registry)
{
    s_registry = registry;
    hot_cold_reset(&s_hot_cold);
    s_seek_active = false;
    s_have_leader = false;
    s_story_pending = false;

    BaseType_t ok = xTaskCreate(seek_task, "seek_poll", 4096, NULL, 4, &s_seek_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "SEEK polling task creation failed");
    } else {
        ESP_LOGI(TAG, "group_mode initialised — seek task running at %d ms", SEEK_POLL_INTERVAL_MS);
    }
}

void group_mode_on_local_scan(const thinkpack_nfc_entry_t *entry)
{
    if (entry == NULL) {
        return;
    }
    switch ((thinkpack_nfc_behavior_t)entry->behavior) {
        case THINKPACK_NFC_BEHAVIOR_SEEK:
            ESP_LOGI(TAG, "SEEK armed — label='%s'", entry->label);
            hot_cold_reset(&s_hot_cold);
            s_seek_active = true;
            break;
        case THINKPACK_NFC_BEHAVIOR_STORY:
            ESP_LOGI(TAG, "STORY scan — label='%s' param=%u", entry->label, (unsigned)entry->param);
            memcpy(s_story_uid, entry->uid, entry->uid_len);
            s_story_uid_len = entry->uid_len;
            s_story_pending = true;
            send_story_request(entry);
            break;
        default:
            /* CHIME / COLOR / NONE: no group-mode action — standalone handles them. */
            break;
    }
}

void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event == NULL) {
        return;
    }

    switch (event->type) {
        case THINKPACK_EVENT_LEADER_ELECTED:
        case THINKPACK_EVENT_BECAME_FOLLOWER:
            memcpy(s_leader_mac, event->peer_mac, 6);
            s_have_leader = true;
            ESP_LOGI(TAG, "Leader known: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_LOST:
            s_have_leader = false;
            break;

        case THINKPACK_EVENT_PEER_DISCOVERED:
            ESP_LOGD(TAG, "Peer discovered: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED:
            if (event->original_msg_type == (uint8_t)MSG_LLM_RESPONSE && s_story_pending &&
                event->large_data != NULL && event->large_length > 0) {
                story_sequence_t seq;
                if (nfc_story_sounds_parse((const char *)event->large_data, event->large_length,
                                           &seq)) {
                    ESP_LOGI(TAG, "STORY reply parsed — %u steps", (unsigned)seq.count);
                    nfc_story_sounds_execute(&seq, story_step_dispatch, NULL);
                } else {
                    ESP_LOGW(TAG, "STORY reply rejected by parser (%zu bytes)",
                             event->large_length);
                }
                s_story_pending = false;
            }
            break;

        default:
            break;
    }
}
