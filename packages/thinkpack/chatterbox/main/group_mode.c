/**
 * @file group_mode.c
 * @brief Echo Chamber group behaviour on incoming + outgoing broadcasts.
 *
 * Broadcasts are sent via thinkpack_mesh_send_large() with
 * MSG_AUDIO_CLIP_BROADCAST as the logical type.  The mesh wraps the payload
 * in MSG_FRAGMENT packets (since our payload is small, this is a single
 * fragment in practice) and fires THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED on
 * receipt with @p original_msg_type == MSG_AUDIO_CLIP_BROADCAST.  This is
 * the agreed transport for new message types until a follow-up PR extends
 * the mesh event dispatcher with a generic "packet received" hook.
 */

#include "group_mode.h"

#include <string.h>

#include "audio_engine.h"
#include "echo_chamber.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "thinkpack_protocol.h"

static const char *TAG = "chat_group";

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static uint8_t s_next_clip_id;
static uint8_t s_local_slot; /**< Cached slot index (leader = 0). */

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void group_mode_init(void)
{
    s_next_clip_id = 0;
    s_local_slot = 0;
}

bool group_mode_broadcast_clip(uint16_t sample_count)
{
    size_t peer_count = thinkpack_mesh_peer_count();

    audio_clip_broadcast_payload_t payload;
    echo_chamber_build_payload(&payload, s_next_clip_id, sample_count, (uint8_t)peer_count);

    esp_err_t err = thinkpack_mesh_send_large(NULL, (uint8_t)MSG_AUDIO_CLIP_BROADCAST,
                                              (const uint8_t *)&payload, sizeof(payload));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "broadcast failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "broadcast clip_id=%u samples=%u peers=%u", s_next_clip_id, sample_count,
             (unsigned)peer_count);
    s_next_clip_id++;
    return true;
}

static void handle_clip_broadcast(const uint8_t *data, size_t length)
{
    if (length < sizeof(audio_clip_broadcast_payload_t)) {
        ESP_LOGW(TAG, "broadcast payload too small (%u bytes)", (unsigned)length);
        return;
    }
    audio_clip_broadcast_payload_t payload;
    memcpy(&payload, data, sizeof(payload));

    uint8_t slot = s_local_slot;
    if (slot >= (uint8_t)sizeof(payload.per_peer_semitone_shift)) {
        slot = 0;
    }
    int8_t shift = payload.per_peer_semitone_shift[slot];

    ESP_LOGI(TAG, "play clip_id=%u slot=%u shift=%d samples=%u", payload.clip_id, slot, shift,
             payload.sample_count);
    /* The PCM itself is streamed separately via a follow-up PR; for now we
     * pitch-shift and replay the locally captured clip (if any) as a
     * visible/audible stand-in. */
    audio_engine_playback_with_pitch_shift(shift);
}

void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event == NULL) {
        return;
    }

    switch (event->type) {
        case THINKPACK_EVENT_PEER_DISCOVERED:
            ESP_LOGI(TAG, "peer discovered: " MACSTR, MAC2STR(event->peer_mac));
            break;
        case THINKPACK_EVENT_PEER_LOST:
            ESP_LOGI(TAG, "peer lost: " MACSTR, MAC2STR(event->peer_mac));
            break;
        case THINKPACK_EVENT_BECAME_LEADER:
            s_local_slot = 0;
            ESP_LOGI(TAG, "became leader — slot 0");
            break;
        case THINKPACK_EVENT_BECAME_FOLLOWER:
            /* Leaders currently don't broadcast slot assignments — followers
             * default to slot 1.  A later PR can thread slot info through
             * MSG_LEADER_CLAIM. */
            s_local_slot = 1;
            ESP_LOGI(TAG, "became follower — slot %u", s_local_slot);
            break;
        case THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED:
            if (event->original_msg_type == (uint8_t)MSG_AUDIO_CLIP_BROADCAST) {
                handle_clip_broadcast(event->large_data, event->large_length);
            }
            break;
        default:
            break;
    }
}
