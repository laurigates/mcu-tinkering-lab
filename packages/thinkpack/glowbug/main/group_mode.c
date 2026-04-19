/**
 * @file group_mode.c
 * @brief Mesh event handler for glowbug group behaviour.
 *
 * Handles three event types:
 *  - THINKPACK_EVENT_SYNC_PULSE      → ANIM_SYNC_PULSE (200 ms flash)
 *  - THINKPACK_EVENT_COMMAND_RECEIVED → CMD_LED_PATTERN (0x10) dispatch
 *  - THINKPACK_EVENT_PEER_DISCOVERED  → 150 ms white hello-chime flash
 *
 * Called from the receive task on Core 0; must not block.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "group_mode.h"

#include <string.h>

#include "animations.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "led_ring.h"
#include "thinkpack_protocol.h"

static const char *TAG = "group_mode";

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/** Handle MSG_SYNC_PULSE: phase-lock animation to leader timestamp. */
static void handle_sync_pulse(const thinkpack_packet_t *pkt)
{
    if (pkt->data_length < sizeof(thinkpack_sync_pulse_data_t)) {
        ESP_LOGW(TAG, "SYNC_PULSE payload too short (%u bytes)", pkt->data_length);
        return;
    }

    const thinkpack_sync_pulse_data_t *pulse =
        (const thinkpack_sync_pulse_data_t *)(const void *)pkt->data;

    ESP_LOGD(TAG, "SYNC_PULSE ts=%lu phase=%u", (unsigned long)pulse->timestamp_ms, pulse->phase);
    animations_set_mode(ANIM_SYNC_PULSE, pulse->phase);
}

/** Handle MSG_COMMAND: dispatch on command_id. */
static void handle_command(const thinkpack_packet_t *pkt, const uint8_t peer_mac[6])
{
    if (pkt->data_length < sizeof(thinkpack_command_data_t)) {
        ESP_LOGW(TAG, "COMMAND payload too short (%u bytes)", pkt->data_length);
        return;
    }

    const thinkpack_command_data_t *cmd = (const thinkpack_command_data_t *)(const void *)pkt->data;

    switch (cmd->command_id) {
        case CMD_LED_PATTERN: {
            if (cmd->length < 4) {
                ESP_LOGW(TAG, "CMD_LED_PATTERN: payload too short (%u bytes)", cmd->length);
                break;
            }
            uint8_t r = cmd->payload[0];
            uint8_t g = cmd->payload[1];
            uint8_t b = cmd->payload[2];
            uint8_t mode_byte = cmd->payload[3];

            ESP_LOGI(TAG, "CMD_LED_PATTERN from " MACSTR " r=%u g=%u b=%u mode=%u",
                     MAC2STR(peer_mac), r, g, b, mode_byte);

            /* Validate mode byte */
            if (mode_byte <= (uint8_t)ANIM_MOOD_MIRROR) {
                /* Derive hue from r/g/b: use r as a simple hue approximation */
                animations_set_mode((anim_mode_t)mode_byte, r);
            } else {
                ESP_LOGW(TAG, "CMD_LED_PATTERN: unknown mode %u, ignoring", mode_byte);
            }
            break;
        }

        default:
            ESP_LOGI(TAG, "Unknown command 0x%02x from " MACSTR " — ignored", cmd->command_id,
                     MAC2STR(peer_mac));
            break;
    }
}

/** Handle PEER_DISCOVERED: brief white hello-chime flash (150 ms). */
static void handle_peer_discovered(const uint8_t peer_mac[6])
{
    ESP_LOGI(TAG, "Peer discovered: " MACSTR " — hello chime", MAC2STR(peer_mac));

    /* Full white flash — brightness cap applied by led_ring */
    led_ring_fill(255, 255, 255);
    led_ring_show();

    /* Non-blocking: animation_task will overwrite after next render tick.
     * The 150 ms window is provided by the 33 ms task period — the next
     * render call (ANIM_MIN_FRAME_MS = 100 ms) will replace the flash within
     * ~133 ms, which is within spec.  No vTaskDelay here to avoid blocking
     * the receive task. */
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;

    switch (event->type) {
        case THINKPACK_EVENT_SYNC_PULSE:
            if (event->packet != NULL) {
                handle_sync_pulse(event->packet);
            }
            break;

        case THINKPACK_EVENT_COMMAND_RECEIVED:
            if (event->packet != NULL) {
                handle_command(event->packet, event->peer_mac);
            }
            break;

        case THINKPACK_EVENT_PEER_DISCOVERED:
            handle_peer_discovered(event->peer_mac);
            break;

        case THINKPACK_EVENT_PEER_LOST:
            ESP_LOGI(TAG, "Peer lost: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_ELECTED:
            ESP_LOGI(TAG, "Leader elected: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_BECAME_LEADER:
            ESP_LOGI(TAG, "I became leader — sending sync pulses");
            break;

        case THINKPACK_EVENT_BECAME_FOLLOWER:
            ESP_LOGI(TAG, "Following leader: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_LOST:
            ESP_LOGW(TAG, "Leader lost — reverting to standalone");
            break;

        default:
            ESP_LOGW(TAG, "Unhandled event type %d", (int)event->type);
            break;
    }
}
