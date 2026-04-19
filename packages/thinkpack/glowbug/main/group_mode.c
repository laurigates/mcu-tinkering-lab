/**
 * @file group_mode.c
 * @brief Mesh event handler for glowbug group behaviour.
 *
 * Event handling:
 *  - THINKPACK_EVENT_SYNC_PULSE       → ANIM_SYNC_PULSE (phase-locked)
 *  - THINKPACK_EVENT_COMMAND_RECEIVED → command_executor_dispatch()
 *  - THINKPACK_EVENT_PEER_DISCOVERED  → 150 ms white hello-chime flash
 *
 * CMD_LED_PATTERN is registered with command_executor in group_mode_init;
 * the executor unpacks the envelope and invokes led_pattern_handler.
 *
 * Called from the receive task on Core 0; must not block.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "group_mode.h"

#include <string.h>

#include "animations.h"
#include "command_executor.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "led_ring.h"
#include "thinkpack_commands.h"
#include "thinkpack_protocol.h"

static const char *TAG = "group_mode";

/* ------------------------------------------------------------------ */
/* Command handlers                                                    */
/* ------------------------------------------------------------------ */

/** Handler for CMD_LED_PATTERN — registered with command_executor. */
static void led_pattern_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                                void *user_ctx)
{
    (void)command_id;
    (void)user_ctx;

    if (length < sizeof(cmd_led_pattern_payload_t)) {
        ESP_LOGW(TAG, "CMD_LED_PATTERN: payload too short (%u bytes)", length);
        return;
    }

    cmd_led_pattern_payload_t p;
    memcpy(&p, payload, sizeof(p));

    ESP_LOGI(TAG, "CMD_LED_PATTERN r=%u g=%u b=%u mode=%u", p.r, p.g, p.b, p.pattern);

    /* Validate mode byte (shared with anim_mode_t enum range) */
    if (p.pattern > (uint8_t)ANIM_MOOD_MIRROR) {
        ESP_LOGW(TAG, "CMD_LED_PATTERN: unknown mode %u, ignoring", p.pattern);
        return;
    }

    /* Derive hue from r channel (compatible with prior behaviour) */
    animations_set_mode((anim_mode_t)p.pattern, p.r);
}

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

void group_mode_init(void)
{
    esp_err_t err = command_executor_register(CMD_LED_PATTERN, led_pattern_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register CMD_LED_PATTERN handler: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Registered CMD_LED_PATTERN handler");
    }
}

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
                command_executor_dispatch(event->packet);
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
