/**
 * @file group_mode.c
 * @brief LLM-driven collective behaviour coordinator.
 *
 * Worker flow (Core 1):
 *   1. Dequeue a trigger string.
 *   2. Snapshot the peer table into a group_manifest_t.
 *   3. Build a prompt via prompt_builder_collective().
 *   4. Query the active AI backend (blocking HTTP call).
 *   5. Parse the JSON response via response_parser_parse().
 *   6. For each parsed command, build a packet and send via thinkpack_mesh_send().
 */

#include "group_mode.h"

#include <string.h>

#include "command_dispatcher.h"
#include "display_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "group_manager.h"
#include "prompt_builder.h"
#include "response_parser.h"
#include "thinkpack_ai.h"
#include "thinkpack_commands.h"
#include "thinkpack_protocol.h"

static const char *TAG = "group_mode";

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define TRIGGER_QUEUE_DEPTH 4
#define TRIGGER_MAX_LEN 64
#define PROMPT_BUF_SIZE 2048
#define WORKER_STACK_SIZE 8192
#define WORKER_PRIORITY 5

/* ------------------------------------------------------------------ */
/* Internal types                                                      */
/* ------------------------------------------------------------------ */

typedef char trigger_t[TRIGGER_MAX_LEN];

/* Context passed to the response callback. */
typedef struct {
    uint8_t src_mac[6];
    uint8_t seq;
} dispatch_ctx_t;

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

static QueueHandle_t s_trigger_queue = NULL;

/* ------------------------------------------------------------------ */
/* Response callback — called by response_parser_parse() per command   */
/* ------------------------------------------------------------------ */

static void on_command(uint8_t target_box_type, uint8_t command_id, const uint8_t *payload,
                       uint8_t payload_len, void *user_ctx)
{
    dispatch_ctx_t *ctx = (dispatch_ctx_t *)user_ctx;

    /* Resolve destination MAC: BOX_UNKNOWN → broadcast (NULL). */
    const uint8_t *dest_mac = NULL;
    if (target_box_type != (uint8_t)BOX_UNKNOWN) {
        size_t count = group_manager_count();
        for (size_t i = 0; i < count; i++) {
            const thinkpack_peer_t *peer = group_manager_get(i);
            if (peer && peer->box_type == target_box_type) {
                dest_mac = peer->mac;
                break;
            }
        }
        if (!dest_mac) {
            ESP_LOGW(TAG, "No peer of box_type %u found — broadcasting command", target_box_type);
        }
    }

    /* Build the command packet via the dispatcher helpers. */
    thinkpack_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    switch ((thinkpack_command_id_t)command_id) {
        case CMD_LED_PATTERN:
            if (payload_len >= sizeof(cmd_led_pattern_payload_t)) {
                const cmd_led_pattern_payload_t *p = (const cmd_led_pattern_payload_t *)payload;
                command_build_led_pattern(&pkt, ctx->seq++, ctx->src_mac, p);
            }
            break;
        case CMD_SET_MOOD:
            if (payload_len >= sizeof(cmd_set_mood_payload_t)) {
                const cmd_set_mood_payload_t *p = (const cmd_set_mood_payload_t *)payload;
                command_build_set_mood(&pkt, ctx->seq++, ctx->src_mac, p);
            }
            break;
        case CMD_PLAY_MELODY:
            if (payload_len >= sizeof(cmd_play_melody_payload_t)) {
                const cmd_play_melody_payload_t *p = (const cmd_play_melody_payload_t *)payload;
                command_build_play_melody(&pkt, ctx->seq++, ctx->src_mac, p);
            }
            break;
        case CMD_BUZZ:
            if (payload_len >= sizeof(cmd_buzz_payload_t)) {
                const cmd_buzz_payload_t *p = (const cmd_buzz_payload_t *)payload;
                command_build_buzz(&pkt, ctx->seq++, ctx->src_mac, p);
            }
            break;
        case CMD_PLAY_SEQUENCE:
            if (payload_len >= sizeof(cmd_play_sequence_payload_t)) {
                const cmd_play_sequence_payload_t *p = (const cmd_play_sequence_payload_t *)payload;
                command_build_play_sequence(&pkt, ctx->seq++, ctx->src_mac, p);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unhandled command_id 0x%02x — skipping", command_id);
            return;
    }

    esp_err_t err = thinkpack_mesh_send(dest_mac, &pkt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "thinkpack_mesh_send failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sent cmd 0x%02x to box_type %u (%s)", command_id, target_box_type,
                 dest_mac ? "unicast" : "broadcast");
    }
}

/* ------------------------------------------------------------------ */
/* Worker task                                                         */
/* ------------------------------------------------------------------ */

static void worker_task(void *pvParam)
{
    static char s_prompt[PROMPT_BUF_SIZE];
    trigger_t trigger;

    for (;;) {
        if (xQueueReceive(s_trigger_queue, &trigger, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI(TAG, "LLM trigger: '%s'", trigger);

        /* Snapshot peer table into manifest. */
        group_manifest_t manifest = {0};
        size_t peer_count = group_manager_count();
        manifest.peer_count = peer_count < 8 ? peer_count : 8;
        for (size_t i = 0; i < manifest.peer_count; i++) {
            const thinkpack_peer_t *peer = group_manager_get(i);
            if (!peer) {
                break;
            }
            manifest.peers[i].box_type = peer->box_type;
            manifest.peers[i].capabilities = peer->capabilities;
            strncpy(manifest.peers[i].name, peer->name, sizeof(manifest.peers[i].name) - 1);
        }

        if (manifest.peer_count == 0) {
            ESP_LOGI(TAG, "No peers — skipping LLM query");
            continue;
        }

        /* Build prompt. */
        int prompt_len = prompt_builder_collective(trigger, &manifest, s_prompt, sizeof(s_prompt));
        if (prompt_len <= 0) {
            ESP_LOGW(TAG, "prompt_builder_collective returned %d — skipping", prompt_len);
            continue;
        }

        /* Query AI backend. */
        const thinkpack_ai_backend_t *ai = thinkpack_ai_get_current();
        ai_response_t response = {0};
        uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

        esp_err_t ret = ai->query_text(s_prompt, &response);
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - t0;

        if (ret != ESP_OK || !response.response_text) {
            ESP_LOGW(TAG, "AI query failed (%s)", esp_err_to_name(ret));
            ai->free_response(&response);
            continue;
        }

        ESP_LOGI(TAG, "AI response in %" PRIu32 " ms (%zu bytes)", elapsed,
                 response.response_length);
        display_manager_set_last_llm_time_ms(elapsed);

        /* Parse and dispatch. */
        dispatch_ctx_t dispatch_ctx = {0};
        thinkpack_mesh_get_mac(dispatch_ctx.src_mac);
        dispatch_ctx.seq = 0;

        ret = response_parser_parse(response.response_text, response.response_length, on_command,
                                    &dispatch_ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "response_parser_parse failed: %s", esp_err_to_name(ret));
        }

        ai->free_response(&response);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t group_mode_init(void)
{
    s_trigger_queue = xQueueCreate(TRIGGER_QUEUE_DEPTH, sizeof(trigger_t));
    if (!s_trigger_queue) {
        ESP_LOGE(TAG, "Failed to create trigger queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(worker_task, "group_worker", WORKER_STACK_SIZE, NULL,
                                            WORKER_PRIORITY, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        vQueueDelete(s_trigger_queue);
        s_trigger_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Group mode initialised (queue depth %d, worker on Core 1)", TRIGGER_QUEUE_DEPTH);
    return ESP_OK;
}

void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    if (!event) {
        return;
    }

    switch (event->type) {
        case THINKPACK_EVENT_PEER_DISCOVERED:
            ESP_LOGI(TAG, "New peer discovered " MACSTR " — triggering LLM",
                     MAC2STR(event->peer_mac));
            group_mode_trigger_llm("new peer joined");
            break;

        case THINKPACK_EVENT_BECAME_LEADER:
            ESP_LOGI(TAG, "This node became mesh leader");
            break;

        case THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED:
            if (event->original_msg_type == (uint8_t)MSG_LLM_RESPONSE) {
                ESP_LOGI(TAG, "Large MSG_LLM_RESPONSE received (%zu bytes) — logged for relay",
                         event->large_length);
            }
            break;

        default:
            break;
    }
}

void group_mode_trigger_llm(const char *trigger)
{
    if (!trigger || !s_trigger_queue) {
        return;
    }

    trigger_t buf = {0};
    strncpy(buf, trigger, sizeof(buf) - 1);

    if (xQueueSend(s_trigger_queue, &buf, pdMS_TO_TICKS(0)) != pdTRUE) {
        ESP_LOGW(TAG, "Trigger queue full — dropping '%s'", trigger);
    }
}
