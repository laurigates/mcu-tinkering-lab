/**
 * @file command_executor.c
 * @brief Follower-side command registry and dispatcher for the ThinkPack behavior layer.
 *
 * Maintains a static table of up to EXECUTOR_MAX_HANDLERS registered handlers.
 * command_executor_dispatch() unpacks the thinkpack_command_data_t envelope from
 * an incoming MSG_COMMAND packet and routes to the appropriate handler.
 */

#include "command_executor.h"

#include <string.h>

#include "esp_log.h"

/* ------------------------------------------------------------------ */
/* Module tag                                                          */
/* ------------------------------------------------------------------ */

static const char *TAG = "cmd_exec";

/* ------------------------------------------------------------------ */
/* Registry                                                            */
/* ------------------------------------------------------------------ */

#define EXECUTOR_MAX_HANDLERS 16

typedef struct {
    uint8_t command_id;
    command_handler_t handler;
    void *user_ctx;
    bool active;
} handler_entry_t;

static handler_entry_t s_registry[EXECUTOR_MAX_HANDLERS];

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t command_executor_register(uint8_t command_id, command_handler_t handler, void *user_ctx)
{
    int free_slot = -1;

    for (int i = 0; i < EXECUTOR_MAX_HANDLERS; i++) {
        if (!s_registry[i].active) {
            if (free_slot < 0) {
                free_slot = i;
            }
            continue;
        }
        if (s_registry[i].command_id == command_id) {
            ESP_LOGW(TAG, "command 0x%02x already registered", command_id);
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (free_slot < 0) {
        ESP_LOGW(TAG, "handler registry full (max %d)", EXECUTOR_MAX_HANDLERS);
        return ESP_ERR_NO_MEM;
    }

    s_registry[free_slot].command_id = command_id;
    s_registry[free_slot].handler = handler;
    s_registry[free_slot].user_ctx = user_ctx;
    s_registry[free_slot].active = true;
    ESP_LOGI(TAG, "registered handler for command 0x%02x at slot %d", command_id, free_slot);
    return ESP_OK;
}

void command_executor_dispatch(const thinkpack_packet_t *packet)
{
    if (!packet) {
        return;
    }

    if (packet->msg_type != MSG_COMMAND) {
        ESP_LOGW(TAG, "dispatch called with non-command packet (msg_type=0x%02x)",
                 packet->msg_type);
        return;
    }

    if (packet->data_length < sizeof(thinkpack_command_data_t)) {
        ESP_LOGW(TAG, "packet data_length %u too small for command envelope (%zu)",
                 packet->data_length, sizeof(thinkpack_command_data_t));
        return;
    }

    const thinkpack_command_data_t *cmd =
        (const thinkpack_command_data_t *)(const void *)packet->data;

    for (int i = 0; i < EXECUTOR_MAX_HANDLERS; i++) {
        if (s_registry[i].active && s_registry[i].command_id == cmd->command_id) {
            s_registry[i].handler(cmd->command_id, cmd->payload, cmd->length,
                                  s_registry[i].user_ctx);
            return;
        }
    }

    ESP_LOGW(TAG, "no handler for command 0x%02x — ignored", cmd->command_id);
}

void command_executor_reset(void)
{
    memset(s_registry, 0, sizeof(s_registry));
    ESP_LOGI(TAG, "handler registry cleared");
}
