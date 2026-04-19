/**
 * @file group_mode.h
 * @brief Mesh event handler for glowbug group (multi-box) behaviour.
 *
 * Register group_mode_on_event as the thinkpack_mesh event callback.
 * Handles SYNC_PULSE, COMMAND_RECEIVED (via command_executor), and
 * PEER_DISCOVERED events and translates them into animation directives.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef GROUP_MODE_H
#define GROUP_MODE_H

#include "espnow_mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise group-mode state and register command handlers.
 *
 * Must be called once before thinkpack_mesh_start().  Registers the
 * CMD_LED_PATTERN (0x10) handler with command_executor so that
 * incoming MSG_COMMAND packets are dispatched to animation helpers.
 */
void group_mode_init(void);

/**
 * @brief Mesh event callback — register with thinkpack_mesh_set_event_callback().
 *
 * Called from the receive task on Core 0.  Dispatches to animation helpers
 * without blocking.  Command packets are forwarded to command_executor.
 *
 * @param event     Event descriptor; valid only for the duration of this call.
 * @param user_ctx  Unused; pass NULL.
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MODE_H */
