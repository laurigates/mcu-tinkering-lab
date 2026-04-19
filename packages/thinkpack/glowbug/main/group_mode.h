/**
 * @file group_mode.h
 * @brief Mesh event handler for glowbug group (multi-box) behaviour.
 *
 * Register group_mode_on_event as the thinkpack_mesh event callback.
 * Handles SYNC_PULSE, COMMAND_RECEIVED, and PEER_DISCOVERED events and
 * translates them into animation directives.
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
 * @brief LED_PATTERN command ID (MSG_COMMAND payload command_id).
 *
 * Payload layout: {r, g, b, mode} — 4 bytes.
 *   r, g, b: target colour (full-scale; brightness cap applied by led_ring).
 *   mode:    anim_mode_t to switch to (ignored if out of range).
 */
#define CMD_LED_PATTERN 0x10u

/**
 * @brief Mesh event callback — register with thinkpack_mesh_set_event_callback().
 *
 * Called from the receive task on Core 0.  Dispatches to animation helpers
 * without blocking.
 *
 * @param event     Event descriptor; valid only for the duration of this call.
 * @param user_ctx  Unused; pass NULL.
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MODE_H */
