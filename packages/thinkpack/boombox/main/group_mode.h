/**
 * @file group_mode.h
 * @brief Mesh event handler for boombox group behaviour.
 *
 * Registered as the thinkpack_mesh event callback. Handles:
 *   SYNC_PULSE        — align beat index to leader phase
 *   PEER_DISCOVERED   — play 3-note hello chime (C5 E5 G5)
 *   COMMAND_RECEIVED  — 0x20 PLAY_MELODY: switch pattern for N beats
 *   BECAME_LEADER     — start sync-pulse broadcast timer
 *   BECAME_FOLLOWER   — stop sync-pulse broadcast timer
 */

#ifndef GROUP_MODE_H
#define GROUP_MODE_H

#include "espnow_mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise group mode state. Call once before thinkpack_mesh_start().
 */
void group_mode_init(void);

/**
 * @brief Mesh event callback — pass directly to thinkpack_mesh_set_event_callback().
 *
 * @param event     Mesh event descriptor.
 * @param user_ctx  Unused (NULL).
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

/**
 * @brief Service chime and temporary-pattern timeouts.
 *
 * Call every control-loop tick alongside standalone_mode_tick().
 *
 * @param now_ms  Current time in milliseconds.
 */
void group_mode_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MODE_H */
