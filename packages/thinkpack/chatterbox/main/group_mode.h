/**
 * @file group_mode.h
 * @brief Mesh event handler for the Chatterbox echo-chamber group behaviour.
 *
 * When local touch captures a clip and the mesh has peers, the leader
 * invokes group_mode_broadcast_clip() to assemble an
 * audio_clip_broadcast_payload_t and send it out via the mesh.  Incoming
 * MSG_AUDIO_CLIP_BROADCAST packets trigger a pitch-shifted local playback
 * at the slot index corresponding to this node.
 */

#ifndef GROUP_MODE_H
#define GROUP_MODE_H

#include <stdbool.h>
#include <stdint.h>

#include "espnow_mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise group-mode state.
 *
 * Must be called once before thinkpack_mesh_start().  This currently only
 * clears module-local state; command_executor handlers are not registered
 * because echo-chamber traffic uses MSG_AUDIO_CLIP_BROADCAST directly rather
 * than MSG_COMMAND.
 */
void group_mode_init(void);

/**
 * @brief Mesh event callback — dispatch MSG_AUDIO_CLIP_BROADCAST.
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

/**
 * @brief Announce a just-captured clip to the mesh.
 *
 * Assembles the audio_clip_broadcast_payload_t via echo_chamber, stamps
 * a fresh clip id, and broadcasts the metadata packet.  The PCM itself is
 * streamed separately via thinkpack_mesh_send_large() when a receiver
 * requests it (not implemented here).
 *
 * @return true on success.
 */
bool group_mode_broadcast_clip(uint16_t sample_count);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MODE_H */
