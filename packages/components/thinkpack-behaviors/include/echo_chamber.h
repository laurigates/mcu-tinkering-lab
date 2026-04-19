/**
 * @file echo_chamber.h
 * @brief Pure-logic Echo Chamber behaviour dispatch for Chatterbox.
 *
 * Composes the per-peer pitch-shift policy and builds a
 * MSG_AUDIO_CLIP_BROADCAST packet carrying an
 * audio_clip_broadcast_payload_t.  Transmission over the mesh is the
 * caller's responsibility — this module only builds the payload and
 * packet.  No PCM data is touched here; the actual clip is streamed via
 * thinkpack_mesh_send_large() separately.
 *
 * The pitch-shift policy is intentionally small and documented inline:
 *   slot 0       →  0 semitones (leader / local playback reference)
 *   odd slots    → -2, -4, -6, … (clamped to PITCH_MIN)
 *   even slots   → +2, +4, +6, … (clamped to PITCH_MAX)
 *
 * This mirrors the "answering call" toy behaviour: each box chirps back
 * the same clip at a different pitch, giving the impression of an
 * echo-chamber conversation between the boxes.
 */

#ifndef ECHO_CHAMBER_H
#define ECHO_CHAMBER_H

#include <stdint.h>

#include "thinkpack_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute the per-peer pitch-shift table for an echo-chamber broadcast.
 *
 * Fills @p table[0 .. len-1] with signed semitone offsets according to the
 * policy documented above.  Slots beyond @p peer_count are set to 0 to keep
 * the payload deterministic.
 *
 * @param peer_count  Number of peers currently in the mesh (>= 0).  Values
 *                    greater than @p len are clamped to @p len.
 * @param table       Output buffer; must be non-NULL if @p len > 0.
 * @param len         Capacity of @p table in entries.
 */
void echo_chamber_compute_pitch_table(uint8_t peer_count, int8_t *table, uint8_t len);

/**
 * @brief Populate an audio_clip_broadcast_payload_t for the echo-chamber.
 *
 * @param out          Payload to fill.  Must not be NULL.
 * @param clip_id      Monotonically increasing (wraps mod 256) per broadcast.
 * @param sample_count PCM sample count (clamped to
 *                     THINKPACK_AUDIO_CLIP_MAX_SAMPLES).
 * @param peer_count   Number of peers currently in the mesh.
 */
void echo_chamber_build_payload(audio_clip_broadcast_payload_t *out, uint8_t clip_id,
                                uint16_t sample_count, uint8_t peer_count);

/**
 * @brief Build the full MSG_AUDIO_CLIP_BROADCAST packet ready to send.
 *
 * Stamps magic bytes and checksum via thinkpack_finalize().
 *
 * @param p            Output packet.  Must not be NULL.
 * @param seq          Sequence number.
 * @param src_mac      Sender MAC.
 * @param clip_id      Broadcast id.
 * @param sample_count PCM sample count.
 * @param peer_count   Current peer count.
 */
void echo_chamber_build_packet(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               uint8_t clip_id, uint16_t sample_count, uint8_t peer_count);

#ifdef __cplusplus
}
#endif

#endif /* ECHO_CHAMBER_H */
