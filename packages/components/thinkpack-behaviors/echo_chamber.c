/**
 * @file echo_chamber.c
 * @brief Echo Chamber behaviour dispatch — builds
 *        MSG_AUDIO_CLIP_BROADCAST packets.
 *
 * Pure logic — no ESP-IDF or hardware dependencies; host-testable.
 */

#include "echo_chamber.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Compile-time payload size assertion                                 */
/* ------------------------------------------------------------------ */

_Static_assert(sizeof(audio_clip_broadcast_payload_t) <= THINKPACK_MAX_DATA_LEN,
               "audio_clip_broadcast_payload_t exceeds packet data limit");

/* ------------------------------------------------------------------ */
/* Pitch-shift policy                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Pitch step applied to each successive slot pair.
 *
 * Slot 0 stays at 0 (reference).  Odd slots go down in -2 steps; even slots
 * (>0) go up in +2 steps.  Both directions saturate at the protocol clamps
 * defined in thinkpack_audio.h (±12 semitones).
 */
#define ECHO_CHAMBER_PITCH_STEP 2
#define ECHO_CHAMBER_PITCH_MIN (-12)
#define ECHO_CHAMBER_PITCH_MAX (+12)

static int8_t clamp_semitones(int value)
{
    if (value < ECHO_CHAMBER_PITCH_MIN) {
        return (int8_t)ECHO_CHAMBER_PITCH_MIN;
    }
    if (value > ECHO_CHAMBER_PITCH_MAX) {
        return (int8_t)ECHO_CHAMBER_PITCH_MAX;
    }
    return (int8_t)value;
}

static int8_t slot_semitone(uint8_t slot)
{
    if (slot == 0) {
        return 0;
    }
    if ((slot & 1u) != 0u) {
        /* Odd slots: 1 → -2, 3 → -4, 5 → -6 ... */
        int step = (int)((slot + 1u) / 2u);
        return clamp_semitones(-(ECHO_CHAMBER_PITCH_STEP * step));
    }
    /* Even non-zero slots: 2 → +2, 4 → +4, 6 → +6 ... */
    int step = (int)(slot / 2u);
    return clamp_semitones(ECHO_CHAMBER_PITCH_STEP * step);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void echo_chamber_compute_pitch_table(uint8_t peer_count, int8_t *table, uint8_t len)
{
    if (table == NULL || len == 0u) {
        return;
    }

    uint8_t active = peer_count;
    if (active > len) {
        active = len;
    }

    for (uint8_t i = 0; i < active; i++) {
        table[i] = slot_semitone(i);
    }
    for (uint8_t i = active; i < len; i++) {
        table[i] = 0;
    }
}

void echo_chamber_build_payload(audio_clip_broadcast_payload_t *out, uint8_t clip_id,
                                uint16_t sample_count, uint8_t peer_count)
{
    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->clip_id = clip_id;

    /* Clamp sample count to the protocol maximum defined in thinkpack_audio.h
     * (32000 samples == 2 s @ 16 kHz).  We hardcode the ceiling here to avoid
     * a compile-time dependency on the audio component from the behaviour
     * layer. */
    const uint16_t max_samples = 32000u;
    out->sample_count = sample_count > max_samples ? max_samples : sample_count;

    echo_chamber_compute_pitch_table(peer_count, out->per_peer_semitone_shift,
                                     (uint8_t)sizeof(out->per_peer_semitone_shift));

    out->flags = 0u;
}

void echo_chamber_build_packet(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                               uint8_t clip_id, uint16_t sample_count, uint8_t peer_count)
{
    if (p == NULL) {
        return;
    }

    audio_clip_broadcast_payload_t payload;
    echo_chamber_build_payload(&payload, clip_id, sample_count, peer_count);

    memset(p, 0, sizeof(*p));
    p->msg_type = (uint8_t)MSG_AUDIO_CLIP_BROADCAST;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = (uint8_t)sizeof(payload);
    memcpy(p->data, &payload, sizeof(payload));
    thinkpack_finalize(p);
}
