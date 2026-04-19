/**
 * @file thinkpack_protocol.c
 * @brief ESP-NOW packet helpers for the ThinkPack modular toy mesh.
 */

#include "thinkpack_protocol.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Compile-time size assertions                                        */
/* ------------------------------------------------------------------ */

_Static_assert(sizeof(thinkpack_packet_t) <= 250,
               "thinkpack_packet_t exceeds ESP-NOW 250-byte frame limit");

_Static_assert(sizeof(thinkpack_beacon_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_beacon_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_election_bid_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_election_bid_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_leader_claim_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_leader_claim_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_capability_reply_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_capability_reply_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_sync_pulse_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_sync_pulse_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_command_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_command_data_t exceeds THINKPACK_MAX_DATA_LEN");
_Static_assert(sizeof(thinkpack_fragment_data_t) <= THINKPACK_MAX_DATA_LEN,
               "thinkpack_fragment_data_t exceeds THINKPACK_MAX_DATA_LEN");

/* ------------------------------------------------------------------ */
/* Checksum                                                            */
/* ------------------------------------------------------------------ */

uint8_t thinkpack_checksum(const uint8_t *data, size_t length)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < length; i++) {
        cs ^= data[i];
    }
    return cs;
}

bool thinkpack_verify_checksum(const thinkpack_packet_t *p)
{
    if (!p) {
        return false;
    }
    uint8_t computed = thinkpack_checksum((const uint8_t *)p, sizeof(*p) - 1);
    return computed == p->checksum;
}

/* ------------------------------------------------------------------ */
/* Finalize                                                            */
/* ------------------------------------------------------------------ */

void thinkpack_finalize(thinkpack_packet_t *p)
{
    if (!p) {
        return;
    }
    p->magic[0] = THINKPACK_MAGIC_0;
    p->magic[1] = THINKPACK_MAGIC_VERSION;
    p->checksum = thinkpack_checksum((const uint8_t *)p, sizeof(*p) - 1);
}

/* ------------------------------------------------------------------ */
/* Priority calculation                                                */
/* ------------------------------------------------------------------ */

uint32_t thinkpack_priority_for_capabilities(uint16_t capabilities, const uint8_t mac[6])
{
    uint32_t score = 0;

    if (capabilities & CAP_LLM) {
        score += PRIO_CAP_LLM;
    }
    if (capabilities & CAP_WIFI) {
        score += PRIO_CAP_WIFI;
    }
    if (capabilities & CAP_DISPLAY) {
        score += PRIO_CAP_DISPLAY;
    }
    if (capabilities & CAP_AUDIO_OUT) {
        score += PRIO_CAP_AUDIO_OUT;
    }
    if (capabilities & CAP_AUDIO_IN) {
        score += PRIO_CAP_AUDIO_IN;
    }

    uint16_t mac_tiebreaker = mac ? (uint16_t)((mac[4] << 8) | mac[5]) : 0u;
    return (score << 16) | mac_tiebreaker;
}

/* ------------------------------------------------------------------ */
/* Prepare helpers                                                     */
/* ------------------------------------------------------------------ */

void thinkpack_prepare_beacon(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                              const thinkpack_beacon_data_t *beacon)
{
    if (!p || !beacon) {
        return;
    }

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_BEACON;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = sizeof(thinkpack_beacon_data_t);
    memcpy(p->data, beacon, sizeof(thinkpack_beacon_data_t));
    thinkpack_finalize(p);
}

void thinkpack_prepare_election_bid(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                    uint32_t priority)
{
    if (!p) {
        return;
    }

    thinkpack_election_bid_data_t bid;
    memset(&bid, 0, sizeof(bid));
    bid.priority = priority;
    memcpy(bid.mac, src_mac, 6);

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_ELECTION_BID;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = sizeof(thinkpack_election_bid_data_t);
    memcpy(p->data, &bid, sizeof(thinkpack_election_bid_data_t));
    thinkpack_finalize(p);
}

void thinkpack_prepare_leader_claim(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                    uint32_t priority, uint8_t channel)
{
    if (!p) {
        return;
    }

    thinkpack_leader_claim_data_t claim;
    memset(&claim, 0, sizeof(claim));
    claim.priority = priority;
    memcpy(claim.mac, src_mac, 6);
    claim.channel = channel;

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_LEADER_CLAIM;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = sizeof(thinkpack_leader_claim_data_t);
    memcpy(p->data, &claim, sizeof(thinkpack_leader_claim_data_t));
    thinkpack_finalize(p);
}

void thinkpack_prepare_sync_pulse(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                  uint32_t timestamp_ms, uint8_t phase)
{
    if (!p) {
        return;
    }

    thinkpack_sync_pulse_data_t pulse;
    memset(&pulse, 0, sizeof(pulse));
    pulse.timestamp_ms = timestamp_ms;
    pulse.phase = phase;

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_SYNC_PULSE;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = sizeof(thinkpack_sync_pulse_data_t);
    memcpy(p->data, &pulse, sizeof(thinkpack_sync_pulse_data_t));
    thinkpack_finalize(p);
}

void thinkpack_prepare_fragment(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                const thinkpack_fragment_data_t *fragment)
{
    if (!p || !fragment) {
        return;
    }

    memset(p, 0, sizeof(*p));
    p->msg_type = MSG_FRAGMENT;
    p->sequence_number = seq;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = sizeof(thinkpack_fragment_data_t);
    memcpy(p->data, fragment, sizeof(thinkpack_fragment_data_t));
    thinkpack_finalize(p);
}
