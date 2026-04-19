/**
 * @file fragment_reassembler.c
 * @brief Pure-C fragment reassembly cache — no ESP-IDF dependencies.
 *
 * Intentionally free of ESP-IDF headers so the file compiles under plain gcc
 * for host-based unit tests.  Logging about evictions and mismatches is
 * deliberately omitted here; callers (espnow_mesh.c) can log using whatever
 * logging facility is available in their build environment.
 */

#include "fragment_reassembler.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/** Return true if two 6-byte MACs are identical. */
static bool mac_equal(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

/** Return the index of the oldest active slot, or -1 if all slots are free. */
static int oldest_slot_index(const reassembly_slot_t *cache)
{
    int idx = -1;
    uint32_t oldest_ms = UINT32_MAX;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].first_fragment_ms <= oldest_ms) {
            oldest_ms = cache[i].first_fragment_ms;
            idx = i;
        }
    }
    return idx;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void reassembler_init(reassembly_slot_t *cache)
{
    memset(cache, 0, sizeof(reassembly_slot_t) * REASSEMBLY_CACHE_SIZE);
}

void reassembler_prune(reassembly_slot_t *cache, uint32_t now_ms, uint32_t stale_ms)
{
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments == 0) {
            continue;
        }
        if ((now_ms - cache[i].first_fragment_ms) >= stale_ms) {
            memset(&cache[i], 0, sizeof(reassembly_slot_t));
        }
    }
}

reassemble_result_t reassembler_absorb(reassembly_slot_t *cache, const uint8_t src_mac[6],
                                       const thinkpack_fragment_data_t *fragment, uint32_t now_ms,
                                       reassembly_slot_t **out_completed)
{
    if (!cache || !src_mac || !fragment || !out_completed) {
        return REASSEMBLE_ERROR;
    }

    /* Validate fragment fields. */
    if (fragment->total_fragments == 0 || fragment->total_fragments > THINKPACK_MAX_FRAGMENTS) {
        return REASSEMBLE_ERROR;
    }
    if (fragment->fragment_index >= fragment->total_fragments) {
        return REASSEMBLE_ERROR;
    }
    if (fragment->data_length > THINKPACK_MAX_FRAGMENT_DATA) {
        return REASSEMBLE_ERROR;
    }

    /* Look for an existing slot matching (src_mac, msg_id). */
    reassembly_slot_t *slot = NULL;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == fragment->msg_id &&
            mac_equal(cache[i].src_mac, src_mac)) {
            slot = &cache[i];
            break;
        }
    }

    if (!slot) {
        /* Try to find a free slot first. */
        for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
            if (cache[i].total_fragments == 0) {
                slot = &cache[i];
                break;
            }
        }

        /* Cache full — evict the oldest entry (caller may log this). */
        if (!slot) {
            int idx = oldest_slot_index(cache);
            if (idx < 0) {
                return REASSEMBLE_ERROR;
            }
            slot = &cache[idx];
        }

        /* Initialise the slot for this new message. */
        memset(slot, 0, sizeof(*slot));
        memcpy(slot->src_mac, src_mac, 6);
        slot->msg_id = fragment->msg_id;
        slot->total_fragments = fragment->total_fragments;
        slot->original_msg_type = fragment->original_msg_type;
        slot->first_fragment_ms = now_ms;
    }

    /* Consistency check: total_fragments must not change mid-stream. */
    if (slot->total_fragments != fragment->total_fragments) {
        memset(slot, 0, sizeof(*slot));
        return REASSEMBLE_ERROR;
    }

    /* Idempotency: ignore duplicate fragments. */
    uint16_t bit = (uint16_t)(1u << fragment->fragment_index);
    if (slot->received_mask & bit) {
        return REASSEMBLE_INCOMPLETE;
    }

    /* Copy this fragment's data into the correct position in the buffer. */
    size_t offset = (size_t)fragment->fragment_index * THINKPACK_MAX_FRAGMENT_DATA;
    memcpy(slot->buffer + offset, fragment->data, fragment->data_length);
    slot->reassembled_len += fragment->data_length;
    slot->received_mask |= bit;

    /* Check if all fragments have arrived. */
    uint16_t full_mask = (uint16_t)((1u << fragment->total_fragments) - 1u);
    if (slot->received_mask == full_mask) {
        *out_completed = slot;
        return REASSEMBLE_COMPLETE;
    }

    return REASSEMBLE_INCOMPLETE;
}

void reassembler_release(reassembly_slot_t *slot)
{
    if (slot) {
        memset(slot, 0, sizeof(*slot));
    }
}
