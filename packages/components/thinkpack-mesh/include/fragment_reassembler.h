/**
 * @file fragment_reassembler.h
 * @brief Pure-C fragment reassembly cache for the ThinkPack mesh layer.
 *
 * This module is intentionally free of ESP-IDF dependencies so that it can
 * be compiled and tested on the host (Linux/macOS) with plain gcc.  The mesh
 * layer (espnow_mesh.c) calls these functions from its receive task.
 *
 * Concurrency: the caller must ensure that only one thread calls these
 * functions at a time (the receive task provides this guarantee).
 */

#ifndef FRAGMENT_REASSEMBLER_H
#define FRAGMENT_REASSEMBLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "thinkpack_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Reassembly cache                                                    */
/* ------------------------------------------------------------------ */

/** Number of concurrent in-progress large messages tracked. */
#define REASSEMBLY_CACHE_SIZE 4

/**
 * @brief State for one in-progress large message.
 *
 * A slot is "active" when total_fragments > 0.  Call reassembler_release()
 * to free it after processing a completed message.
 */
typedef struct {
    uint8_t src_mac[6];                        /**< Source MAC of the sender                   */
    uint8_t msg_id;                            /**< Unique large-message ID from the sender     */
    uint8_t total_fragments;                   /**< Expected fragment count (0 = slot unused)   */
    uint8_t original_msg_type;                 /**< Logical message type wrapped by fragments   */
    uint16_t received_mask;                    /**< Bit N set once fragment N has been absorbed */
    size_t reassembled_len;                    /**< Running byte count of assembled data        */
    uint32_t first_fragment_ms;                /**< Timestamp of the first fragment received    */
    uint8_t buffer[THINKPACK_MAX_REASSEMBLED]; /**< Assembled payload            */
} reassembly_slot_t;

/* ------------------------------------------------------------------ */
/* Return codes                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    REASSEMBLE_INCOMPLETE = 0, /**< More fragments still expected        */
    REASSEMBLE_COMPLETE,       /**< All fragments received; slot is ready */
    REASSEMBLE_ERROR,          /**< Protocol violation; slot discarded    */
} reassemble_result_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Zero-initialise the reassembly cache.
 *
 * @param cache  Array of REASSEMBLY_CACHE_SIZE slots.
 */
void reassembler_init(reassembly_slot_t *cache);

/**
 * @brief Free entries whose first_fragment_ms is older than stale_ms.
 *
 * @param cache     Array of REASSEMBLY_CACHE_SIZE slots.
 * @param now_ms    Current time in milliseconds.
 * @param stale_ms  Age threshold; slots older than this are pruned.
 */
void reassembler_prune(reassembly_slot_t *cache, uint32_t now_ms, uint32_t stale_ms);

/**
 * @brief Absorb one fragment into the reassembly cache.
 *
 * Looks up or allocates a slot keyed on (src_mac, msg_id).  If all
 * fragments for the message have now been received the function sets
 * *out_completed and returns REASSEMBLE_COMPLETE.  The caller must call
 * reassembler_release() on the completed slot after processing it.
 *
 * If the cache is full and a new (src_mac, msg_id) pair arrives the oldest
 * slot is overwritten (the new fragment is then placed into it).
 *
 * @param cache          Array of REASSEMBLY_CACHE_SIZE slots.
 * @param src_mac        6-byte source MAC of the sender.
 * @param fragment       Decoded fragment payload.
 * @param now_ms         Current time in milliseconds.
 * @param out_completed  Set to the completed slot on REASSEMBLE_COMPLETE;
 *                       unchanged otherwise.
 * @return               REASSEMBLE_INCOMPLETE / COMPLETE / ERROR.
 */
reassemble_result_t reassembler_absorb(reassembly_slot_t *cache, const uint8_t src_mac[6],
                                       const thinkpack_fragment_data_t *fragment, uint32_t now_ms,
                                       reassembly_slot_t **out_completed);

/**
 * @brief Mark a completed slot as free so it can be reused.
 *
 * @param slot  The slot returned via out_completed by reassembler_absorb().
 */
void reassembler_release(reassembly_slot_t *slot);

#ifdef __cplusplus
}
#endif

#endif /* FRAGMENT_REASSEMBLER_H */
