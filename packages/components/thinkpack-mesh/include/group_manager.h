/**
 * @file group_manager.h
 * @brief Peer table and capability aggregation for the ThinkPack mesh.
 *
 * Maintains a static array of up to THINKPACK_MAX_PEERS entries. All
 * functions that mutate or traverse the table are protected by an internal
 * FreeRTOS mutex and are safe to call from any task.
 */

#ifndef GROUP_MANAGER_H
#define GROUP_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "thinkpack_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Peer descriptor                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief Complete description of a known peer node.
 */
typedef struct {
    uint8_t mac[6];                    /**< Peer WiFi-STA MAC address */
    uint8_t box_type;                  /**< thinkpack_box_type_t */
    uint16_t capabilities;             /**< CAP_* bitmask */
    uint32_t priority;                 /**< Capability score + MAC tiebreaker */
    char name[THINKPACK_BOX_NAME_LEN]; /**< Human-readable name (NUL-term) */
    uint32_t last_seen_ms;             /**< Timestamp of last received packet */
    int8_t rssi;                       /**< RSSI of last received packet (dBm) */
} thinkpack_peer_t;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the group manager.
 *
 * Clears the peer table and creates the internal mutex. Must be called
 * once before any other group_manager_* function.
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the mutex cannot be created.
 */
esp_err_t group_manager_init(void);

/* ------------------------------------------------------------------ */
/* Mutation                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Insert or update a peer record.
 *
 * If a record with the same MAC already exists it is updated in place.
 * If no record exists and the table is not full, a new entry is appended.
 * If the table is full and the peer is not already present the call is a
 * no-op and returns false.
 *
 * @param peer  Non-NULL descriptor to upsert.
 * @return true if the peer was newly added, false if updated or table full.
 */
bool group_manager_upsert(const thinkpack_peer_t *peer);

/**
 * @brief Callback invoked for each peer removed by group_manager_prune().
 *
 * Called while the peer table mutex is held — keep the callback short and
 * do not call any group_manager_* function from within it.
 *
 * @param peer      Descriptor of the removed peer (valid only during callback).
 * @param user_ctx  Opaque pointer supplied to group_manager_prune().
 */
typedef void (*group_manager_removed_cb_t)(const thinkpack_peer_t *peer, void *user_ctx);

/**
 * @brief Remove peers that have not been seen within @p stale_ms.
 *
 * Iterates the peer table and removes any entry where
 * (@p now_ms - last_seen_ms) > @p stale_ms. For each removed entry
 * @p cb is invoked (if non-NULL) before the slot is freed.
 *
 * @param now_ms    Current system time in milliseconds.
 * @param stale_ms  Age threshold in milliseconds.
 * @param cb        Optional removal callback.
 * @param user_ctx  Forwarded to @p cb.
 */
void group_manager_prune(uint32_t now_ms, uint32_t stale_ms, group_manager_removed_cb_t cb,
                         void *user_ctx);

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

/** @return Number of peers currently in the table. */
size_t group_manager_count(void);

/**
 * @brief Return the peer at the given index.
 *
 * The returned pointer is into the internal static table. It remains
 * valid until the next call to group_manager_upsert() or
 * group_manager_prune() — copy the data if you need to retain it.
 *
 * @param index  0-based index; must be < group_manager_count().
 * @return Pointer to the peer descriptor, or NULL if out of range.
 */
const thinkpack_peer_t *group_manager_get(size_t index);

/**
 * @brief Find a peer by its MAC address.
 *
 * @param mac  6-byte MAC to look up.
 * @return Pointer to the matching descriptor, or NULL if not found.
 */
const thinkpack_peer_t *group_manager_find(const uint8_t mac[6]);

/**
 * @brief OR the capability bitmasks of all known peers.
 *
 * @return Aggregate CAP_* bitmask, or 0 if the table is empty.
 */
uint16_t group_manager_aggregate_capabilities(void);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MANAGER_H */
