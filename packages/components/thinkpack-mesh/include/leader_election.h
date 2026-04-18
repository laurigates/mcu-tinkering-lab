/**
 * @file leader_election.h
 * @brief Bully-algorithm leader election for the ThinkPack mesh.
 *
 * The module is driven purely by external calls — there is no internal task.
 * The mesh layer must call leader_election_tick() periodically (e.g. every
 * 100 ms from the beacon task) to advance timers.
 *
 * Priority is a uint32_t encoded by thinkpack_priority_for_capabilities():
 *   bits 31-16: capability score
 *   bits 15-0:  MAC tiebreaker
 * Higher value wins.
 */

#ifndef LEADER_ELECTION_H
#define LEADER_ELECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

/** Current leadership state of this node. */
typedef enum {
    LEADER_STATE_UNKNOWN,  /**< No election has completed yet */
    LEADER_STATE_ELECTING, /**< An election is in progress */
    LEADER_STATE_FOLLOWER, /**< Following a known leader */
    LEADER_STATE_LEADER,   /**< This node is the current leader */
} leader_state_t;

/**
 * @brief Callback invoked when the leadership state changes.
 *
 * Called from the context that drives leader_election_tick() (beacon task,
 * Core 0). Must not block.
 *
 * @param state            New state for this node.
 * @param leader_mac       MAC of the current leader (own MAC if LEADER).
 * @param leader_priority  Priority value of the current leader.
 * @param user_ctx         Opaque pointer supplied at init time.
 */
typedef void (*leader_change_cb_t)(leader_state_t state, const uint8_t leader_mac[6],
                                   uint32_t leader_priority, void *user_ctx);

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the leader-election module.
 *
 * Must be called once before any other leader_election_* function.
 *
 * @param own_priority  This node's pre-computed election priority.
 * @param own_mac       This node's 6-byte WiFi-STA MAC address.
 * @param cb            State-change callback; may be NULL.
 * @param user_ctx      Forwarded to every callback invocation.
 * @return ESP_OK always (parameter validation via assert).
 */
esp_err_t leader_election_init(uint32_t own_priority, const uint8_t own_mac[6],
                               leader_change_cb_t cb, void *user_ctx);

/* ------------------------------------------------------------------ */
/* Actions                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Broadcast an election bid and enter LEADER_STATE_ELECTING.
 *
 * The actual ESP-NOW send is performed by the mesh layer via the send
 * callback it provided; this function sets state and records the election
 * start time so leader_election_tick() can resolve the election after
 * 2000 ms.
 *
 * @return ESP_OK on success.
 */
esp_err_t leader_election_trigger(void);

/* ------------------------------------------------------------------ */
/* Observation feed-in                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Record an election bid observed from a peer.
 *
 * Updates the highest-seen bid during an election. If this node is the
 * current leader and a higher-priority bid arrives, it immediately
 * concedes and re-enters LEADER_STATE_ELECTING.
 *
 * @param mac       6-byte source MAC of the bidding node.
 * @param priority  Priority value carried in the bid packet.
 */
void leader_election_observe_bid(const uint8_t mac[6], uint32_t priority);

/**
 * @brief Record a leader claim observed from a peer.
 *
 * Accepts the claim if the claimer outranks the current known leader.
 * Updates the last-heard-from-leader timestamp whenever the current
 * leader re-asserts.
 *
 * @param mac       6-byte MAC of the claiming node.
 * @param priority  Priority value carried in the claim packet.
 */
void leader_election_observe_claim(const uint8_t mac[6], uint32_t priority);

/* ------------------------------------------------------------------ */
/* Periodic tick                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Advance election and leader-loss timers.
 *
 * Must be called regularly (recommended: every 100 ms) from the beacon
 * task. Resolves completed elections and detects leader timeouts.
 *
 * @param now_ms  Current system time in milliseconds (e.g. from esp_timer
 *                or xTaskGetTickCount converted to ms).
 */
void leader_election_tick(uint32_t now_ms);

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

/** @return Current leadership state of this node. */
leader_state_t leader_election_get_state(void);

/** @return true if this node is currently the leader. */
bool leader_election_is_leader(void);

/**
 * @brief Retrieve the current leader's identity.
 *
 * @param out_mac       Receives the 6-byte leader MAC (zeroed if unknown).
 * @param out_priority  Receives the leader's priority (0 if unknown); may be NULL.
 */
void leader_election_get_leader(uint8_t out_mac[6], uint32_t *out_priority);

#ifdef __cplusplus
}
#endif

#endif /* LEADER_ELECTION_H */
