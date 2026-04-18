/**
 * @file leader_election.c
 * @brief Bully-algorithm leader election for the ThinkPack mesh.
 *
 * State machine:
 *
 *   UNKNOWN   ──trigger()──────────────────────────────► ELECTING
 *   ELECTING  ──2000 ms window elapses, own wins────────► LEADER
 *   ELECTING  ──2000 ms window elapses, peer wins───────► FOLLOWER
 *   FOLLOWER  ──observe_claim() from better peer────────► FOLLOWER (updated)
 *   FOLLOWER  ──5000 ms silence from leader─────────────► UNKNOWN → trigger() → ELECTING
 *   LEADER    ──observe_bid() with higher priority───────► ELECTING (concession)
 *
 * The mesh layer is responsible for all ESP-NOW sends. When this module
 * transitions into ELECTING or LEADER it fires leader_change_cb_t; the
 * mesh layer listens for those transitions and broadcasts the appropriate
 * packet (election bid or leader claim).
 */

#include "leader_election.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "election";

/* ------------------------------------------------------------------ */
/* Timing constants                                                    */
/* ------------------------------------------------------------------ */

/** Duration of a single election window in milliseconds. */
#define ELECTION_WINDOW_MS 2000u

/** Milliseconds of silence from the leader before declaring it lost. */
#define LEADER_TIMEOUT_MS 5000u

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

static uint32_t s_own_priority = 0;
static uint8_t s_own_mac[6] = {0};

static leader_state_t s_state = LEADER_STATE_UNKNOWN;

/** Highest-priority bid observed during the current election window. */
static uint8_t s_best_bid_mac[6] = {0};
static uint32_t s_best_bid_priority = 0;

/** Currently acknowledged leader. */
static uint8_t s_leader_mac[6] = {0};
static uint32_t s_leader_priority = 0;

/** Millisecond timestamp when the current election window opened. */
static uint32_t s_window_start_ms = 0;

/** Millisecond timestamp of the most recent leader heartbeat (beacon / claim). */
static uint32_t s_last_leader_heard_ms = 0;

/** Most recent now_ms seen by leader_election_tick(); used by observe_claim(). */
static uint32_t s_now_ms = 0;

static leader_change_cb_t s_cb = NULL;
static void *s_user_ctx = NULL;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static void fire_cb(leader_state_t state, const uint8_t leader_mac[6], uint32_t leader_priority)
{
    if (s_cb) {
        s_cb(state, leader_mac, leader_priority, s_user_ctx);
    }
}

/**
 * Enter ELECTING state, seeding the best-bid table with our own values
 * so the resolution logic can treat ourselves as a candidate uniformly.
 */
static void enter_electing(uint32_t now_ms)
{
    s_state = LEADER_STATE_ELECTING;
    s_window_start_ms = now_ms;
    s_best_bid_priority = s_own_priority;
    memcpy(s_best_bid_mac, s_own_mac, 6);
    ESP_LOGI(TAG, "Election window opened at %" PRIu32 " ms (own priority %" PRIu32 ")", now_ms,
             s_own_priority);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

esp_err_t leader_election_init(uint32_t own_priority, const uint8_t own_mac[6],
                               leader_change_cb_t cb, void *user_ctx)
{
    s_own_priority = own_priority;
    memcpy(s_own_mac, own_mac, 6);
    s_cb = cb;
    s_user_ctx = user_ctx;

    s_state = LEADER_STATE_UNKNOWN;
    memset(s_best_bid_mac, 0, sizeof(s_best_bid_mac));
    s_best_bid_priority = 0;
    memset(s_leader_mac, 0, sizeof(s_leader_mac));
    s_leader_priority = 0;
    s_window_start_ms = 0;
    s_last_leader_heard_ms = 0;
    s_now_ms = 0;

    ESP_LOGI(TAG, "Leader election initialised (own priority %" PRIu32 ")", own_priority);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Actions                                                             */
/* ------------------------------------------------------------------ */

esp_err_t leader_election_trigger(void)
{
    enter_electing(s_now_ms);
    /* Notify the mesh layer so it can broadcast an election bid. */
    fire_cb(LEADER_STATE_ELECTING, s_own_mac, s_own_priority);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Observation feed-in                                                 */
/* ------------------------------------------------------------------ */

void leader_election_observe_bid(const uint8_t mac[6], uint32_t priority)
{
    if (!mac) {
        return;
    }

    if (s_state == LEADER_STATE_ELECTING) {
        if (priority > s_best_bid_priority) {
            s_best_bid_priority = priority;
            memcpy(s_best_bid_mac, mac, 6);
            ESP_LOGD(TAG, "Updated best bid: " MACSTR " priority %" PRIu32, MAC2STR(mac), priority);
        }
        return;
    }

    if (s_state == LEADER_STATE_LEADER) {
        if (priority > s_own_priority) {
            /* A higher-priority node is running — concede immediately. */
            ESP_LOGI(TAG, "Conceding: peer " MACSTR " priority %" PRIu32 " > own %" PRIu32,
                     MAC2STR(mac), priority, s_own_priority);
            enter_electing(s_now_ms);
            fire_cb(LEADER_STATE_ELECTING, s_own_mac, s_own_priority);
        }
    }
}

void leader_election_observe_claim(const uint8_t mac[6], uint32_t priority)
{
    if (!mac) {
        return;
    }

    bool is_current_leader = (memcmp(mac, s_leader_mac, 6) == 0);

    if (is_current_leader) {
        /* Current leader is re-asserting — refresh the liveness timer. */
        s_last_leader_heard_ms = s_now_ms;
        ESP_LOGD(TAG, "Leader " MACSTR " heartbeat at %" PRIu32 " ms", MAC2STR(mac), s_now_ms);
    }

    if (priority > s_leader_priority) {
        /* Better leader found — accept unconditionally. */
        ESP_LOGI(TAG, "Accepting new leader " MACSTR " priority %" PRIu32, MAC2STR(mac), priority);
        memcpy(s_leader_mac, mac, 6);
        s_leader_priority = priority;
        s_last_leader_heard_ms = s_now_ms;
        s_state = LEADER_STATE_FOLLOWER;
        fire_cb(LEADER_STATE_FOLLOWER, s_leader_mac, s_leader_priority);
    }
    /* If we are the leader and the claim has lower priority, ignore it. */
}

/* ------------------------------------------------------------------ */
/* Periodic tick                                                       */
/* ------------------------------------------------------------------ */

void leader_election_tick(uint32_t now_ms)
{
    s_now_ms = now_ms;

    switch (s_state) {
        case LEADER_STATE_ELECTING: {
            uint32_t elapsed = now_ms - s_window_start_ms;
            if (elapsed < ELECTION_WINDOW_MS) {
                break; /* Window still open — keep collecting bids. */
            }

            /* Window closed — resolve: highest bid wins. */
            if (s_best_bid_priority >= s_own_priority &&
                memcmp(s_best_bid_mac, s_own_mac, 6) == 0) {
                /* We are the winner (own priority is the best seen). */
                ESP_LOGI(TAG, "Elected as leader (priority %" PRIu32 ")", s_own_priority);
                s_state = LEADER_STATE_LEADER;
                memcpy(s_leader_mac, s_own_mac, 6);
                s_leader_priority = s_own_priority;
                fire_cb(LEADER_STATE_LEADER, s_leader_mac, s_leader_priority);
            } else {
                /* A peer won — become a follower. */
                ESP_LOGI(TAG, "Election lost; following " MACSTR " (priority %" PRIu32 ")",
                         MAC2STR(s_best_bid_mac), s_best_bid_priority);
                s_state = LEADER_STATE_FOLLOWER;
                memcpy(s_leader_mac, s_best_bid_mac, 6);
                s_leader_priority = s_best_bid_priority;
                s_last_leader_heard_ms = now_ms;
                fire_cb(LEADER_STATE_FOLLOWER, s_leader_mac, s_leader_priority);
            }
            break;
        }

        case LEADER_STATE_FOLLOWER: {
            uint32_t silence = now_ms - s_last_leader_heard_ms;
            if (silence >= LEADER_TIMEOUT_MS) {
                ESP_LOGW(TAG,
                         "Leader " MACSTR " silent for %" PRIu32 " ms — triggering re-election",
                         MAC2STR(s_leader_mac), silence);
                /* Clear leader state and notify app. */
                memset(s_leader_mac, 0, sizeof(s_leader_mac));
                s_leader_priority = 0;
                s_state = LEADER_STATE_UNKNOWN;
                fire_cb(LEADER_STATE_UNKNOWN, s_leader_mac, 0);
                /* Immediately start a new election. */
                enter_electing(now_ms);
                fire_cb(LEADER_STATE_ELECTING, s_own_mac, s_own_priority);
            }
            break;
        }

        case LEADER_STATE_UNKNOWN:
        case LEADER_STATE_LEADER:
            /* No timer-driven transitions from these states in tick(). */
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

leader_state_t leader_election_get_state(void)
{
    return s_state;
}

bool leader_election_is_leader(void)
{
    return s_state == LEADER_STATE_LEADER;
}

void leader_election_get_leader(uint8_t out_mac[6], uint32_t *out_priority)
{
    if (out_mac) {
        memcpy(out_mac, s_leader_mac, 6);
    }
    if (out_priority) {
        *out_priority = s_leader_priority;
    }
}
