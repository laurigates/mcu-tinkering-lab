/**
 * @file espnow_mesh.h
 * @brief Public API for the ThinkPack ESP-NOW mesh layer.
 *
 * Wraps ESP-NOW to provide peer discovery, leader election, and group
 * management for the ThinkPack modular toy mesh (up to THINKPACK_MAX_PEERS
 * nodes on a single WiFi channel).
 *
 * Typical usage:
 * @code
 *   thinkpack_mesh_config_t cfg = {
 *       .box_type          = BOX_BRAINBOX,
 *       .capabilities      = CAP_WIFI | CAP_LLM | CAP_DISPLAY,
 *       .channel           = 1,
 *       .battery_level     = 100,
 *       .name              = "brainbox-01",
 *       .beacon_interval_ms = 500,
 *   };
 *   thinkpack_mesh_init(&cfg);
 *   thinkpack_mesh_set_event_callback(my_cb, NULL);
 *   thinkpack_mesh_start();
 * @endcode
 */

#ifndef ESPNOW_MESH_H
#define ESPNOW_MESH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "thinkpack_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Mesh initialisation configuration.
 *
 * Pass to thinkpack_mesh_init() before calling thinkpack_mesh_start().
 */
typedef struct {
    uint8_t box_type;                  /**< thinkpack_box_type_t */
    uint16_t capabilities;             /**< CAP_* bitmask */
    uint8_t channel;                   /**< WiFi channel (1-13) */
    uint8_t battery_level;             /**< 0-100 percent */
    char name[THINKPACK_BOX_NAME_LEN]; /**< Human-readable box name (NUL-term) */
    uint32_t beacon_interval_ms;       /**< Beacon period; typical 500 ms for 2 Hz */
} thinkpack_mesh_config_t;

/* ------------------------------------------------------------------ */
/* Events                                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief Event types delivered to the application via the event callback.
 */
typedef enum {
    THINKPACK_EVENT_PEER_DISCOVERED,        /**< A new peer was seen for the first time    */
    THINKPACK_EVENT_PEER_LOST,              /**< A previously known peer has gone stale    */
    THINKPACK_EVENT_LEADER_ELECTED,         /**< Election resolved; a leader is known      */
    THINKPACK_EVENT_LEADER_LOST,            /**< Leader went silent; re-election triggered */
    THINKPACK_EVENT_BECAME_LEADER,          /**< This node won the election                */
    THINKPACK_EVENT_BECAME_FOLLOWER,        /**< This node deferred to a higher-priority peer */
    THINKPACK_EVENT_SYNC_PULSE,             /**< MSG_SYNC_PULSE received from leader       */
    THINKPACK_EVENT_COMMAND_RECEIVED,       /**< MSG_COMMAND received                      */
    THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED, /**< All fragments reassembled; full payload ready */
} thinkpack_mesh_event_t;

/**
 * @brief Event data passed to the application callback.
 *
 * @p packet is valid only for THINKPACK_EVENT_SYNC_PULSE and
 * THINKPACK_EVENT_COMMAND_RECEIVED. It points into a temporary buffer
 * on the receive task stack and must not be retained after the callback
 * returns.
 *
 * For THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED:
 *   - @p large_data points to the fully reassembled buffer (valid until the
 *     callback returns; do not retain the pointer).
 *   - @p large_length is the total number of reassembled bytes.
 *   - @p original_msg_type is the logical message type that was fragmented
 *     (e.g. MSG_LLM_RESPONSE).
 *   - @p packet is NULL.
 * All three fields are zeroed / NULL for every other event type.
 */
typedef struct {
    thinkpack_mesh_event_t type;      /**< Event kind                                       */
    uint8_t peer_mac[6];              /**< MAC address of the relevant peer                 */
    const thinkpack_packet_t *packet; /**< Non-NULL for SYNC_PULSE / COMMAND_RECEIVED       */
    const uint8_t *large_data;        /**< Reassembled buffer (LARGE_MESSAGE_RECEIVED only) */
    size_t large_length;              /**< Byte count of large_data                         */
    uint8_t original_msg_type;        /**< Wrapped logical type (LARGE_MESSAGE_RECEIVED only) */
} thinkpack_mesh_event_data_t;

/**
 * @brief Application-supplied event callback.
 *
 * Called from the receive task on Core 0. Keep it short — do not block
 * or call ESP-IDF APIs that require the WiFi task.
 *
 * @param event     Event descriptor; valid only for the duration of the call.
 * @param user_ctx  Opaque pointer supplied to thinkpack_mesh_set_event_callback().
 */
typedef void (*thinkpack_mesh_event_cb_t)(const thinkpack_mesh_event_data_t *event, void *user_ctx);

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the mesh layer.
 *
 * Initialises NVS, the WiFi driver, ESP-NOW, the group manager, and the
 * leader-election module. Does NOT start background tasks — call
 * thinkpack_mesh_start() for that.
 *
 * @param config  Non-NULL pointer to mesh configuration.
 * @return ESP_OK on success, or a forwarded ESP-IDF error code.
 */
esp_err_t thinkpack_mesh_init(const thinkpack_mesh_config_t *config);

/**
 * @brief Start the beacon and receive FreeRTOS tasks.
 *
 * Must be called after thinkpack_mesh_init(). Triggers an initial leader
 * election bid.
 *
 * @return ESP_OK on success.
 */
esp_err_t thinkpack_mesh_start(void);

/**
 * @brief Stop background tasks and deinitialise ESP-NOW.
 *
 * The WiFi driver is left running so the caller can reinitialise or shut
 * down independently.
 *
 * @return ESP_OK on success.
 */
esp_err_t thinkpack_mesh_stop(void);

/* ------------------------------------------------------------------ */
/* Configuration / callbacks                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Register the application event callback.
 *
 * May be called before or after thinkpack_mesh_start(). Thread-safe.
 *
 * @param cb        Callback function, or NULL to deregister.
 * @param user_ctx  Opaque pointer forwarded to every callback invocation.
 * @return ESP_OK always.
 */
esp_err_t thinkpack_mesh_set_event_callback(thinkpack_mesh_event_cb_t cb, void *user_ctx);

/* ------------------------------------------------------------------ */
/* Sending                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Transmit a prepared packet.
 *
 * If @p mac is NULL the packet is broadcast to all peers. Otherwise the
 * peer is registered with ESP-NOW if not already present, then the packet
 * is sent unicast.
 *
 * @param mac     6-byte destination MAC, or NULL for broadcast.
 * @param packet  Non-NULL prepared packet (magic + checksum already set).
 * @return ESP_OK on success, or a forwarded ESP-IDF error code.
 */
esp_err_t thinkpack_mesh_send(const uint8_t *mac, const thinkpack_packet_t *packet);

/**
 * @brief Send a large buffer, fragmenting if necessary.
 *
 * If @p length <= THINKPACK_MAX_DATA_LEN the data is sent as a single packet
 * with @p msg_type directly (no fragmentation overhead). Otherwise the buffer
 * is split into MSG_FRAGMENT packets (each wrapping @p msg_type) and sent
 * sequentially with a 5 ms inter-frame delay to avoid overwhelming the
 * ESP-NOW driver.
 *
 * The receiver collects all fragments and fires a single
 * THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED event once every fragment arrives.
 *
 * @param mac      6-byte destination MAC, or NULL for broadcast.
 * @param msg_type Logical message type (e.g. MSG_LLM_RESPONSE).
 * @param data     Buffer to send (must not be NULL).
 * @param length   Bytes in @p data (max THINKPACK_MAX_REASSEMBLED).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for bad arguments,
 *         or a forwarded ESP-IDF error on send failure.
 */
esp_err_t thinkpack_mesh_send_large(const uint8_t *mac, uint8_t msg_type, const uint8_t *data,
                                    size_t length);

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Retrieve the local WiFi-STA MAC address.
 *
 * @param out_mac  Output buffer; receives 6 bytes.
 */
void thinkpack_mesh_get_mac(uint8_t out_mac[6]);

/**
 * @brief Return the number of peers currently tracked by the group manager.
 *
 * @return Peer count (0 … THINKPACK_MAX_PEERS).
 */
size_t thinkpack_mesh_peer_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ESPNOW_MESH_H */
