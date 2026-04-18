/**
 * @file thinkpack_protocol.h
 * @brief ESP-NOW packet format, message types, and capability flags for the
 *        ThinkPack modular toy mesh.
 *
 * See docs/requirements/PRD-010 and docs/decisions/ADR-014 for design rationale.
 */

#ifndef THINKPACK_PROTOCOL_H
#define THINKPACK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Magic bytes and versioning                                          */
/* ------------------------------------------------------------------ */

/** ASCII 't' (0x74) — first magic byte identifying ThinkPack frames. */
#define THINKPACK_MAGIC_0 0x74
/** Protocol version byte — embedded in the second magic position. */
#define THINKPACK_MAGIC_VERSION 0x01

/* ------------------------------------------------------------------ */
/* Protocol constants                                                  */
/* ------------------------------------------------------------------ */

/** Maximum payload bytes carried inside a packet's data field. */
#define THINKPACK_MAX_DATA_LEN 200
/** Maximum number of peer nodes tracked by the mesh. */
#define THINKPACK_MAX_PEERS 10
/** Length of a human-readable box name (including NUL terminator). */
#define THINKPACK_BOX_NAME_LEN 16

/* ------------------------------------------------------------------ */
/* Message types                                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    MSG_BEACON = 0x01,
    MSG_ELECTION_BID = 0x02,
    MSG_LEADER_CLAIM = 0x03,
    MSG_CAPABILITY_REQUEST = 0x04,
    MSG_CAPABILITY_REPLY = 0x05,
    MSG_COMMAND = 0x06,
    MSG_STATUS = 0x07,
    MSG_LLM_REQUEST = 0x08,
    MSG_LLM_RESPONSE = 0x09,
    MSG_SYNC_PULSE = 0x0A,
    MSG_COLLECTIVE_TRIGGER = 0x0B
} thinkpack_msg_type_t;

/* ------------------------------------------------------------------ */
/* Box types                                                           */
/* ------------------------------------------------------------------ */

typedef enum {
    BOX_UNKNOWN = 0,
    BOX_CHATTERBOX = 1,
    BOX_GLOWBUG = 2,
    BOX_BRAINBOX = 3,
    BOX_BOOMBOX = 4,
    BOX_FINDERBOX = 5
} thinkpack_box_type_t;

/* ------------------------------------------------------------------ */
/* Capability bitmask (FR-T08)                                         */
/* ------------------------------------------------------------------ */

#define CAP_WIFI (1u << 0)        /**< WiFi connectivity             */
#define CAP_LLM (1u << 1)         /**< On-device / proxied LLM       */
#define CAP_AUDIO_OUT (1u << 2)   /**< Speaker / audio output        */
#define CAP_AUDIO_IN (1u << 3)    /**< Microphone / audio input      */
#define CAP_DISPLAY (1u << 4)     /**< Screen or e-ink display       */
#define CAP_LED_RING (1u << 5)    /**< Addressable LED ring          */
#define CAP_IMU (1u << 6)         /**< Inertial measurement unit     */
#define CAP_LIGHT_SENSE (1u << 7) /**< Ambient light sensor          */
#define CAP_NFC (1u << 8)         /**< NFC reader / writer           */
#define CAP_POTS (1u << 9)        /**< Potentiometers / knobs        */
#define CAP_TOUCH (1u << 10)      /**< Capacitive touch pads         */
#define CAP_RHYTHM (1u << 11)     /**< Rhythm / beat detection       */
#define CAP_STORAGE (1u << 12)    /**< Persistent local storage      */

/* ------------------------------------------------------------------ */
/* Leader election priority scores (FR-T07)                            */
/*                                                                     */
/* Priority is a uint32_t:                                             */
/*   bits 31-16: capability score (max 1850, well within 16 bits)     */
/*   bits 15-0:  MAC tiebreaker (mac[4] << 8 | mac[5])               */
/* ------------------------------------------------------------------ */

#define PRIO_CAP_LLM 1000      /**< Points for CAP_LLM       */
#define PRIO_CAP_WIFI 500      /**< Points for CAP_WIFI      */
#define PRIO_CAP_DISPLAY 200   /**< Points for CAP_DISPLAY   */
#define PRIO_CAP_AUDIO_OUT 100 /**< Points for CAP_AUDIO_OUT */
#define PRIO_CAP_AUDIO_IN 50   /**< Points for CAP_AUDIO_IN  */

/* ------------------------------------------------------------------ */
/* Wire packet                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Complete ESP-NOW frame transmitted between ThinkPack boxes.
 *
 * Always ≤ 212 bytes, comfortably within the 250-byte ESP-NOW limit.
 */
typedef struct __attribute__((packed)) {
    uint8_t magic[2];                     /**< {THINKPACK_MAGIC_0, THINKPACK_MAGIC_VERSION} */
    uint8_t msg_type;                     /**< thinkpack_msg_type_t                         */
    uint8_t sequence_number;              /**< Rolling sequence counter                     */
    uint8_t src_mac[6];                   /**< Sender MAC address                           */
    uint8_t data_length;                  /**< Bytes of data[] actually populated           */
    uint8_t data[THINKPACK_MAX_DATA_LEN]; /**< Payload — cast to payload struct  */
    uint8_t checksum;                     /**< XOR of all preceding bytes                   */
} thinkpack_packet_t;

/* ------------------------------------------------------------------ */
/* Payload structs                                                     */
/* ------------------------------------------------------------------ */

/** Payload for MSG_BEACON — periodic self-announcement. */
typedef struct __attribute__((packed)) {
    uint8_t box_type;                  /**< thinkpack_box_type_t           */
    uint16_t capabilities;             /**< CAP_* bitmask                  */
    uint32_t priority;                 /**< Capability score + MAC tiebreaker */
    uint8_t battery_level;             /**< 0-100 percent                  */
    uint8_t group_state;               /**< Application-defined group state */
    char name[THINKPACK_BOX_NAME_LEN]; /**< Human-readable name (NUL-term) */
} thinkpack_beacon_data_t;

/** Payload for MSG_ELECTION_BID — candidate announces its priority. */
typedef struct __attribute__((packed)) {
    uint32_t priority; /**< Capability score + MAC tiebreaker */
    uint8_t mac[6];    /**< Candidate MAC address             */
} thinkpack_election_bid_data_t;

/** Payload for MSG_LEADER_CLAIM — winner announces itself. */
typedef struct __attribute__((packed)) {
    uint32_t priority; /**< Winning priority value  */
    uint8_t mac[6];    /**< Leader MAC address      */
    uint8_t channel;   /**< Operating channel       */
} thinkpack_leader_claim_data_t;

/** Payload for MSG_CAPABILITY_REPLY — response to capability request. */
typedef struct __attribute__((packed)) {
    uint8_t box_type;                  /**< thinkpack_box_type_t           */
    uint16_t capabilities;             /**< CAP_* bitmask                  */
    uint32_t priority;                 /**< Capability score + MAC tiebreaker */
    char name[THINKPACK_BOX_NAME_LEN]; /**< Human-readable name (NUL-term) */
} thinkpack_capability_reply_data_t;

/** Payload for MSG_SYNC_PULSE — leader-driven synchronisation tick. */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms; /**< Leader's millisecond timestamp */
    uint8_t phase;         /**< Current animation/sync phase   */
} thinkpack_sync_pulse_data_t;

/**
 * @brief Generic command envelope (MSG_COMMAND).
 *
 * Real command payload structures are defined in later phases and
 * stored in the @p payload field.
 */
typedef struct __attribute__((packed)) {
    uint8_t command_id;  /**< Application-defined command identifier */
    uint8_t length;      /**< Bytes used in payload[]                */
    uint8_t payload[48]; /**< Command-specific data                  */
} thinkpack_command_data_t;

/* ------------------------------------------------------------------ */
/* Helper function prototypes                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Compute XOR checksum over @p length bytes of @p data.
 * @return XOR of all bytes, or 0 for empty input.
 */
uint8_t thinkpack_checksum(const uint8_t *data, size_t length);

/**
 * @brief Verify that a packet's checksum field is consistent with its content.
 * @return true if the checksum is valid, false otherwise.
 */
bool thinkpack_verify_checksum(const thinkpack_packet_t *p);

/**
 * @brief Fill @p p's magic bytes and compute its trailing checksum.
 *
 * Call after populating all other fields.
 */
void thinkpack_finalize(thinkpack_packet_t *p);

/**
 * @brief Build a MSG_BEACON packet.
 *
 * @param p       Packet buffer to populate.
 * @param seq     Sequence number.
 * @param src_mac Sender's 6-byte MAC.
 * @param beacon  Beacon payload to embed.
 */
void thinkpack_prepare_beacon(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                              const thinkpack_beacon_data_t *beacon);

/**
 * @brief Build a MSG_ELECTION_BID packet.
 *
 * @param p        Packet buffer to populate.
 * @param seq      Sequence number.
 * @param src_mac  Sender's 6-byte MAC.
 * @param priority Computed election priority.
 */
void thinkpack_prepare_election_bid(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                    uint32_t priority);

/**
 * @brief Build a MSG_LEADER_CLAIM packet.
 *
 * @param p        Packet buffer to populate.
 * @param seq      Sequence number.
 * @param src_mac  Sender's 6-byte MAC.
 * @param priority Winning priority value.
 * @param channel  Operating channel the leader has selected.
 */
void thinkpack_prepare_leader_claim(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                    uint32_t priority, uint8_t channel);

/**
 * @brief Build a MSG_SYNC_PULSE packet.
 *
 * @param p            Packet buffer to populate.
 * @param seq          Sequence number.
 * @param src_mac      Sender's 6-byte MAC.
 * @param timestamp_ms Leader's current millisecond timestamp.
 * @param phase        Current synchronisation phase.
 */
void thinkpack_prepare_sync_pulse(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                  uint32_t timestamp_ms, uint8_t phase);

/**
 * @brief Compute the election priority for a node.
 *
 * Priority encoding:
 *   - Upper 16 bits: sum of PRIO_CAP_* for each set capability bit
 *   - Lower 16 bits: MAC tiebreaker (mac[4] << 8 | mac[5])
 *
 * The capability score dominates; the MAC tiebreaker breaks ties
 * deterministically without coordination.
 *
 * @param capabilities  CAP_* bitmask for this node.
 * @param mac           6-byte MAC address of this node.
 * @return              32-bit priority value.
 */
uint32_t thinkpack_priority_for_capabilities(uint16_t capabilities, const uint8_t mac[6]);

#endif /* THINKPACK_PROTOCOL_H */
