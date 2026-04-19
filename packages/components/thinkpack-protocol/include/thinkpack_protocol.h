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
/* Fragmentation constants                                             */
/* ------------------------------------------------------------------ */

/** Data bytes per fragment — leaves room for the fragment header. */
#define THINKPACK_MAX_FRAGMENT_DATA 192
/** Maximum fragments per large message (16 × 192 = 3072 bytes). */
#define THINKPACK_MAX_FRAGMENTS 16
/** Maximum bytes a reassembled large message may span. */
#define THINKPACK_MAX_REASSEMBLED (THINKPACK_MAX_FRAGMENTS * THINKPACK_MAX_FRAGMENT_DATA)

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
    MSG_COLLECTIVE_TRIGGER = 0x0B,
    MSG_FRAGMENT = 0x0C,             /**< Fragment of a large message         */
    MSG_AUDIO_CLIP_BROADCAST = 0x0D, /**< Leader-driven echo-chamber clip ref */
    /* 0x0E reserved for future audio use */
    MSG_OTA_MANIFEST = 0x0F, /**< OTA firmware manifest (PR F)                 */
    MSG_OTA_CHUNK = 0x10,    /**< OTA firmware chunk (PR F)                    */
    MSG_OTA_ACK = 0x11,      /**< OTA aggregated ACK (PR F)                    */
    MSG_OTA_COMPLETE = 0x12, /**< OTA completion marker + SHA256 (PR F)        */
    MSG_OTA_ERROR = 0x13,    /**< OTA error notification (PR F)                */
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

/**
 * @brief Payload for MSG_FRAGMENT — one slice of a large message.
 *
 * Senders split a large buffer into up to THINKPACK_MAX_FRAGMENTS fragments
 * and send each as a separate MSG_FRAGMENT packet.  Receivers accumulate
 * fragments and emit a single THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED event
 * once all fragments for a (src_mac, msg_id) arrive.
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_id;                            /**< Unique ID per large message (wraps mod 256) */
    uint8_t fragment_index;                    /**< 0-based index of this fragment              */
    uint8_t total_fragments;                   /**< Total number of fragments in the message    */
    uint8_t original_msg_type;                 /**< Logical type wrapped (e.g. MSG_LLM_RESPONSE) */
    uint8_t data_length;                       /**< Bytes of data[] populated in this fragment  */
    uint8_t data[THINKPACK_MAX_FRAGMENT_DATA]; /**< Fragment payload bytes     */
} thinkpack_fragment_data_t;

/**
 * @brief Payload for MSG_AUDIO_CLIP_BROADCAST — echo-chamber clip metadata.
 *
 * Chatterbox leader announces a just-recorded clip; each follower applies the
 * semitone shift in @p per_peer_semitone_shift at its own slot index (0-based,
 * wraps mod THINKPACK_MAX_PEERS).  The actual PCM is streamed separately via
 * the fragmented-send path (thinkpack_mesh_send_large with MSG_LLM_RESPONSE-
 * style transport); this payload is a lightweight metadata header carried in
 * a normal-size packet so every box learns about the clip quickly.
 *
 * Slot mapping convention: the slot index is the follower's position in the
 * leader's peer table at send time (0-based).  Followers that don't know
 * their own slot default to slot 0.  Unused slots are clamped to 0 semitones.
 */
typedef struct __attribute__((packed)) {
    uint8_t clip_id;                   /**< Unique per broadcast (wraps mod 256) */
    uint16_t sample_count;             /**< PCM samples (<= THINKPACK_AUDIO_CLIP_MAX_SAMPLES) */
    int8_t per_peer_semitone_shift[8]; /**< Per-slot pitch offset, clamped to ±12 */
    uint8_t flags;                     /**< Reserved — must be 0 */
} audio_clip_broadcast_payload_t;

/* ------------------------------------------------------------------ */
/* OTA payloads (PR F)                                                 */
/*                                                                     */
/* OTA uses the mesh to relay firmware from Brainbox to followers.    */
/* A manifest announces the update; chunks carry the binary; a        */
/* completion packet carries the full SHA256 for end-to-end verify.   */
/* All structs are packed and sized to fit inside THINKPACK_MAX_DATA_LEN. */
/* ------------------------------------------------------------------ */

/** Maximum payload bytes carried in a single OTA chunk. */
#define THINKPACK_OTA_CHUNK_DATA_MAX 180
/** Maximum number of chunks in a single OTA session (≈180 KB @180 B/chunk).
 *  Larger firmware must be split into multiple sessions or chunk size raised. */
#define THINKPACK_OTA_MAX_CHUNKS 1024
/** Sentinel target_box_mask meaning "everyone". */
#define THINKPACK_OTA_TARGET_ALL 0xFFu

/** Payload for MSG_OTA_MANIFEST — announces an incoming firmware update. */
typedef struct __attribute__((packed)) {
    uint32_t version;        /**< Monotonic firmware version integer. */
    uint32_t total_size;     /**< Total firmware size in bytes.       */
    uint16_t chunk_count;    /**< Number of MSG_OTA_CHUNK packets.    */
    uint8_t sha256[32];      /**< Expected SHA256 of the whole image. */
    uint8_t target_box_mask; /**< Bitmask over thinkpack_box_type_t,
                                  or THINKPACK_OTA_TARGET_ALL.        */
    uint8_t reserved;        /**< Pad to even length; must be 0.      */
} ota_manifest_payload_t;

/** Payload for MSG_OTA_CHUNK — one slice of the firmware image. */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;                       /**< 0-based index.     */
    uint8_t data_len;                           /**< Bytes in data[].   */
    uint8_t data[THINKPACK_OTA_CHUNK_DATA_MAX]; /**< Raw firmware bytes.*/
} ota_chunk_payload_t;

/** Payload for MSG_OTA_ACK — aggregated acknowledgement. */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;    /**< Highest contiguous chunk received.    */
    uint8_t receiver_box_id; /**< thinkpack_box_type_t of the ACKer.    */
} ota_ack_payload_t;

/** Payload for MSG_OTA_COMPLETE — all chunks sent, final SHA256. */
typedef struct __attribute__((packed)) {
    uint32_t version;         /**< Matches the manifest version.      */
    uint8_t success;          /**< 1 = broadcast OK, 0 = aborted.     */
    uint8_t final_sha256[32]; /**< Full-image SHA256 for verification.*/
} ota_complete_payload_t;

/** Payload for MSG_OTA_ERROR — notifies sender that a receiver aborted. */
typedef struct __attribute__((packed)) {
    uint8_t error_code;   /**< thinkpack_ota_error_t, see thinkpack_ota.h */
    uint16_t chunk_index; /**< Chunk where the failure was detected.      */
} ota_error_payload_t;

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

/**
 * @brief Build a MSG_FRAGMENT packet.
 *
 * @param p        Packet buffer to populate.
 * @param seq      Sequence number.
 * @param src_mac  Sender's 6-byte MAC.
 * @param fragment Fragment payload to embed.
 */
void thinkpack_prepare_fragment(thinkpack_packet_t *p, uint8_t seq, const uint8_t src_mac[6],
                                const thinkpack_fragment_data_t *fragment);

/**
 * @brief Compute the number of fragments needed to send @p total_bytes.
 *
 * Returns 0 for zero-length input; otherwise ceil(total_bytes /
 * THINKPACK_MAX_FRAGMENT_DATA), capped at THINKPACK_MAX_FRAGMENTS.
 *
 * @param total_bytes  Total number of bytes to fragment.
 * @return             Number of fragments (0 … THINKPACK_MAX_FRAGMENTS).
 */
static inline uint8_t thinkpack_fragment_count(size_t total_bytes)
{
    if (total_bytes == 0) {
        return 0;
    }
    size_t count = (total_bytes + THINKPACK_MAX_FRAGMENT_DATA - 1) / THINKPACK_MAX_FRAGMENT_DATA;
    return (uint8_t)(count > THINKPACK_MAX_FRAGMENTS ? THINKPACK_MAX_FRAGMENTS : count);
}

#endif /* THINKPACK_PROTOCOL_H */
