/**
 * @file thinkpack_ota.h
 * @brief OTA firmware relay over ThinkPack ESP-NOW mesh.
 *
 * The Brainbox downloads firmware from a GitHub Release using the
 * `ota-github` component, then fragments the binary into
 * MSG_OTA_CHUNK packets and broadcasts them to the mesh. Followers
 * (glowbug, boombox, chatterbox, finderbox) register a receiver via
 * @ref thinkpack_ota_receiver_init; on MSG_OTA_COMPLETE with a
 * matching SHA256 they apply the image via `esp_ota_*` and reboot.
 *
 * ### Module boundaries
 *  - `chunk_state.c`       — pure-logic chunk assembly (host-testable).
 *  - `manifest_parser.c`   — pure-logic manifest validator.
 *  - `sha256_verify.c`     — thin wrapper over mbedtls (ESP) / vendored
 *                            single-file SHA256 (host tests).
 *  - `apply.c`             — esp_ota_* wrapper (ESP-only, guarded).
 *  - `ota_receiver.c`      — mesh event hook that drives the state
 *                            machine + applies on success.
 *
 * ### Broadcast-rate limiter (Brainbox side)
 *
 * The Brainbox broadcaster inserts a delay of
 * @ref THINKPACK_OTA_DEFAULT_CHUNK_DELAY_MS milliseconds between each
 * MSG_OTA_CHUNK packet. At 180 bytes per chunk and the default 10 ms
 * spacing this yields ≈18 kB/s and keeps the WiFi radio below the
 * burst-loss threshold observed in bench tests. A 1 MB firmware
 * therefore takes ≈60 s plus manifest/complete overhead. Tune via
 * @ref thinkpack_ota_broadcaster_cfg_t::inter_chunk_delay_ms.
 */

#ifndef THINKPACK_OTA_H
#define THINKPACK_OTA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "thinkpack_protocol.h"

#ifdef ESP_PLATFORM
#include "esp_err.h"
#else
typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NO_MEM 0x101
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

/** Default inter-chunk spacing in milliseconds. */
#define THINKPACK_OTA_DEFAULT_CHUNK_DELAY_MS 10

/** Number of failed boots after which the follower rolls back. */
#define THINKPACK_OTA_ROLLBACK_THRESHOLD 3

/* ------------------------------------------------------------------ */
/* Error codes reported via MSG_OTA_ERROR                              */
/* ------------------------------------------------------------------ */

typedef enum {
    THINKPACK_OTA_ERR_NONE = 0,
    THINKPACK_OTA_ERR_MANIFEST_INVALID = 1,
    THINKPACK_OTA_ERR_CHUNK_OVERFLOW = 2,
    THINKPACK_OTA_ERR_CHUNK_MISSING = 3,
    THINKPACK_OTA_ERR_SIZE_MISMATCH = 4,
    THINKPACK_OTA_ERR_SHA_MISMATCH = 5,
    THINKPACK_OTA_ERR_APPLY_FAILED = 6,
    THINKPACK_OTA_ERR_OUT_OF_MEMORY = 7,
} thinkpack_ota_error_t;

/* ------------------------------------------------------------------ */
/* Manifest validation (pure logic)                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Validate a manifest payload for internal consistency.
 *
 * Checks: non-zero version, total_size non-zero and ≤ chunk_count ×
 * THINKPACK_OTA_CHUNK_DATA_MAX, chunk_count ≤ THINKPACK_OTA_MAX_CHUNKS,
 * reserved byte = 0.
 *
 * @return THINKPACK_OTA_ERR_NONE on success, or a specific error code.
 */
thinkpack_ota_error_t thinkpack_ota_manifest_validate(const ota_manifest_payload_t *m);

/**
 * @brief Whether this node should accept an update for @p target_mask.
 *
 * @param target_mask  manifest's target_box_mask.
 * @param my_box_type  this node's thinkpack_box_type_t.
 * @return true if the node should participate in the update.
 */
bool thinkpack_ota_manifest_targets_box(uint8_t target_mask, uint8_t my_box_type);

/* ------------------------------------------------------------------ */
/* Chunk assembly state machine (pure logic)                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Accumulating state for a single OTA session.
 *
 * The caller allocates a uint32_t bitmap of
 * `(chunk_count + 31) / 32` words and a `total_size`-byte image
 * buffer, and passes them in via @ref thinkpack_ota_chunk_state_init.
 * This keeps the state machine allocation-free and host-testable.
 */
typedef struct {
    uint32_t version;          /**< From manifest.           */
    uint32_t total_size;       /**< From manifest.           */
    uint16_t chunk_count;      /**< From manifest.           */
    uint16_t chunks_received;  /**< Running tally.           */
    uint8_t *image;            /**< Caller-owned buffer.     */
    uint32_t *received_bitmap; /**< Caller-owned bitmap.     */
    size_t bitmap_words;       /**< Bitmap length in u32s.   */
    uint8_t expected_sha[32];  /**< Copied from manifest.    */
} thinkpack_ota_chunk_state_t;

/**
 * @brief Initialise a chunk-state from a validated manifest.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_ARG on NULL / oversize input.
 */
esp_err_t thinkpack_ota_chunk_state_init(thinkpack_ota_chunk_state_t *s,
                                         const ota_manifest_payload_t *m, uint8_t *image_buf,
                                         size_t image_buf_size, uint32_t *bitmap_buf,
                                         size_t bitmap_buf_words);

/**
 * @brief Integrate a single chunk into the state.
 *
 * Ignores duplicates, rejects out-of-range or oversized chunks.
 *
 * @return THINKPACK_OTA_ERR_NONE if the chunk was accepted (including
 *         duplicates), or a specific error code.
 */
thinkpack_ota_error_t thinkpack_ota_chunk_state_update(thinkpack_ota_chunk_state_t *s,
                                                       const ota_chunk_payload_t *c);

/** True when every chunk has been received exactly once. */
bool thinkpack_ota_chunk_state_is_complete(const thinkpack_ota_chunk_state_t *s);

/**
 * @brief After completion, verify the assembled image SHA256.
 *
 * @return THINKPACK_OTA_ERR_NONE if SHA256 matches the manifest, else
 *         THINKPACK_OTA_ERR_SHA_MISMATCH.
 */
thinkpack_ota_error_t thinkpack_ota_chunk_state_finalize(const thinkpack_ota_chunk_state_t *s);

/* ------------------------------------------------------------------ */
/* SHA256 helper                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Compute SHA256 of @p data into @p out.
 *
 * @param data Input buffer (may be NULL iff len == 0).
 * @param len  Input length in bytes.
 * @param out  32-byte output buffer.
 */
void thinkpack_ota_sha256(const uint8_t *data, size_t len, uint8_t out[32]);

/**
 * @brief Compare SHA256(@p data, @p len) against @p expected.
 *
 * @return true if the digest matches.
 */
bool thinkpack_ota_sha256_verify(const uint8_t *data, size_t len, const uint8_t expected[32]);

/* ------------------------------------------------------------------ */
/* ESP-only apply wrapper                                              */
/* ------------------------------------------------------------------ */

#ifdef ESP_PLATFORM
/**
 * @brief Write @p image to the next OTA partition and schedule a
 *        reboot into it.
 *
 * Assumes the caller has already verified SHA256. On error, the
 * partition is marked invalid so the bootloader rolls back.
 *
 * @return ESP_OK on success; never returns on success (reboots).
 */
esp_err_t thinkpack_ota_apply(const ota_complete_payload_t *cp, const uint8_t *image, size_t len);
#endif

/* ------------------------------------------------------------------ */
/* Receiver (ESP-only)                                                 */
/* ------------------------------------------------------------------ */

#ifdef ESP_PLATFORM
/**
 * @brief Initialise the OTA receiver.
 *
 * Registers a mesh event observer that listens for MSG_OTA_MANIFEST /
 * CHUNK / COMPLETE / ERROR frames and drives the chunk state machine.
 * On MSG_OTA_COMPLETE with a valid SHA256 it calls @ref
 * thinkpack_ota_apply which reboots into the new image.
 *
 * @param my_box_type  thinkpack_box_type_t of this follower; used to
 *                     filter manifests by target_box_mask.
 */
esp_err_t thinkpack_ota_receiver_init(uint8_t my_box_type);

/* Forward-declare the mesh callback type without pulling the mesh
 * header into pure-logic consumers. Declaring the struct as opaque. */
struct thinkpack_mesh_event_data_s;

typedef void (*thinkpack_ota_chained_cb_t)(const struct thinkpack_mesh_event_data_s *ev,
                                           void *user_ctx);

/**
 * @brief Install a callback that will be forwarded BEFORE the OTA
 *        receiver inspects each mesh event. Must be called before
 *        @ref thinkpack_ota_receiver_init.
 */
esp_err_t thinkpack_ota_receiver_chain_callback(thinkpack_ota_chained_cb_t cb, void *ctx);
#endif

#ifdef __cplusplus
}
#endif

#endif /* THINKPACK_OTA_H */
