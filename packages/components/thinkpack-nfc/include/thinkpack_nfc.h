/**
 * @file thinkpack_nfc.h
 * @brief Pure-logic NFC tag registry for ThinkPack Finderbox.
 *
 * Maps NFC UID bytes to behaviour labels and parameters, and provides
 * a compact blob (de)serialiser suitable for the ESP32 NVS blob API.
 *
 * No ESP-IDF dependencies — safe to compile on a host with gcc/clang.
 */

#ifndef THINKPACK_NFC_H
#define THINKPACK_NFC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum UID length supported (MIFARE 7-byte UID). */
#define THINKPACK_NFC_UID_MAX_LEN 7

/** Maximum number of tag entries in a registry. */
#define THINKPACK_NFC_MAX_TAGS 16

/** Maximum label length including trailing NUL. */
#define THINKPACK_NFC_LABEL_MAX_LEN 24

/** Blob magic byte: 't' for thinkpack. */
#define THINKPACK_NFC_BLOB_MAGIC 0x74u

/** Blob format version byte. */
#define THINKPACK_NFC_BLOB_VERSION 0x01u

/** Named behaviours carried in thinkpack_nfc_entry_t::behavior. */
typedef enum {
    THINKPACK_NFC_BEHAVIOR_NONE = 0,
    THINKPACK_NFC_BEHAVIOR_CHIME = 1, /**< Play chime + LED colour (param = hue) */
    THINKPACK_NFC_BEHAVIOR_STORY = 2, /**< Trigger LLM story (param = track_id) */
    THINKPACK_NFC_BEHAVIOR_COLOR = 3, /**< Set global mood colour (param = hue) */
    THINKPACK_NFC_BEHAVIOR_SEEK = 4,  /**< Hot-Cold seek target (param unused) */
} thinkpack_nfc_behavior_t;

typedef struct {
    uint8_t uid[THINKPACK_NFC_UID_MAX_LEN];
    uint8_t uid_len; /**< Valid length 1..THINKPACK_NFC_UID_MAX_LEN. */
    uint8_t behavior;
    uint8_t param;
    char label[THINKPACK_NFC_LABEL_MAX_LEN];
} thinkpack_nfc_entry_t;

typedef struct {
    thinkpack_nfc_entry_t entries[THINKPACK_NFC_MAX_TAGS];
    uint8_t count;
} thinkpack_nfc_registry_t;

/* ------------------------------------------------------------------ */
/* UID helpers                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Copy @p src into @p dst, clamping to THINKPACK_NFC_UID_MAX_LEN.
 *
 * Unused trailing bytes in @p dst are zero-filled.  If @p src_len
 * exceeds the max, only the first MAX bytes are copied and dst_len is
 * set to MAX.
 */
void thinkpack_nfc_normalize_uid(const uint8_t *src, uint8_t src_len, uint8_t *dst,
                                 uint8_t *dst_len);

/** True if UIDs compare equal byte-for-byte (same length required). */
bool thinkpack_nfc_uid_equals(const uint8_t *a, uint8_t a_len, const uint8_t *b, uint8_t b_len);

/* ------------------------------------------------------------------ */
/* Registry operations                                                 */
/* ------------------------------------------------------------------ */

/** Reset @p reg to zero entries. */
void thinkpack_nfc_registry_init(thinkpack_nfc_registry_t *reg);

/**
 * @brief Look up an entry by UID.
 * @return Non-NULL pointer into @p reg if found, NULL otherwise.
 */
const thinkpack_nfc_entry_t *thinkpack_nfc_lookup(const thinkpack_nfc_registry_t *reg,
                                                  const uint8_t *uid, uint8_t uid_len);

/**
 * @brief Insert or replace an entry.
 *
 * If an entry with the same UID exists it is overwritten in place.
 * Otherwise the entry is appended; returns false if the registry is
 * already full.
 */
bool thinkpack_nfc_registry_upsert(thinkpack_nfc_registry_t *reg,
                                   const thinkpack_nfc_entry_t *entry);

/**
 * @brief Remove an entry by UID.
 * @return true if an entry was removed.
 */
bool thinkpack_nfc_registry_remove(thinkpack_nfc_registry_t *reg, const uint8_t *uid,
                                   uint8_t uid_len);

/* ------------------------------------------------------------------ */
/* NVS blob format                                                     */
/* ------------------------------------------------------------------ */

/** Exact size of a serialised registry in bytes. */
#define THINKPACK_NFC_BLOB_HEADER_SIZE 3
#define THINKPACK_NFC_BLOB_ENTRY_SIZE \
    (1 + THINKPACK_NFC_UID_MAX_LEN + 1 + 1 + THINKPACK_NFC_LABEL_MAX_LEN)
#define THINKPACK_NFC_BLOB_MAX_SIZE \
    (THINKPACK_NFC_BLOB_HEADER_SIZE + (THINKPACK_NFC_MAX_TAGS * THINKPACK_NFC_BLOB_ENTRY_SIZE))

/**
 * @brief Serialise @p reg to a flat byte buffer.
 *
 * Layout: [magic][version][count] then count×entry.
 * Each entry: [uid_len][uid[7]][behavior][param][label[24]].
 *
 * @return Number of bytes written, or 0 if @p buf is NULL or too small.
 */
size_t thinkpack_nfc_registry_serialize(const thinkpack_nfc_registry_t *reg, uint8_t *buf,
                                        size_t buf_len);

/**
 * @brief Deserialise a blob into @p reg.
 *
 * Validates magic, version, and declared entry count.
 *
 * @return true on success, false on format error.
 */
bool thinkpack_nfc_registry_deserialize(thinkpack_nfc_registry_t *reg, const uint8_t *buf,
                                        size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* THINKPACK_NFC_H */
