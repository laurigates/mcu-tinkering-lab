/**
 * @file thinkpack_nfc.c
 * @brief Implementation of the thinkpack-nfc pure-logic component.
 */

#include "thinkpack_nfc.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* UID helpers                                                         */
/* ------------------------------------------------------------------ */

void thinkpack_nfc_normalize_uid(const uint8_t *src, uint8_t src_len, uint8_t *dst,
                                 uint8_t *dst_len)
{
    if (dst == NULL || dst_len == NULL) {
        return;
    }

    uint8_t n = (src_len > THINKPACK_NFC_UID_MAX_LEN) ? THINKPACK_NFC_UID_MAX_LEN : src_len;
    memset(dst, 0, THINKPACK_NFC_UID_MAX_LEN);
    if (src != NULL && n > 0) {
        memcpy(dst, src, n);
    }
    *dst_len = n;
}

bool thinkpack_nfc_uid_equals(const uint8_t *a, uint8_t a_len, const uint8_t *b, uint8_t b_len)
{
    if (a_len != b_len) {
        return false;
    }
    if (a == NULL || b == NULL) {
        return false;
    }
    return memcmp(a, b, a_len) == 0;
}

/* ------------------------------------------------------------------ */
/* Registry operations                                                 */
/* ------------------------------------------------------------------ */

void thinkpack_nfc_registry_init(thinkpack_nfc_registry_t *reg)
{
    memset(reg, 0, sizeof(*reg));
}

const thinkpack_nfc_entry_t *thinkpack_nfc_lookup(const thinkpack_nfc_registry_t *reg,
                                                  const uint8_t *uid, uint8_t uid_len)
{
    if (reg == NULL || uid == NULL) {
        return NULL;
    }
    for (uint8_t i = 0; i < reg->count; i++) {
        const thinkpack_nfc_entry_t *e = &reg->entries[i];
        if (thinkpack_nfc_uid_equals(e->uid, e->uid_len, uid, uid_len)) {
            return e;
        }
    }
    return NULL;
}

/** Find slot index for a UID, or -1 if not present. */
static int find_slot(const thinkpack_nfc_registry_t *reg, const uint8_t *uid, uint8_t uid_len)
{
    for (uint8_t i = 0; i < reg->count; i++) {
        const thinkpack_nfc_entry_t *e = &reg->entries[i];
        if (thinkpack_nfc_uid_equals(e->uid, e->uid_len, uid, uid_len)) {
            return (int)i;
        }
    }
    return -1;
}

bool thinkpack_nfc_registry_upsert(thinkpack_nfc_registry_t *reg,
                                   const thinkpack_nfc_entry_t *entry)
{
    if (reg == NULL || entry == NULL) {
        return false;
    }
    int slot = find_slot(reg, entry->uid, entry->uid_len);
    if (slot >= 0) {
        reg->entries[slot] = *entry;
        return true;
    }
    if (reg->count >= THINKPACK_NFC_MAX_TAGS) {
        return false;
    }
    reg->entries[reg->count++] = *entry;
    return true;
}

bool thinkpack_nfc_registry_remove(thinkpack_nfc_registry_t *reg, const uint8_t *uid,
                                   uint8_t uid_len)
{
    if (reg == NULL) {
        return false;
    }
    int slot = find_slot(reg, uid, uid_len);
    if (slot < 0) {
        return false;
    }
    /* Swap-with-last to keep the registry compact. */
    reg->count--;
    if ((uint8_t)slot != reg->count) {
        reg->entries[slot] = reg->entries[reg->count];
    }
    memset(&reg->entries[reg->count], 0, sizeof(reg->entries[0]));
    return true;
}

/* ------------------------------------------------------------------ */
/* NVS blob format                                                     */
/* ------------------------------------------------------------------ */

size_t thinkpack_nfc_registry_serialize(const thinkpack_nfc_registry_t *reg, uint8_t *buf,
                                        size_t buf_len)
{
    if (reg == NULL || buf == NULL) {
        return 0;
    }
    size_t need =
        THINKPACK_NFC_BLOB_HEADER_SIZE + (size_t)reg->count * THINKPACK_NFC_BLOB_ENTRY_SIZE;
    if (buf_len < need) {
        return 0;
    }

    size_t offset = 0;
    buf[offset++] = THINKPACK_NFC_BLOB_MAGIC;
    buf[offset++] = THINKPACK_NFC_BLOB_VERSION;
    buf[offset++] = reg->count;

    for (uint8_t i = 0; i < reg->count; i++) {
        const thinkpack_nfc_entry_t *e = &reg->entries[i];
        buf[offset++] = e->uid_len;
        memcpy(&buf[offset], e->uid, THINKPACK_NFC_UID_MAX_LEN);
        offset += THINKPACK_NFC_UID_MAX_LEN;
        buf[offset++] = e->behavior;
        buf[offset++] = e->param;
        memcpy(&buf[offset], e->label, THINKPACK_NFC_LABEL_MAX_LEN);
        offset += THINKPACK_NFC_LABEL_MAX_LEN;
    }
    return offset;
}

bool thinkpack_nfc_registry_deserialize(thinkpack_nfc_registry_t *reg, const uint8_t *buf,
                                        size_t buf_len)
{
    if (reg == NULL || buf == NULL) {
        return false;
    }
    if (buf_len < THINKPACK_NFC_BLOB_HEADER_SIZE) {
        return false;
    }
    if (buf[0] != THINKPACK_NFC_BLOB_MAGIC) {
        return false;
    }
    if (buf[1] != THINKPACK_NFC_BLOB_VERSION) {
        return false;
    }
    uint8_t count = buf[2];
    if (count > THINKPACK_NFC_MAX_TAGS) {
        return false;
    }
    size_t need = THINKPACK_NFC_BLOB_HEADER_SIZE + (size_t)count * THINKPACK_NFC_BLOB_ENTRY_SIZE;
    if (buf_len < need) {
        return false;
    }

    thinkpack_nfc_registry_init(reg);
    size_t offset = THINKPACK_NFC_BLOB_HEADER_SIZE;
    for (uint8_t i = 0; i < count; i++) {
        thinkpack_nfc_entry_t *e = &reg->entries[i];
        e->uid_len = buf[offset++];
        memcpy(e->uid, &buf[offset], THINKPACK_NFC_UID_MAX_LEN);
        offset += THINKPACK_NFC_UID_MAX_LEN;
        e->behavior = buf[offset++];
        e->param = buf[offset++];
        memcpy(e->label, &buf[offset], THINKPACK_NFC_LABEL_MAX_LEN);
        offset += THINKPACK_NFC_LABEL_MAX_LEN;
        /* Guard against UIDs longer than MAX (corrupt blob). */
        if (e->uid_len > THINKPACK_NFC_UID_MAX_LEN) {
            return false;
        }
    }
    reg->count = count;
    return true;
}
