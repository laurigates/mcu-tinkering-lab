/**
 * @file manifest_parser.c
 * @brief Pure-logic validation of ota_manifest_payload_t.
 */

#include "thinkpack_ota.h"

thinkpack_ota_error_t thinkpack_ota_manifest_validate(const ota_manifest_payload_t *m)
{
    if (m == NULL) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    if (m->version == 0) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    if (m->total_size == 0) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    if (m->chunk_count == 0 || m->chunk_count > THINKPACK_OTA_MAX_CHUNKS) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    /* The image must fit inside chunk_count chunks. */
    size_t max_bytes = (size_t)m->chunk_count * (size_t)THINKPACK_OTA_CHUNK_DATA_MAX;
    if ((size_t)m->total_size > max_bytes) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    /* Last-chunk lower bound: (chunk_count - 1) chunks must be strictly
     * less than total_size (i.e. at least one byte in the final chunk). */
    size_t without_last = (size_t)(m->chunk_count - 1) * (size_t)THINKPACK_OTA_CHUNK_DATA_MAX;
    if (without_last >= (size_t)m->total_size) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    if (m->reserved != 0) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    return THINKPACK_OTA_ERR_NONE;
}

bool thinkpack_ota_manifest_targets_box(uint8_t target_mask, uint8_t my_box_type)
{
    if (target_mask == THINKPACK_OTA_TARGET_ALL) {
        return true;
    }
    if (my_box_type == 0 || my_box_type >= 8) {
        /* Only box types 1..7 fit in the 8-bit bitmask; unknown types
         * default to NOT targeted. */
        return false;
    }
    return (target_mask & (uint8_t)(1u << my_box_type)) != 0;
}
