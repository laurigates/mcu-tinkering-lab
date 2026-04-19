/**
 * @file chunk_state.c
 * @brief Pure-logic assembly of MSG_OTA_CHUNK packets into an image.
 */

#include <string.h>

#include "thinkpack_ota.h"

static size_t words_for_chunks(uint16_t chunk_count)
{
    return (size_t)(((uint32_t)chunk_count + 31u) / 32u);
}

static bool bitmap_test(const uint32_t *bm, uint16_t idx)
{
    return (bm[idx / 32u] & (1u << (idx % 32u))) != 0;
}

static void bitmap_set(uint32_t *bm, uint16_t idx)
{
    bm[idx / 32u] |= (1u << (idx % 32u));
}

esp_err_t thinkpack_ota_chunk_state_init(thinkpack_ota_chunk_state_t *s,
                                         const ota_manifest_payload_t *m, uint8_t *image_buf,
                                         size_t image_buf_size, uint32_t *bitmap_buf,
                                         size_t bitmap_buf_words)
{
    if (s == NULL || m == NULL || image_buf == NULL || bitmap_buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (image_buf_size < (size_t)m->total_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t need_words = words_for_chunks(m->chunk_count);
    if (bitmap_buf_words < need_words) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(s, 0, sizeof(*s));
    s->version = m->version;
    s->total_size = m->total_size;
    s->chunk_count = m->chunk_count;
    s->chunks_received = 0;
    s->image = image_buf;
    s->received_bitmap = bitmap_buf;
    s->bitmap_words = need_words;
    memcpy(s->expected_sha, m->sha256, 32);
    memset(bitmap_buf, 0, need_words * sizeof(uint32_t));
    return ESP_OK;
}

thinkpack_ota_error_t thinkpack_ota_chunk_state_update(thinkpack_ota_chunk_state_t *s,
                                                       const ota_chunk_payload_t *c)
{
    if (s == NULL || c == NULL) {
        return THINKPACK_OTA_ERR_CHUNK_OVERFLOW;
    }
    if (c->chunk_index >= s->chunk_count) {
        return THINKPACK_OTA_ERR_CHUNK_OVERFLOW;
    }
    if (c->data_len == 0 || c->data_len > THINKPACK_OTA_CHUNK_DATA_MAX) {
        return THINKPACK_OTA_ERR_CHUNK_OVERFLOW;
    }

    size_t offset = (size_t)c->chunk_index * (size_t)THINKPACK_OTA_CHUNK_DATA_MAX;
    if (offset + c->data_len > s->total_size) {
        /* Either the final short chunk has been sized wrong, or a mid-
         * image chunk exceeded the max size. */
        return THINKPACK_OTA_ERR_SIZE_MISMATCH;
    }

    /* For non-final chunks the size must equal the max. */
    if (c->chunk_index < (uint16_t)(s->chunk_count - 1) &&
        c->data_len != THINKPACK_OTA_CHUNK_DATA_MAX) {
        return THINKPACK_OTA_ERR_SIZE_MISMATCH;
    }

    if (bitmap_test(s->received_bitmap, c->chunk_index)) {
        /* Duplicate — silently accept. */
        return THINKPACK_OTA_ERR_NONE;
    }

    memcpy(s->image + offset, c->data, c->data_len);
    bitmap_set(s->received_bitmap, c->chunk_index);
    s->chunks_received++;
    return THINKPACK_OTA_ERR_NONE;
}

bool thinkpack_ota_chunk_state_is_complete(const thinkpack_ota_chunk_state_t *s)
{
    if (s == NULL) {
        return false;
    }
    return s->chunks_received == s->chunk_count;
}

thinkpack_ota_error_t thinkpack_ota_chunk_state_finalize(const thinkpack_ota_chunk_state_t *s)
{
    if (s == NULL || s->image == NULL) {
        return THINKPACK_OTA_ERR_MANIFEST_INVALID;
    }
    if (!thinkpack_ota_chunk_state_is_complete(s)) {
        return THINKPACK_OTA_ERR_CHUNK_MISSING;
    }
    if (!thinkpack_ota_sha256_verify(s->image, s->total_size, s->expected_sha)) {
        return THINKPACK_OTA_ERR_SHA_MISMATCH;
    }
    return THINKPACK_OTA_ERR_NONE;
}
