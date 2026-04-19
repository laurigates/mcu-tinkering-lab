/**
 * @file test_chunk_state.c
 * @brief Host tests for the OTA chunk-assembly state machine.
 */

#include <stdlib.h>
#include <string.h>

#include "thinkpack_ota.h"
#include "thinkpack_protocol.h"
#include "unity_compat.h"

#define IMG_CHUNKS 4
#define IMG_TAIL 17
#define IMG_SIZE ((IMG_CHUNKS - 1) * THINKPACK_OTA_CHUNK_DATA_MAX + IMG_TAIL)
#define BITMAP_WORDS ((IMG_CHUNKS + 31) / 32)

static uint8_t g_image[IMG_SIZE];
static uint32_t g_bitmap[BITMAP_WORDS];
static uint8_t g_reconstructed[IMG_SIZE];

static void fill_reference_image(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        buf[i] = (uint8_t)(i * 7 + 13);
    }
}

static void make_chunk(ota_chunk_payload_t *c, uint16_t idx, const uint8_t *src, size_t total)
{
    memset(c, 0, sizeof(*c));
    c->chunk_index = idx;
    size_t offset = (size_t)idx * THINKPACK_OTA_CHUNK_DATA_MAX;
    size_t remain = total - offset;
    size_t take = remain > THINKPACK_OTA_CHUNK_DATA_MAX ? THINKPACK_OTA_CHUNK_DATA_MAX : remain;
    c->data_len = (uint8_t)take;
    memcpy(c->data, src + offset, take);
}

static void make_manifest(ota_manifest_payload_t *m, const uint8_t *ref, size_t size,
                          uint16_t count)
{
    memset(m, 0, sizeof(*m));
    m->version = 7;
    m->total_size = (uint32_t)size;
    m->chunk_count = count;
    m->target_box_mask = THINKPACK_OTA_TARGET_ALL;
    thinkpack_ota_sha256(ref, size, m->sha256);
}

static void reset_state(void)
{
    memset(g_image, 0, sizeof(g_image));
    memset(g_bitmap, 0, sizeof(g_bitmap));
    memset(g_reconstructed, 0, sizeof(g_reconstructed));
}

static void test_in_order_complete(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));

    for (uint16_t i = 0; i < IMG_CHUNKS; ++i) {
        ota_chunk_payload_t c;
        make_chunk(&c, i, g_reconstructed, IMG_SIZE);
        TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_update(&s, &c));
    }

    TEST_ASSERT_TRUE(thinkpack_ota_chunk_state_is_complete(&s));
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_finalize(&s));
    TEST_ASSERT_EQUAL_MEMORY(g_reconstructed, g_image, IMG_SIZE);
}

static void test_out_of_order_and_duplicates(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));

    uint16_t order[IMG_CHUNKS] = {3, 1, 0, 2};
    for (int i = 0; i < IMG_CHUNKS; ++i) {
        ota_chunk_payload_t c;
        make_chunk(&c, order[i], g_reconstructed, IMG_SIZE);
        TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_update(&s, &c));
    }

    /* Resend chunk 2; state machine should silently accept. */
    ota_chunk_payload_t dup;
    make_chunk(&dup, 2, g_reconstructed, IMG_SIZE);
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_update(&s, &dup));
    TEST_ASSERT_EQUAL(IMG_CHUNKS, s.chunks_received);

    TEST_ASSERT_TRUE(thinkpack_ota_chunk_state_is_complete(&s));
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_finalize(&s));
}

static void test_missing_chunk_fails_finalize(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));

    for (uint16_t i = 0; i < IMG_CHUNKS - 1; ++i) {
        ota_chunk_payload_t c;
        make_chunk(&c, i, g_reconstructed, IMG_SIZE);
        TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_update(&s, &c));
    }

    TEST_ASSERT_FALSE(thinkpack_ota_chunk_state_is_complete(&s));
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_CHUNK_MISSING, thinkpack_ota_chunk_state_finalize(&s));
}

static void test_out_of_range_chunk_rejected(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));

    ota_chunk_payload_t c;
    make_chunk(&c, 0, g_reconstructed, IMG_SIZE);
    c.chunk_index = IMG_CHUNKS + 2;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_CHUNK_OVERFLOW, thinkpack_ota_chunk_state_update(&s, &c));
}

static void test_sha_mismatch_detected(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);
    /* Corrupt expected SHA. */
    m.sha256[0] ^= 0xff;

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));
    for (uint16_t i = 0; i < IMG_CHUNKS; ++i) {
        ota_chunk_payload_t c;
        make_chunk(&c, i, g_reconstructed, IMG_SIZE);
        TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_chunk_state_update(&s, &c));
    }
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_SHA_MISMATCH, thinkpack_ota_chunk_state_finalize(&s));
}

static void test_mid_chunk_wrong_size_rejected(void)
{
    reset_state();
    fill_reference_image(g_reconstructed, IMG_SIZE);

    ota_manifest_payload_t m;
    make_manifest(&m, g_reconstructed, IMG_SIZE, IMG_CHUNKS);

    thinkpack_ota_chunk_state_t s;
    TEST_ASSERT_EQUAL(
        ESP_OK, thinkpack_ota_chunk_state_init(&s, &m, g_image, IMG_SIZE, g_bitmap, BITMAP_WORDS));

    /* Mid-image chunk claiming a partial length. */
    ota_chunk_payload_t c;
    make_chunk(&c, 1, g_reconstructed, IMG_SIZE);
    c.data_len = 42;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_SIZE_MISMATCH, thinkpack_ota_chunk_state_update(&s, &c));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_in_order_complete);
    RUN_TEST(test_out_of_order_and_duplicates);
    RUN_TEST(test_missing_chunk_fails_finalize);
    RUN_TEST(test_out_of_range_chunk_rejected);
    RUN_TEST(test_sha_mismatch_detected);
    RUN_TEST(test_mid_chunk_wrong_size_rejected);
    return UNITY_END();
}
