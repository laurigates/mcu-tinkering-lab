/**
 * @file test_manifest_parser.c
 * @brief Host tests for manifest validation + targeting logic.
 */

#include <string.h>

#include "thinkpack_ota.h"
#include "thinkpack_protocol.h"
#include "unity_compat.h"

static void make_valid_manifest(ota_manifest_payload_t *m)
{
    memset(m, 0, sizeof(*m));
    m->version = 2;
    m->total_size = 2 * THINKPACK_OTA_CHUNK_DATA_MAX + 17; /* 3 chunks */
    m->chunk_count = 3;
    m->target_box_mask = THINKPACK_OTA_TARGET_ALL;
    /* sha256 left as zeros — validator does not inspect it. */
}

static void test_happy_path(void)
{
    ota_manifest_payload_t m;
    make_valid_manifest(&m);
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_NONE, thinkpack_ota_manifest_validate(&m));
}

static void test_zero_version_rejected(void)
{
    ota_manifest_payload_t m;
    make_valid_manifest(&m);
    m.version = 0;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_MANIFEST_INVALID, thinkpack_ota_manifest_validate(&m));
}

static void test_oversized_total_rejected(void)
{
    ota_manifest_payload_t m;
    make_valid_manifest(&m);
    /* Claim more bytes than 3 × 180. */
    m.total_size = 4 * THINKPACK_OTA_CHUNK_DATA_MAX;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_MANIFEST_INVALID, thinkpack_ota_manifest_validate(&m));
}

static void test_chunk_count_cap(void)
{
    ota_manifest_payload_t m;
    make_valid_manifest(&m);
    m.chunk_count = THINKPACK_OTA_MAX_CHUNKS + 1;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_MANIFEST_INVALID, thinkpack_ota_manifest_validate(&m));
}

static void test_reserved_byte_must_be_zero(void)
{
    ota_manifest_payload_t m;
    make_valid_manifest(&m);
    m.reserved = 1;
    TEST_ASSERT_EQUAL(THINKPACK_OTA_ERR_MANIFEST_INVALID, thinkpack_ota_manifest_validate(&m));
}

static void test_targets_all_covers_unknown_box(void)
{
    TEST_ASSERT_TRUE(thinkpack_ota_manifest_targets_box(THINKPACK_OTA_TARGET_ALL, BOX_BOOMBOX));
    TEST_ASSERT_TRUE(thinkpack_ota_manifest_targets_box(THINKPACK_OTA_TARGET_ALL, BOX_UNKNOWN));
}

static void test_targets_specific_box(void)
{
    uint8_t mask = (uint8_t)(1u << BOX_GLOWBUG);
    TEST_ASSERT_TRUE(thinkpack_ota_manifest_targets_box(mask, BOX_GLOWBUG));
    TEST_ASSERT_FALSE(thinkpack_ota_manifest_targets_box(mask, BOX_FINDERBOX));
    TEST_ASSERT_FALSE(thinkpack_ota_manifest_targets_box(mask, BOX_UNKNOWN));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_happy_path);
    RUN_TEST(test_zero_version_rejected);
    RUN_TEST(test_oversized_total_rejected);
    RUN_TEST(test_chunk_count_cap);
    RUN_TEST(test_reserved_byte_must_be_zero);
    RUN_TEST(test_targets_all_covers_unknown_box);
    RUN_TEST(test_targets_specific_box);
    return UNITY_END();
}
