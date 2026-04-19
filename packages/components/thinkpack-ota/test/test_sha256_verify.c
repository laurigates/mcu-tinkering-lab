/**
 * @file test_sha256_verify.c
 * @brief Known-answer tests for the host SHA256 wrapper.
 */

#include <string.h>

#include "thinkpack_ota.h"
#include "unity_compat.h"

/* Precomputed SHA256 digests from the FIPS 180-2 test vectors. */

static const uint8_t SHA256_EMPTY[32] = {
    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55};

/* SHA256("abc"). */
static const uint8_t SHA256_ABC[32] = {
    0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
    0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad};

static void test_sha256_empty(void)
{
    uint8_t out[32];
    thinkpack_ota_sha256(NULL, 0, out);
    TEST_ASSERT_EQUAL_MEMORY(SHA256_EMPTY, out, 32);
    TEST_ASSERT_TRUE(thinkpack_ota_sha256_verify(NULL, 0, SHA256_EMPTY));
}

static void test_sha256_abc(void)
{
    const uint8_t abc[] = {'a', 'b', 'c'};
    uint8_t out[32];
    thinkpack_ota_sha256(abc, sizeof(abc), out);
    TEST_ASSERT_EQUAL_MEMORY(SHA256_ABC, out, 32);
    TEST_ASSERT_TRUE(thinkpack_ota_sha256_verify(abc, sizeof(abc), SHA256_ABC));
}

static void test_sha256_mismatch_detected(void)
{
    const uint8_t abc[] = {'a', 'b', 'c'};
    /* Expected digest of "xyz", not "abc" — must fail. */
    uint8_t wrong[32];
    memcpy(wrong, SHA256_ABC, 32);
    wrong[0] ^= 0xff;
    TEST_ASSERT_FALSE(thinkpack_ota_sha256_verify(abc, sizeof(abc), wrong));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sha256_empty);
    RUN_TEST(test_sha256_abc);
    RUN_TEST(test_sha256_mismatch_detected);
    return UNITY_END();
}
