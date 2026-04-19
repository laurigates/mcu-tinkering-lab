/**
 * @file test_nfc.c
 * @brief Host-based unit tests for thinkpack-nfc.
 */

#include <string.h>

#include "thinkpack_nfc.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* UID helpers                                                         */
/* ------------------------------------------------------------------ */

static void test_normalize_short_uid_zero_pads(void)
{
    const uint8_t in[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t out[THINKPACK_NFC_UID_MAX_LEN];
    uint8_t out_len = 0xFF;
    thinkpack_nfc_normalize_uid(in, 4, out, &out_len);

    TEST_ASSERT_EQUAL(4, out_len);
    TEST_ASSERT_EQUAL(0xAA, out[0]);
    TEST_ASSERT_EQUAL(0xBB, out[1]);
    TEST_ASSERT_EQUAL(0xCC, out[2]);
    TEST_ASSERT_EQUAL(0xDD, out[3]);
    TEST_ASSERT_EQUAL(0x00, out[4]);
    TEST_ASSERT_EQUAL(0x00, out[5]);
    TEST_ASSERT_EQUAL(0x00, out[6]);
}

static void test_normalize_clamps_oversize_uid(void)
{
    const uint8_t in[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint8_t out[THINKPACK_NFC_UID_MAX_LEN];
    uint8_t out_len = 0;
    thinkpack_nfc_normalize_uid(in, 10, out, &out_len);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_UID_MAX_LEN, out_len);
    TEST_ASSERT_EQUAL(1, out[0]);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_UID_MAX_LEN, out[THINKPACK_NFC_UID_MAX_LEN - 1]);
}

static void test_uid_equals(void)
{
    const uint8_t a[] = {1, 2, 3, 4};
    const uint8_t b[] = {1, 2, 3, 4};
    const uint8_t c[] = {1, 2, 3, 5};
    TEST_ASSERT_TRUE(thinkpack_nfc_uid_equals(a, 4, b, 4));
    TEST_ASSERT_FALSE(thinkpack_nfc_uid_equals(a, 4, c, 4));
    TEST_ASSERT_FALSE(thinkpack_nfc_uid_equals(a, 4, b, 3)); /* length mismatch */
}

/* ------------------------------------------------------------------ */
/* Registry                                                            */
/* ------------------------------------------------------------------ */

static thinkpack_nfc_entry_t make_entry(uint8_t seed, const char *label, thinkpack_nfc_behavior_t b,
                                        uint8_t param)
{
    thinkpack_nfc_entry_t e;
    memset(&e, 0, sizeof(e));
    for (uint8_t i = 0; i < THINKPACK_NFC_UID_MAX_LEN; i++) {
        e.uid[i] = seed + i;
    }
    e.uid_len = THINKPACK_NFC_UID_MAX_LEN;
    e.behavior = (uint8_t)b;
    e.param = param;
    strncpy(e.label, label, THINKPACK_NFC_LABEL_MAX_LEN - 1);
    return e;
}

static void test_registry_empty_lookup_returns_null(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);
    const uint8_t uid[] = {1, 2, 3, 4, 5, 6, 7};
    TEST_ASSERT_NULL(thinkpack_nfc_lookup(&reg, uid, 7));
}

static void test_registry_upsert_and_lookup(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);

    thinkpack_nfc_entry_t e = make_entry(0x10, "dinosaur", THINKPACK_NFC_BEHAVIOR_STORY, 3);
    TEST_ASSERT_TRUE(thinkpack_nfc_registry_upsert(&reg, &e));
    TEST_ASSERT_EQUAL(1, reg.count);

    const thinkpack_nfc_entry_t *got = thinkpack_nfc_lookup(&reg, e.uid, e.uid_len);
    TEST_ASSERT_NOT_NULL(got);
    TEST_ASSERT_EQUAL_STRING("dinosaur", got->label);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BEHAVIOR_STORY, got->behavior);
    TEST_ASSERT_EQUAL(3, got->param);
}

static void test_registry_upsert_overwrites_existing(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);

    thinkpack_nfc_entry_t a = make_entry(0x20, "dino", THINKPACK_NFC_BEHAVIOR_CHIME, 10);
    thinkpack_nfc_entry_t b = make_entry(0x20, "t-rex", THINKPACK_NFC_BEHAVIOR_STORY, 99);
    TEST_ASSERT_TRUE(thinkpack_nfc_registry_upsert(&reg, &a));
    TEST_ASSERT_TRUE(thinkpack_nfc_registry_upsert(&reg, &b));
    TEST_ASSERT_EQUAL(1, reg.count);

    const thinkpack_nfc_entry_t *got = thinkpack_nfc_lookup(&reg, a.uid, a.uid_len);
    TEST_ASSERT_NOT_NULL(got);
    TEST_ASSERT_EQUAL_STRING("t-rex", got->label);
    TEST_ASSERT_EQUAL(99, got->param);
}

static void test_registry_upsert_rejects_when_full(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);

    for (int i = 0; i < THINKPACK_NFC_MAX_TAGS; i++) {
        thinkpack_nfc_entry_t e =
            make_entry((uint8_t)(0x30 + i), "x", THINKPACK_NFC_BEHAVIOR_CHIME, (uint8_t)i);
        TEST_ASSERT_TRUE(thinkpack_nfc_registry_upsert(&reg, &e));
    }
    thinkpack_nfc_entry_t extra = make_entry(0xFE, "extra", THINKPACK_NFC_BEHAVIOR_STORY, 0);
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_upsert(&reg, &extra));
    TEST_ASSERT_EQUAL(THINKPACK_NFC_MAX_TAGS, reg.count);
}

static void test_registry_remove(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);

    thinkpack_nfc_entry_t a = make_entry(0x40, "a", THINKPACK_NFC_BEHAVIOR_CHIME, 1);
    thinkpack_nfc_entry_t b = make_entry(0x50, "b", THINKPACK_NFC_BEHAVIOR_STORY, 2);
    thinkpack_nfc_entry_t c = make_entry(0x60, "c", THINKPACK_NFC_BEHAVIOR_COLOR, 3);
    thinkpack_nfc_registry_upsert(&reg, &a);
    thinkpack_nfc_registry_upsert(&reg, &b);
    thinkpack_nfc_registry_upsert(&reg, &c);

    TEST_ASSERT_TRUE(thinkpack_nfc_registry_remove(&reg, b.uid, b.uid_len));
    TEST_ASSERT_EQUAL(2, reg.count);
    TEST_ASSERT_NULL(thinkpack_nfc_lookup(&reg, b.uid, b.uid_len));
    /* a and c must survive */
    TEST_ASSERT_NOT_NULL(thinkpack_nfc_lookup(&reg, a.uid, a.uid_len));
    TEST_ASSERT_NOT_NULL(thinkpack_nfc_lookup(&reg, c.uid, c.uid_len));

    /* Removing missing UID returns false */
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_remove(&reg, b.uid, b.uid_len));
}

/* ------------------------------------------------------------------ */
/* NVS blob round-trip                                                 */
/* ------------------------------------------------------------------ */

static void test_blob_round_trip(void)
{
    thinkpack_nfc_registry_t src;
    thinkpack_nfc_registry_init(&src);

    thinkpack_nfc_entry_t a = make_entry(0x70, "red-bird", THINKPACK_NFC_BEHAVIOR_CHIME, 200);
    thinkpack_nfc_entry_t b = make_entry(0x80, "purple-whale", THINKPACK_NFC_BEHAVIOR_STORY, 5);
    thinkpack_nfc_registry_upsert(&src, &a);
    thinkpack_nfc_registry_upsert(&src, &b);

    uint8_t buf[THINKPACK_NFC_BLOB_MAX_SIZE];
    size_t n = thinkpack_nfc_registry_serialize(&src, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BLOB_HEADER_SIZE + 2 * THINKPACK_NFC_BLOB_ENTRY_SIZE, n);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BLOB_MAGIC, buf[0]);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BLOB_VERSION, buf[1]);
    TEST_ASSERT_EQUAL(2, buf[2]);

    thinkpack_nfc_registry_t dst;
    TEST_ASSERT_TRUE(thinkpack_nfc_registry_deserialize(&dst, buf, n));
    TEST_ASSERT_EQUAL(2, dst.count);

    const thinkpack_nfc_entry_t *got_a = thinkpack_nfc_lookup(&dst, a.uid, a.uid_len);
    TEST_ASSERT_NOT_NULL(got_a);
    TEST_ASSERT_EQUAL_STRING("red-bird", got_a->label);
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BEHAVIOR_CHIME, got_a->behavior);
    TEST_ASSERT_EQUAL(200, got_a->param);

    const thinkpack_nfc_entry_t *got_b = thinkpack_nfc_lookup(&dst, b.uid, b.uid_len);
    TEST_ASSERT_NOT_NULL(got_b);
    TEST_ASSERT_EQUAL_STRING("purple-whale", got_b->label);
}

static void test_blob_serialize_rejects_small_buffer(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);
    thinkpack_nfc_entry_t e = make_entry(0x90, "x", THINKPACK_NFC_BEHAVIOR_NONE, 0);
    thinkpack_nfc_registry_upsert(&reg, &e);

    uint8_t buf[4]; /* too small */
    TEST_ASSERT_EQUAL(0, thinkpack_nfc_registry_serialize(&reg, buf, sizeof(buf)));
}

static void test_blob_deserialize_rejects_bad_magic(void)
{
    uint8_t buf[THINKPACK_NFC_BLOB_HEADER_SIZE] = {0x00, THINKPACK_NFC_BLOB_VERSION, 0};
    thinkpack_nfc_registry_t dst;
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_deserialize(&dst, buf, sizeof(buf)));
}

static void test_blob_deserialize_rejects_bad_version(void)
{
    uint8_t buf[THINKPACK_NFC_BLOB_HEADER_SIZE] = {THINKPACK_NFC_BLOB_MAGIC, 0xFF, 0};
    thinkpack_nfc_registry_t dst;
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_deserialize(&dst, buf, sizeof(buf)));
}

static void test_blob_deserialize_rejects_oversize_count(void)
{
    uint8_t buf[THINKPACK_NFC_BLOB_HEADER_SIZE] = {
        THINKPACK_NFC_BLOB_MAGIC, THINKPACK_NFC_BLOB_VERSION, THINKPACK_NFC_MAX_TAGS + 1};
    thinkpack_nfc_registry_t dst;
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_deserialize(&dst, buf, sizeof(buf)));
}

static void test_blob_deserialize_rejects_truncated(void)
{
    thinkpack_nfc_registry_t reg;
    thinkpack_nfc_registry_init(&reg);
    thinkpack_nfc_entry_t a = make_entry(0xA0, "a", THINKPACK_NFC_BEHAVIOR_CHIME, 1);
    thinkpack_nfc_entry_t b = make_entry(0xB0, "b", THINKPACK_NFC_BEHAVIOR_STORY, 2);
    thinkpack_nfc_registry_upsert(&reg, &a);
    thinkpack_nfc_registry_upsert(&reg, &b);

    uint8_t buf[THINKPACK_NFC_BLOB_MAX_SIZE];
    size_t n = thinkpack_nfc_registry_serialize(&reg, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);

    thinkpack_nfc_registry_t dst;
    /* Cut the last entry off */
    TEST_ASSERT_FALSE(thinkpack_nfc_registry_deserialize(&dst, buf, n - 5));
}

static void test_blob_empty_registry_round_trip(void)
{
    thinkpack_nfc_registry_t src;
    thinkpack_nfc_registry_init(&src);

    uint8_t buf[THINKPACK_NFC_BLOB_MAX_SIZE];
    size_t n = thinkpack_nfc_registry_serialize(&src, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(THINKPACK_NFC_BLOB_HEADER_SIZE, n);

    thinkpack_nfc_registry_t dst;
    TEST_ASSERT_TRUE(thinkpack_nfc_registry_deserialize(&dst, buf, n));
    TEST_ASSERT_EQUAL(0, dst.count);
}

/* ------------------------------------------------------------------ */
/* Test runner                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_normalize_short_uid_zero_pads);
    RUN_TEST(test_normalize_clamps_oversize_uid);
    RUN_TEST(test_uid_equals);

    RUN_TEST(test_registry_empty_lookup_returns_null);
    RUN_TEST(test_registry_upsert_and_lookup);
    RUN_TEST(test_registry_upsert_overwrites_existing);
    RUN_TEST(test_registry_upsert_rejects_when_full);
    RUN_TEST(test_registry_remove);

    RUN_TEST(test_blob_round_trip);
    RUN_TEST(test_blob_serialize_rejects_small_buffer);
    RUN_TEST(test_blob_deserialize_rejects_bad_magic);
    RUN_TEST(test_blob_deserialize_rejects_bad_version);
    RUN_TEST(test_blob_deserialize_rejects_oversize_count);
    RUN_TEST(test_blob_deserialize_rejects_truncated);
    RUN_TEST(test_blob_empty_registry_round_trip);

    int failures = UNITY_END();
    return failures == 0 ? 0 : 1;
}
