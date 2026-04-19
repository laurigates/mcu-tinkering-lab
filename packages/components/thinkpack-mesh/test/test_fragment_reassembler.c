/**
 * @file test_fragment_reassembler.c
 * @brief Host-based unit tests for fragment_reassembler.c.
 *
 * Tests pure reassembly logic: slot lookup, ordering, duplicates, concurrent
 * messages, cache eviction, pruning, and error handling.  No hardware required.
 *
 * Build and run: make test
 */

#include <string.h>

#include "fragment_reassembler.h"
#include "thinkpack_protocol.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Test fixtures                                                       */
/* ------------------------------------------------------------------ */

static const uint8_t MAC_A[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x01};
static const uint8_t MAC_B[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x02};

/** Fill a fragment with predictable content for a given (msg_id, index). */
static void make_fragment(thinkpack_fragment_data_t *f, uint8_t msg_id, uint8_t index,
                          uint8_t total, uint8_t original_type, uint8_t data_byte)
{
    memset(f, 0, sizeof(*f));
    f->msg_id = msg_id;
    f->fragment_index = index;
    f->total_fragments = total;
    f->original_msg_type = original_type;
    f->data_length = 4;
    memset(f->data, data_byte, 4);
}

/* ------------------------------------------------------------------ */
/* test_single_fragment_is_incomplete                                  */
/* ------------------------------------------------------------------ */

static void test_single_fragment_is_incomplete(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 1, 0, 3, MSG_LLM_RESPONSE, 0xAA);

    reassembly_slot_t *completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, MAC_A, &f, 1000, &completed);

    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r);
    TEST_ASSERT_NULL(completed);
}

/* ------------------------------------------------------------------ */
/* test_all_fragments_in_order_complete                                */
/* ------------------------------------------------------------------ */

static void test_all_fragments_in_order_complete(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    const uint8_t total = 3;
    reassembly_slot_t *completed = NULL;
    reassemble_result_t r;

    for (uint8_t i = 0; i < total - 1; i++) {
        thinkpack_fragment_data_t f;
        make_fragment(&f, 10, i, total, MSG_LLM_RESPONSE, (uint8_t)(0x10 + i));
        completed = NULL;
        r = reassembler_absorb(cache, MAC_A, &f, 1000 + i, &completed);
        TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r);
        TEST_ASSERT_NULL(completed);
    }

    /* Last fragment — should complete. */
    thinkpack_fragment_data_t last;
    make_fragment(&last, 10, total - 1, total, MSG_LLM_RESPONSE, 0x1F);
    completed = NULL;
    r = reassembler_absorb(cache, MAC_A, &last, 1002, &completed);

    TEST_ASSERT_EQUAL(REASSEMBLE_COMPLETE, r);
    TEST_ASSERT_NOT_NULL(completed);
    TEST_ASSERT_EQUAL(MSG_LLM_RESPONSE, completed->original_msg_type);
    /* 3 fragments × 4 bytes each = 12 bytes. */
    TEST_ASSERT_EQUAL(12, (int)completed->reassembled_len);

    reassembler_release(completed);
    /* Slot should be free now. */
    TEST_ASSERT_EQUAL(0, (int)completed->total_fragments);
}

/* ------------------------------------------------------------------ */
/* test_out_of_order_complete                                          */
/* ------------------------------------------------------------------ */

static void test_out_of_order_complete(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    /* Send fragments 2, 0, 1 — should complete after fragment 1. */
    const uint8_t total = 3;
    reassembly_slot_t *completed = NULL;
    reassemble_result_t r;

    uint8_t order[] = {2, 0, 1};
    for (int step = 0; step < 3; step++) {
        thinkpack_fragment_data_t f;
        make_fragment(&f, 20, order[step], total, MSG_LLM_REQUEST, (uint8_t)(0x20 + order[step]));
        completed = NULL;
        r = reassembler_absorb(cache, MAC_A, &f, 2000 + (uint32_t)step, &completed);
        if (step < 2) {
            TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r);
            TEST_ASSERT_NULL(completed);
        }
    }

    TEST_ASSERT_EQUAL(REASSEMBLE_COMPLETE, r);
    TEST_ASSERT_NOT_NULL(completed);
    TEST_ASSERT_EQUAL(12, (int)completed->reassembled_len);

    /* Verify data was placed at the right offsets — data_byte was 0x20 + fragment_index,
     * so fragment 0 wrote 0x20, fragment 1 wrote 0x21, fragment 2 wrote 0x22. */
    TEST_ASSERT_EQUAL(0x20, completed->buffer[0]); /* fragment 0 data byte */
    TEST_ASSERT_EQUAL(0x21, completed->buffer[THINKPACK_MAX_FRAGMENT_DATA]);     /* frag 1 */
    TEST_ASSERT_EQUAL(0x22, completed->buffer[2 * THINKPACK_MAX_FRAGMENT_DATA]); /* frag 2 */

    reassembler_release(completed);
}

/* ------------------------------------------------------------------ */
/* test_duplicate_fragment_idempotent                                  */
/* ------------------------------------------------------------------ */

static void test_duplicate_fragment_idempotent(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 30, 0, 2, MSG_STATUS, 0xDD);

    reassembly_slot_t *completed = NULL;

    /* First absorption — incomplete. */
    reassemble_result_t r1 = reassembler_absorb(cache, MAC_A, &f, 3000, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r1);

    /* Duplicate — still incomplete, no state change. */
    reassemble_result_t r2 = reassembler_absorb(cache, MAC_A, &f, 3001, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r2);
    TEST_ASSERT_NULL(completed);

    /* The slot's received_mask should still have only bit 0 set. */
    TEST_ASSERT_EQUAL(1, (int)cache[0].received_mask);
}

/* ------------------------------------------------------------------ */
/* test_two_concurrent_messages_from_different_macs                   */
/* ------------------------------------------------------------------ */

static void test_two_concurrent_messages_from_different_macs(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    /* Both senders use the same msg_id — slots are keyed by (mac, msg_id). */
    const uint8_t total = 2;
    thinkpack_fragment_data_t fA0, fA1, fB0, fB1;
    make_fragment(&fA0, 5, 0, total, MSG_LLM_RESPONSE, 0xA0);
    make_fragment(&fA1, 5, 1, total, MSG_LLM_RESPONSE, 0xA1);
    make_fragment(&fB0, 5, 0, total, MSG_COMMAND, 0xB0);
    make_fragment(&fB1, 5, 1, total, MSG_COMMAND, 0xB1);

    reassembly_slot_t *completed = NULL;

    /* Interleave: A0, B0, A1, B1. */
    reassemble_result_t rA0 = reassembler_absorb(cache, MAC_A, &fA0, 100, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, rA0);

    reassemble_result_t rB0 = reassembler_absorb(cache, MAC_B, &fB0, 101, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, rB0);

    completed = NULL;
    reassemble_result_t rA1 = reassembler_absorb(cache, MAC_A, &fA1, 102, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_COMPLETE, rA1);
    TEST_ASSERT_NOT_NULL(completed);
    TEST_ASSERT_EQUAL(MSG_LLM_RESPONSE, completed->original_msg_type);
    reassembler_release(completed);

    completed = NULL;
    reassemble_result_t rB1 = reassembler_absorb(cache, MAC_B, &fB1, 103, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_COMPLETE, rB1);
    TEST_ASSERT_NOT_NULL(completed);
    TEST_ASSERT_EQUAL(MSG_COMMAND, completed->original_msg_type);
    reassembler_release(completed);
}

/* ------------------------------------------------------------------ */
/* test_cache_full_evicts_oldest                                       */
/* ------------------------------------------------------------------ */

static void test_cache_full_evicts_oldest(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    /* Fill all REASSEMBLY_CACHE_SIZE slots with 2-fragment messages. */
    /* Use distinct MACs so the keying is unambiguous. */
    uint8_t macs[REASSEMBLY_CACHE_SIZE + 1][6];
    for (int i = 0; i <= REASSEMBLY_CACHE_SIZE; i++) {
        memset(macs[i], 0, 6);
        macs[i][5] = (uint8_t)(i + 1);
    }

    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        thinkpack_fragment_data_t f;
        make_fragment(&f, (uint8_t)i, 0, 2, MSG_STATUS, (uint8_t)i);
        reassembly_slot_t *completed = NULL;
        reassemble_result_t r =
            reassembler_absorb(cache, macs[i], &f, (uint32_t)(1000 + i), /* oldest = slot 0 */
                               &completed);
        TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r);
    }

    /* Cache is now full.  Send a fragment for a brand-new (mac, msg_id). */
    thinkpack_fragment_data_t f_new;
    make_fragment(&f_new, 99, 0, 2, MSG_STATUS, 0xFF);
    reassembly_slot_t *completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, macs[REASSEMBLY_CACHE_SIZE], &f_new,
                                               9000, /* much later — slot 0 is oldest */
                                               &completed);
    /* Should be INCOMPLETE (not an error) — new slot was allocated by evicting oldest. */
    TEST_ASSERT_EQUAL(REASSEMBLE_INCOMPLETE, r);
    TEST_ASSERT_NULL(completed);

    /* Verify the new slot is in the cache with msg_id=99. */
    bool found = false;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == 99) {
            found = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(found);
}

/* ------------------------------------------------------------------ */
/* test_prune_stale_entries                                            */
/* ------------------------------------------------------------------ */

static void test_prune_stale_entries(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    /* Insert a fragment at t=0. */
    thinkpack_fragment_data_t f;
    make_fragment(&f, 42, 0, 3, MSG_STATUS, 0x55);
    reassembly_slot_t *completed = NULL;
    reassembler_absorb(cache, MAC_A, &f, 0, &completed);

    /* Prune with stale_ms=5000 at t=4999 — slot should survive. */
    reassembler_prune(cache, 4999, 5000);
    bool found_before = false;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == 42) {
            found_before = true;
        }
    }
    TEST_ASSERT_TRUE(found_before);

    /* Prune at t=5000 — slot should be freed. */
    reassembler_prune(cache, 5000, 5000);
    bool found_after = false;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == 42) {
            found_after = true;
        }
    }
    TEST_ASSERT_FALSE(found_after);
}

/* ------------------------------------------------------------------ */
/* test_total_fragments_mismatch_returns_error                         */
/* ------------------------------------------------------------------ */

static void test_total_fragments_mismatch_returns_error(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    /* Fragment 0 claims total=3. */
    thinkpack_fragment_data_t f0;
    make_fragment(&f0, 77, 0, 3, MSG_STATUS, 0x01);
    reassembly_slot_t *completed = NULL;
    reassembler_absorb(cache, MAC_A, &f0, 100, &completed);

    /* Fragment 1 claims total=2 — mismatch. */
    thinkpack_fragment_data_t f1;
    make_fragment(&f1, 77, 1, 2, MSG_STATUS, 0x02);
    completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, MAC_A, &f1, 101, &completed);

    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, r);
    TEST_ASSERT_NULL(completed);

    /* Slot should have been cleared. */
    bool found = false;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == 77) {
            found = true;
        }
    }
    TEST_ASSERT_FALSE(found);
}

/* ------------------------------------------------------------------ */
/* test_invalid_total_fragments_returns_error                          */
/* ------------------------------------------------------------------ */

static void test_invalid_total_fragments_returns_error(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    memset(&f, 0, sizeof(f));
    f.msg_id = 1;
    f.fragment_index = 0;
    f.total_fragments = 0; /* invalid */
    f.data_length = 1;

    reassembly_slot_t *completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, MAC_A, &f, 0, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, r);
}

/* ------------------------------------------------------------------ */
/* test_fragment_index_out_of_range_returns_error                      */
/* ------------------------------------------------------------------ */

static void test_fragment_index_out_of_range_returns_error(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    memset(&f, 0, sizeof(f));
    f.msg_id = 2;
    f.fragment_index = 3; /* >= total_fragments=2 */
    f.total_fragments = 2;
    f.data_length = 1;

    reassembly_slot_t *completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, MAC_A, &f, 0, &completed);
    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, r);
}

/* ------------------------------------------------------------------ */
/* test_null_args_return_error                                         */
/* ------------------------------------------------------------------ */

static void test_null_args_return_error(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 1, 0, 1, MSG_STATUS, 0x01);
    reassembly_slot_t *completed = NULL;

    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, reassembler_absorb(NULL, MAC_A, &f, 0, &completed));
    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, reassembler_absorb(cache, NULL, &f, 0, &completed));
    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, reassembler_absorb(cache, MAC_A, NULL, 0, &completed));
    TEST_ASSERT_EQUAL(REASSEMBLE_ERROR, reassembler_absorb(cache, MAC_A, &f, 0, NULL));
}

/* ------------------------------------------------------------------ */
/* test_single_fragment_message_completes                              */
/* ------------------------------------------------------------------ */

static void test_single_fragment_message_completes(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 55, 0, 1, MSG_BEACON, 0xBB);

    reassembly_slot_t *completed = NULL;
    reassemble_result_t r = reassembler_absorb(cache, MAC_A, &f, 500, &completed);

    TEST_ASSERT_EQUAL(REASSEMBLE_COMPLETE, r);
    TEST_ASSERT_NOT_NULL(completed);
    TEST_ASSERT_EQUAL(4, (int)completed->reassembled_len);
    TEST_ASSERT_EQUAL(MSG_BEACON, completed->original_msg_type);

    reassembler_release(completed);
}

/* ------------------------------------------------------------------ */
/* test_reassembler_release_zeroes_slot                                */
/* ------------------------------------------------------------------ */

static void test_reassembler_release_zeroes_slot(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 60, 0, 1, MSG_STATUS, 0xCC);

    reassembly_slot_t *completed = NULL;
    reassembler_absorb(cache, MAC_A, &f, 1, &completed);
    TEST_ASSERT_NOT_NULL(completed);

    reassembler_release(completed);
    TEST_ASSERT_EQUAL(0, (int)completed->total_fragments);
    TEST_ASSERT_EQUAL(0, (int)completed->received_mask);
    TEST_ASSERT_EQUAL(0, (int)completed->reassembled_len);
}

/* ------------------------------------------------------------------ */
/* test_prune_does_not_affect_fresh_entries                            */
/* ------------------------------------------------------------------ */

static void test_prune_does_not_affect_fresh_entries(void)
{
    reassembly_slot_t cache[REASSEMBLY_CACHE_SIZE];
    reassembler_init(cache);

    thinkpack_fragment_data_t f;
    make_fragment(&f, 70, 0, 2, MSG_STATUS, 0x77);
    reassembly_slot_t *completed = NULL;
    reassembler_absorb(cache, MAC_A, &f, 10000, &completed);

    /* Prune at t=10001, stale_ms=5000 — entry is only 1 ms old. */
    reassembler_prune(cache, 10001, 5000);

    bool found = false;
    for (int i = 0; i < REASSEMBLY_CACHE_SIZE; i++) {
        if (cache[i].total_fragments > 0 && cache[i].msg_id == 70) {
            found = true;
        }
    }
    TEST_ASSERT_TRUE(found);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_single_fragment_is_incomplete);
    RUN_TEST(test_all_fragments_in_order_complete);
    RUN_TEST(test_out_of_order_complete);
    RUN_TEST(test_duplicate_fragment_idempotent);
    RUN_TEST(test_two_concurrent_messages_from_different_macs);
    RUN_TEST(test_cache_full_evicts_oldest);
    RUN_TEST(test_prune_stale_entries);
    RUN_TEST(test_total_fragments_mismatch_returns_error);
    RUN_TEST(test_invalid_total_fragments_returns_error);
    RUN_TEST(test_fragment_index_out_of_range_returns_error);
    RUN_TEST(test_null_args_return_error);
    RUN_TEST(test_single_fragment_message_completes);
    RUN_TEST(test_reassembler_release_zeroes_slot);
    RUN_TEST(test_prune_does_not_affect_fresh_entries);

    int failures = UNITY_END();
    return (failures > 0) ? 1 : 0;
}
