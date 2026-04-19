/**
 * @file echo_chamber_test.c
 * @brief Host-based unit tests for the echo_chamber behaviour.
 *
 * Assertions cover:
 *   - Pitch-shift distribution across slot indices (0, odd, even).
 *   - Peer-count edge cases: 0, 1, exactly 8, > 8 (clamped).
 *   - Payload sizing + clip_id fidelity.
 *   - Packet construction: magic, checksum, msg_type, data_length.
 */

#include <string.h>

#include "echo_chamber.h"
#include "thinkpack_protocol.h"
#include "unity_compat.h"

static const uint8_t TEST_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
static const uint8_t TEST_SEQ = 0x7A;

/* ------------------------------------------------------------------ */
/* Pitch-table policy                                                  */
/* ------------------------------------------------------------------ */

static void test_pitch_table_zero_peers_is_all_zero(void)
{
    int8_t table[8];
    memset(table, 0x7F, sizeof(table)); /* poison */

    echo_chamber_compute_pitch_table(0, table, 8);

    for (uint8_t i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL(0, table[i]);
    }
}

static void test_pitch_table_single_peer_only_slot0(void)
{
    int8_t table[8];
    memset(table, 0x7F, sizeof(table));

    echo_chamber_compute_pitch_table(1, table, 8);

    TEST_ASSERT_EQUAL(0, table[0]);
    for (uint8_t i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL(0, table[i]);
    }
}

static void test_pitch_table_full_distribution(void)
{
    int8_t table[8];
    memset(table, 0x7F, sizeof(table));

    echo_chamber_compute_pitch_table(8, table, 8);

    /* Policy: slot 0 = 0, odd slots go negative in -2 steps,
     * even slots (>0) go positive in +2 steps. */
    TEST_ASSERT_EQUAL(0, table[0]);
    TEST_ASSERT_EQUAL(-2, table[1]);
    TEST_ASSERT_EQUAL(2, table[2]);
    TEST_ASSERT_EQUAL(-4, table[3]);
    TEST_ASSERT_EQUAL(4, table[4]);
    TEST_ASSERT_EQUAL(-6, table[5]);
    TEST_ASSERT_EQUAL(6, table[6]);
    TEST_ASSERT_EQUAL(-8, table[7]);
}

static void test_pitch_table_peer_count_exceeds_table(void)
{
    int8_t table[8];
    memset(table, 0x7F, sizeof(table));

    /* peer_count > table len should clamp to table len */
    echo_chamber_compute_pitch_table(200, table, 8);

    TEST_ASSERT_EQUAL(0, table[0]);
    TEST_ASSERT_EQUAL(-2, table[1]);
    TEST_ASSERT_EQUAL(6, table[6]);
    TEST_ASSERT_EQUAL(-8, table[7]);
}

static void test_pitch_table_null_guard(void)
{
    /* Must not crash */
    echo_chamber_compute_pitch_table(3, NULL, 0);
    echo_chamber_compute_pitch_table(3, NULL, 8);
    TEST_ASSERT_TRUE(1);
}

/* ------------------------------------------------------------------ */
/* Payload builder                                                     */
/* ------------------------------------------------------------------ */

static void test_build_payload_basic(void)
{
    audio_clip_broadcast_payload_t p;
    memset(&p, 0xAA, sizeof(p));

    echo_chamber_build_payload(&p, 0x2A, 12345, 4);

    TEST_ASSERT_EQUAL(0x2A, p.clip_id);
    TEST_ASSERT_EQUAL(12345, p.sample_count);
    TEST_ASSERT_EQUAL(0, p.flags);

    /* First four slots populated per policy; remainder zero. */
    TEST_ASSERT_EQUAL(0, p.per_peer_semitone_shift[0]);
    TEST_ASSERT_EQUAL(-2, p.per_peer_semitone_shift[1]);
    TEST_ASSERT_EQUAL(2, p.per_peer_semitone_shift[2]);
    TEST_ASSERT_EQUAL(-4, p.per_peer_semitone_shift[3]);
    TEST_ASSERT_EQUAL(0, p.per_peer_semitone_shift[4]);
    TEST_ASSERT_EQUAL(0, p.per_peer_semitone_shift[7]);
}

static void test_build_payload_clamps_sample_count(void)
{
    audio_clip_broadcast_payload_t p;
    echo_chamber_build_payload(&p, 0, 65535, 1);

    /* Max samples = 2 s @ 16 kHz = 32000 */
    TEST_ASSERT_EQUAL(32000, p.sample_count);
}

static void test_build_payload_null_guard(void)
{
    echo_chamber_build_payload(NULL, 0, 0, 0);
    TEST_ASSERT_TRUE(1);
}

/* ------------------------------------------------------------------ */
/* Packet builder                                                      */
/* ------------------------------------------------------------------ */

static void test_build_packet_stamps_magic_and_checksum(void)
{
    thinkpack_packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt));

    echo_chamber_build_packet(&pkt, TEST_SEQ, TEST_MAC, 0x03, 8000, 3);

    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_0, pkt.magic[0]);
    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_VERSION, pkt.magic[1]);
    TEST_ASSERT_EQUAL(MSG_AUDIO_CLIP_BROADCAST, pkt.msg_type);
    TEST_ASSERT_EQUAL(TEST_SEQ, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(audio_clip_broadcast_payload_t), pkt.data_length);
    TEST_ASSERT_TRUE(thinkpack_verify_checksum(&pkt));

    /* Payload round-trip */
    audio_clip_broadcast_payload_t out;
    memcpy(&out, pkt.data, sizeof(out));
    TEST_ASSERT_EQUAL(0x03, out.clip_id);
    TEST_ASSERT_EQUAL(8000, out.sample_count);
    TEST_ASSERT_EQUAL(0, out.per_peer_semitone_shift[0]);
    TEST_ASSERT_EQUAL(-2, out.per_peer_semitone_shift[1]);
    TEST_ASSERT_EQUAL(2, out.per_peer_semitone_shift[2]);
}

static void test_build_packet_zero_peers(void)
{
    thinkpack_packet_t pkt;
    echo_chamber_build_packet(&pkt, 0, TEST_MAC, 0, 0, 0);

    TEST_ASSERT_TRUE(thinkpack_verify_checksum(&pkt));
    audio_clip_broadcast_payload_t out;
    memcpy(&out, pkt.data, sizeof(out));
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL(0, out.per_peer_semitone_shift[i]);
    }
}

static void test_build_packet_null_guard(void)
{
    echo_chamber_build_packet(NULL, 0, TEST_MAC, 0, 0, 0);
    TEST_ASSERT_TRUE(1);
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_pitch_table_zero_peers_is_all_zero);
    RUN_TEST(test_pitch_table_single_peer_only_slot0);
    RUN_TEST(test_pitch_table_full_distribution);
    RUN_TEST(test_pitch_table_peer_count_exceeds_table);
    RUN_TEST(test_pitch_table_null_guard);

    RUN_TEST(test_build_payload_basic);
    RUN_TEST(test_build_payload_clamps_sample_count);
    RUN_TEST(test_build_payload_null_guard);

    RUN_TEST(test_build_packet_stamps_magic_and_checksum);
    RUN_TEST(test_build_packet_zero_peers);
    RUN_TEST(test_build_packet_null_guard);

    int failures = UNITY_END();
    return failures == 0 ? 0 : 1;
}
