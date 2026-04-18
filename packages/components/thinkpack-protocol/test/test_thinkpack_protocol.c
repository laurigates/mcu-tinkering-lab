/**
 * @file test_thinkpack_protocol.c
 * @brief Host-based unit tests for the thinkpack-protocol component.
 *
 * Tests pure protocol logic: XOR checksum, packet construction, magic bytes,
 * priority calculation, and struct sizes. No hardware required.
 *
 * Build and run: make test
 */

#include <string.h>

#include "thinkpack_protocol.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static const uint8_t TEST_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0x01, 0x02};

/** Verify that a packet's checksum field matches its content. */
static void assert_packet_checksum_valid(const thinkpack_packet_t *p)
{
    uint8_t computed = thinkpack_checksum((const uint8_t *)p, sizeof(*p) - 1);
    TEST_ASSERT_EQUAL(computed, p->checksum);
}

/** Verify that magic bytes are set correctly. */
static void assert_magic_valid(const thinkpack_packet_t *p)
{
    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_0, p->magic[0]);
    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_VERSION, p->magic[1]);
}

/* ------------------------------------------------------------------ */
/* thinkpack_checksum                                                  */
/* ------------------------------------------------------------------ */

static void test_checksum_empty_buffer(void)
{
    /* XOR identity: empty input → 0 */
    TEST_ASSERT_EQUAL(0, thinkpack_checksum((const uint8_t *)"", 0));
}

static void test_checksum_single_byte(void)
{
    uint8_t buf[] = {0xAB};
    TEST_ASSERT_EQUAL(0xAB, thinkpack_checksum(buf, 1));
}

static void test_checksum_xor_cancels(void)
{
    /* 0xFF ^ 0xFF == 0x00 */
    uint8_t buf[] = {0xFF, 0xFF};
    TEST_ASSERT_EQUAL(0x00, thinkpack_checksum(buf, 2));
}

static void test_checksum_multi_byte(void)
{
    /* 0x01 ^ 0x02 = 0x03, 0x03 ^ 0x04 = 0x07 */
    uint8_t buf[] = {0x01, 0x02, 0x04};
    TEST_ASSERT_EQUAL(0x07, thinkpack_checksum(buf, 3));
}

static void test_checksum_full_byte_range(void)
{
    uint8_t buf[] = {0x00, 0xFF, 0xAA, 0x55};
    /* 0x00 ^ 0xFF = 0xFF, 0xFF ^ 0xAA = 0x55, 0x55 ^ 0x55 = 0x00 */
    TEST_ASSERT_EQUAL(0x00, thinkpack_checksum(buf, 4));
}

/* ------------------------------------------------------------------ */
/* thinkpack_verify_checksum                                           */
/* ------------------------------------------------------------------ */

static void test_verify_checksum_correct(void)
{
    thinkpack_packet_t pkt = {0};
    pkt.msg_type = MSG_BEACON;
    pkt.sequence_number = 0x01;
    memcpy(pkt.src_mac, TEST_MAC, 6);
    pkt.data_length = 0;
    thinkpack_finalize(&pkt);
    TEST_ASSERT_TRUE(thinkpack_verify_checksum(&pkt));
}

static void test_verify_checksum_wrong(void)
{
    thinkpack_packet_t pkt = {0};
    thinkpack_finalize(&pkt);
    /* Corrupt the checksum */
    pkt.checksum ^= 0xFF;
    TEST_ASSERT_FALSE(thinkpack_verify_checksum(&pkt));
}

static void test_verify_checksum_zero_length(void)
{
    /* Empty (zeroed) packet — finalize computes a valid checksum */
    thinkpack_packet_t pkt = {0};
    thinkpack_finalize(&pkt);
    TEST_ASSERT_TRUE(thinkpack_verify_checksum(&pkt));
}

static void test_verify_checksum_round_trip(void)
{
    thinkpack_packet_t pkt = {0};
    pkt.msg_type = MSG_SYNC_PULSE;
    pkt.sequence_number = 0x42;
    memcpy(pkt.src_mac, TEST_MAC, 6);
    pkt.data_length = 3;
    pkt.data[0] = 0xDE;
    pkt.data[1] = 0xAD;
    pkt.data[2] = 0xBE;
    thinkpack_finalize(&pkt);
    TEST_ASSERT_TRUE(thinkpack_verify_checksum(&pkt));
}

/* ------------------------------------------------------------------ */
/* thinkpack_finalize                                                  */
/* ------------------------------------------------------------------ */

static void test_finalize_sets_magic_bytes(void)
{
    thinkpack_packet_t pkt = {0};
    thinkpack_finalize(&pkt);
    assert_magic_valid(&pkt);
}

static void test_finalize_checksum_valid(void)
{
    thinkpack_packet_t pkt = {0};
    pkt.msg_type = MSG_COMMAND;
    thinkpack_finalize(&pkt);
    assert_packet_checksum_valid(&pkt);
}

static void test_finalize_null_no_crash(void)
{
    /* Must not crash */
    thinkpack_finalize(NULL);
}

/* ------------------------------------------------------------------ */
/* thinkpack_prepare_beacon                                            */
/* ------------------------------------------------------------------ */

static void test_prepare_beacon_fields(void)
{
    thinkpack_packet_t pkt = {0};
    thinkpack_beacon_data_t beacon = {0};
    beacon.box_type = BOX_BRAINBOX;
    beacon.capabilities = CAP_LLM | CAP_WIFI | CAP_DISPLAY;
    beacon.priority = thinkpack_priority_for_capabilities(beacon.capabilities, TEST_MAC);
    beacon.battery_level = 80;
    beacon.group_state = 1;
    strncpy(beacon.name, "Brainbox-1", THINKPACK_BOX_NAME_LEN - 1);

    thinkpack_prepare_beacon(&pkt, 0x10, TEST_MAC, &beacon);

    TEST_ASSERT_EQUAL(MSG_BEACON, pkt.msg_type);
    TEST_ASSERT_EQUAL(0x10, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL(sizeof(thinkpack_beacon_data_t), pkt.data_length);
    assert_magic_valid(&pkt);
    assert_packet_checksum_valid(&pkt);
}

static void test_prepare_beacon_payload_round_trip(void)
{
    thinkpack_packet_t pkt = {0};
    thinkpack_beacon_data_t beacon = {0};
    beacon.box_type = BOX_CHATTERBOX;
    beacon.capabilities = CAP_AUDIO_OUT | CAP_AUDIO_IN;
    beacon.battery_level = 55;
    beacon.group_state = 3;
    strncpy(beacon.name, "Chatter", THINKPACK_BOX_NAME_LEN - 1);

    thinkpack_prepare_beacon(&pkt, 0x20, TEST_MAC, &beacon);

    thinkpack_beacon_data_t out = {0};
    memcpy(&out, pkt.data, sizeof(out));

    TEST_ASSERT_EQUAL(BOX_CHATTERBOX, out.box_type);
    TEST_ASSERT_EQUAL(CAP_AUDIO_OUT | CAP_AUDIO_IN, out.capabilities);
    TEST_ASSERT_EQUAL(55, out.battery_level);
    TEST_ASSERT_EQUAL(3, out.group_state);
    TEST_ASSERT_EQUAL_STRING("Chatter", out.name);
}

static void test_prepare_beacon_null_packet_no_crash(void)
{
    thinkpack_beacon_data_t beacon = {0};
    thinkpack_prepare_beacon(NULL, 0, TEST_MAC, &beacon);
}

static void test_prepare_beacon_null_beacon_no_crash(void)
{
    thinkpack_packet_t pkt = {0};
    thinkpack_prepare_beacon(&pkt, 0, TEST_MAC, NULL);
}

/* ------------------------------------------------------------------ */
/* thinkpack_prepare_election_bid                                      */
/* ------------------------------------------------------------------ */

static void test_prepare_election_bid_fields(void)
{
    thinkpack_packet_t pkt = {0};
    uint32_t priority = thinkpack_priority_for_capabilities(CAP_LLM | CAP_WIFI, TEST_MAC);

    thinkpack_prepare_election_bid(&pkt, 0x30, TEST_MAC, priority);

    TEST_ASSERT_EQUAL(MSG_ELECTION_BID, pkt.msg_type);
    TEST_ASSERT_EQUAL(0x30, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL(sizeof(thinkpack_election_bid_data_t), pkt.data_length);
    assert_magic_valid(&pkt);
    assert_packet_checksum_valid(&pkt);
}

static void test_prepare_election_bid_priority_mac_round_trip(void)
{
    thinkpack_packet_t pkt = {0};
    uint32_t priority = 0xDEADBEEF;

    thinkpack_prepare_election_bid(&pkt, 0x31, TEST_MAC, priority);

    thinkpack_election_bid_data_t out = {0};
    memcpy(&out, pkt.data, sizeof(out));

    TEST_ASSERT_EQUAL(priority, out.priority);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, out.mac, 6);
}

static void test_prepare_election_bid_null_no_crash(void)
{
    thinkpack_prepare_election_bid(NULL, 0, TEST_MAC, 0);
}

/* ------------------------------------------------------------------ */
/* thinkpack_prepare_leader_claim                                      */
/* ------------------------------------------------------------------ */

static void test_prepare_leader_claim_fields(void)
{
    thinkpack_packet_t pkt = {0};
    uint32_t priority = thinkpack_priority_for_capabilities(CAP_LLM, TEST_MAC);

    thinkpack_prepare_leader_claim(&pkt, 0x40, TEST_MAC, priority, 6);

    TEST_ASSERT_EQUAL(MSG_LEADER_CLAIM, pkt.msg_type);
    TEST_ASSERT_EQUAL(0x40, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL(sizeof(thinkpack_leader_claim_data_t), pkt.data_length);
    assert_magic_valid(&pkt);
    assert_packet_checksum_valid(&pkt);
}

static void test_prepare_leader_claim_priority_mac_channel_round_trip(void)
{
    thinkpack_packet_t pkt = {0};
    uint32_t priority = 0x12345678;
    uint8_t channel = 11;

    thinkpack_prepare_leader_claim(&pkt, 0x41, TEST_MAC, priority, channel);

    thinkpack_leader_claim_data_t out = {0};
    memcpy(&out, pkt.data, sizeof(out));

    TEST_ASSERT_EQUAL(priority, out.priority);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, out.mac, 6);
    TEST_ASSERT_EQUAL(channel, out.channel);
}

static void test_prepare_leader_claim_null_no_crash(void)
{
    thinkpack_prepare_leader_claim(NULL, 0, TEST_MAC, 0, 0);
}

/* ------------------------------------------------------------------ */
/* thinkpack_prepare_sync_pulse                                        */
/* ------------------------------------------------------------------ */

static void test_prepare_sync_pulse_fields(void)
{
    thinkpack_packet_t pkt = {0};

    thinkpack_prepare_sync_pulse(&pkt, 0x50, TEST_MAC, 123456789UL, 3);

    TEST_ASSERT_EQUAL(MSG_SYNC_PULSE, pkt.msg_type);
    TEST_ASSERT_EQUAL(0x50, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL(sizeof(thinkpack_sync_pulse_data_t), pkt.data_length);
    assert_magic_valid(&pkt);
    assert_packet_checksum_valid(&pkt);
}

static void test_prepare_sync_pulse_timestamp_phase_round_trip(void)
{
    thinkpack_packet_t pkt = {0};
    uint32_t ts = 0xCAFEBABE;
    uint8_t phase = 7;

    thinkpack_prepare_sync_pulse(&pkt, 0x51, TEST_MAC, ts, phase);

    thinkpack_sync_pulse_data_t out = {0};
    memcpy(&out, pkt.data, sizeof(out));

    TEST_ASSERT_EQUAL(ts, out.timestamp_ms);
    TEST_ASSERT_EQUAL(phase, out.phase);
}

static void test_prepare_sync_pulse_null_no_crash(void)
{
    thinkpack_prepare_sync_pulse(NULL, 0, TEST_MAC, 0, 0);
}

/* ------------------------------------------------------------------ */
/* thinkpack_priority_for_capabilities                                 */
/* ------------------------------------------------------------------ */

static void test_priority_llm_wifi(void)
{
    /* cap_score = 1000 + 500 = 1500; tiebreaker = mac[4]<<8 | mac[5] = 0x0102 */
    uint32_t p = thinkpack_priority_for_capabilities(CAP_LLM | CAP_WIFI, TEST_MAC);
    uint32_t expected = (1500u << 16) | 0x0102u;
    TEST_ASSERT_EQUAL(expected, p);
}

static void test_priority_zero_capabilities(void)
{
    /* Only MAC tiebreaker */
    uint32_t p = thinkpack_priority_for_capabilities(0, TEST_MAC);
    uint32_t expected = (0u << 16) | 0x0102u;
    TEST_ASSERT_EQUAL(expected, p);
}

static void test_priority_all_scored_capabilities(void)
{
    /* All five scored caps: 1000+500+200+100+50 = 1850 */
    uint16_t caps = CAP_LLM | CAP_WIFI | CAP_DISPLAY | CAP_AUDIO_OUT | CAP_AUDIO_IN;
    uint32_t p = thinkpack_priority_for_capabilities(caps, TEST_MAC);
    uint32_t expected = (1850u << 16) | 0x0102u;
    TEST_ASSERT_EQUAL(expected, p);
}

static void test_priority_mac_tiebreaker_low_bits(void)
{
    /* Two nodes with same caps — MAC bytes differ */
    uint8_t mac_a[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t mac_b[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
    uint16_t caps = CAP_LLM;

    uint32_t pa = thinkpack_priority_for_capabilities(caps, mac_a);
    uint32_t pb = thinkpack_priority_for_capabilities(caps, mac_b);

    /* Same capability score, different tiebreaker */
    TEST_ASSERT_EQUAL((uint32_t)(pa >> 16), (uint32_t)(pb >> 16));
    TEST_ASSERT_NOT_EQUAL(pa, pb);
    /* mac_b has higher tiebreaker */
    TEST_ASSERT_TRUE(pb > pa);
}

static void test_priority_unscored_caps_do_not_add_score(void)
{
    /* CAP_NFC, CAP_POTS, etc. are not in the priority table — score stays 0 */
    uint16_t caps = CAP_NFC | CAP_POTS | CAP_TOUCH | CAP_RHYTHM | CAP_STORAGE;
    uint32_t p = thinkpack_priority_for_capabilities(caps, TEST_MAC);
    uint32_t expected = (0u << 16) | 0x0102u;
    TEST_ASSERT_EQUAL(expected, p);
}

/* ------------------------------------------------------------------ */
/* Packet layout and size sanity                                       */
/* ------------------------------------------------------------------ */

static void test_packet_size_within_espnow_limit(void)
{
    /* Compile-time assertion already enforces this; runtime check for docs */
    TEST_ASSERT_TRUE(sizeof(thinkpack_packet_t) <= 250);
}

static void test_packet_size_exact(void)
{
    /* magic[2] + msg_type(1) + seq(1) + src_mac[6] + data_length(1)
     * + data[200] + checksum(1) = 212 bytes */
    TEST_ASSERT_EQUAL(2 + 1 + 1 + 6 + 1 + 200 + 1, (int)sizeof(thinkpack_packet_t));
    TEST_ASSERT_EQUAL(212, (int)sizeof(thinkpack_packet_t));
}

static void test_payload_structs_fit_in_max_data_len(void)
{
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_beacon_data_t) <= THINKPACK_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_election_bid_data_t) <= THINKPACK_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_leader_claim_data_t) <= THINKPACK_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_capability_reply_data_t) <= THINKPACK_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_sync_pulse_data_t) <= THINKPACK_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(thinkpack_command_data_t) <= THINKPACK_MAX_DATA_LEN);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    /* thinkpack_checksum */
    RUN_TEST(test_checksum_empty_buffer);
    RUN_TEST(test_checksum_single_byte);
    RUN_TEST(test_checksum_xor_cancels);
    RUN_TEST(test_checksum_multi_byte);
    RUN_TEST(test_checksum_full_byte_range);

    /* thinkpack_verify_checksum */
    RUN_TEST(test_verify_checksum_correct);
    RUN_TEST(test_verify_checksum_wrong);
    RUN_TEST(test_verify_checksum_zero_length);
    RUN_TEST(test_verify_checksum_round_trip);

    /* thinkpack_finalize */
    RUN_TEST(test_finalize_sets_magic_bytes);
    RUN_TEST(test_finalize_checksum_valid);
    RUN_TEST(test_finalize_null_no_crash);

    /* thinkpack_prepare_beacon */
    RUN_TEST(test_prepare_beacon_fields);
    RUN_TEST(test_prepare_beacon_payload_round_trip);
    RUN_TEST(test_prepare_beacon_null_packet_no_crash);
    RUN_TEST(test_prepare_beacon_null_beacon_no_crash);

    /* thinkpack_prepare_election_bid */
    RUN_TEST(test_prepare_election_bid_fields);
    RUN_TEST(test_prepare_election_bid_priority_mac_round_trip);
    RUN_TEST(test_prepare_election_bid_null_no_crash);

    /* thinkpack_prepare_leader_claim */
    RUN_TEST(test_prepare_leader_claim_fields);
    RUN_TEST(test_prepare_leader_claim_priority_mac_channel_round_trip);
    RUN_TEST(test_prepare_leader_claim_null_no_crash);

    /* thinkpack_prepare_sync_pulse */
    RUN_TEST(test_prepare_sync_pulse_fields);
    RUN_TEST(test_prepare_sync_pulse_timestamp_phase_round_trip);
    RUN_TEST(test_prepare_sync_pulse_null_no_crash);

    /* thinkpack_priority_for_capabilities */
    RUN_TEST(test_priority_llm_wifi);
    RUN_TEST(test_priority_zero_capabilities);
    RUN_TEST(test_priority_all_scored_capabilities);
    RUN_TEST(test_priority_mac_tiebreaker_low_bits);
    RUN_TEST(test_priority_unscored_caps_do_not_add_score);

    /* Packet layout */
    RUN_TEST(test_packet_size_within_espnow_limit);
    RUN_TEST(test_packet_size_exact);
    RUN_TEST(test_payload_structs_fit_in_max_data_len);

    int failures = UNITY_END();
    return (failures > 0) ? 1 : 0;
}
