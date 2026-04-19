/**
 * @file test_commands.c
 * @brief Host-based unit tests for thinkpack-behaviors (dispatcher + executor).
 *
 * Tests cover:
 *   - Round-trip packet construction for all command builders
 *   - Edge cases for CMD_PLAY_SEQUENCE (0 notes, 20 notes)
 *   - command_executor_register: duplicate rejection, registry-full rejection
 *   - command_executor_dispatch: correct routing, argument fidelity
 *   - command_executor_dispatch: unknown command_id does not crash
 *   - command_executor_reset: clears all registrations
 *
 * Build and run: make test
 */

#include <string.h>

#include "command_dispatcher.h"
#include "command_executor.h"
#include "thinkpack_protocol.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Shared test fixtures                                                */
/* ------------------------------------------------------------------ */

static const uint8_t TEST_MAC[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
static const uint8_t TEST_SEQ = 0x42;

/** Verify magic bytes and checksum of any finalized packet. */
static void assert_packet_valid(const thinkpack_packet_t *p)
{
    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_0, p->magic[0]);
    TEST_ASSERT_EQUAL(THINKPACK_MAGIC_VERSION, p->magic[1]);
    TEST_ASSERT_TRUE(thinkpack_verify_checksum(p));
}

/** Extract the command envelope from a built packet. */
static const thinkpack_command_data_t *get_cmd(const thinkpack_packet_t *p)
{
    return (const thinkpack_command_data_t *)(const void *)p->data;
}

/* ------------------------------------------------------------------ */
/* command_build_led_pattern                                           */
/* ------------------------------------------------------------------ */

static void test_build_led_pattern_round_trip(void)
{
    cmd_led_pattern_payload_t in = {
        .r = 0xFF, .g = 0x80, .b = 0x10, .pattern = LED_PATTERN_RAINBOW};
    thinkpack_packet_t pkt;

    command_build_led_pattern(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    TEST_ASSERT_EQUAL(MSG_COMMAND, pkt.msg_type);
    TEST_ASSERT_EQUAL(TEST_SEQ, pkt.sequence_number);
    TEST_ASSERT_EQUAL_MEMORY(TEST_MAC, pkt.src_mac, 6);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(thinkpack_command_data_t), pkt.data_length);

    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_LED_PATTERN, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_led_pattern_payload_t), cmd->length);

    cmd_led_pattern_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(0xFF, out.r);
    TEST_ASSERT_EQUAL(0x80, out.g);
    TEST_ASSERT_EQUAL(0x10, out.b);
    TEST_ASSERT_EQUAL(LED_PATTERN_RAINBOW, out.pattern);
}

/* ------------------------------------------------------------------ */
/* command_build_set_mood                                              */
/* ------------------------------------------------------------------ */

static void test_build_set_mood_round_trip(void)
{
    cmd_set_mood_payload_t in = {.hue = 200, .intensity = 75};
    thinkpack_packet_t pkt;

    command_build_set_mood(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    TEST_ASSERT_EQUAL(MSG_COMMAND, pkt.msg_type);

    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_SET_MOOD, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_set_mood_payload_t), cmd->length);

    cmd_set_mood_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(200, out.hue);
    TEST_ASSERT_EQUAL(75, out.intensity);
}

/* ------------------------------------------------------------------ */
/* command_build_play_melody                                           */
/* ------------------------------------------------------------------ */

static void test_build_play_melody_round_trip(void)
{
    cmd_play_melody_payload_t in = {.pattern_id = 2, .repeat_count = 8};
    thinkpack_packet_t pkt;

    command_build_play_melody(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_PLAY_MELODY, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_play_melody_payload_t), cmd->length);

    cmd_play_melody_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(2, out.pattern_id);
    TEST_ASSERT_EQUAL(8, out.repeat_count);
}

/* ------------------------------------------------------------------ */
/* command_build_buzz                                                  */
/* ------------------------------------------------------------------ */

static void test_build_buzz_round_trip(void)
{
    cmd_buzz_payload_t in = {.note = 12, .duration_ms = 250, .semitone_shift = 0};
    thinkpack_packet_t pkt;

    command_build_buzz(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_BUZZ, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_buzz_payload_t), cmd->length);

    cmd_buzz_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(12, out.note);
    TEST_ASSERT_EQUAL(250, out.duration_ms);
    TEST_ASSERT_EQUAL(0, out.semitone_shift);
}

static void test_build_buzz_negative_semitone(void)
{
    cmd_buzz_payload_t in = {.note = 7, .duration_ms = 100, .semitone_shift = -3};
    thinkpack_packet_t pkt;

    command_build_buzz(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);

    cmd_buzz_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(-3, (int8_t)out.semitone_shift);
}

/* ------------------------------------------------------------------ */
/* command_build_play_sequence — edge cases                            */
/* ------------------------------------------------------------------ */

static void test_build_play_sequence_zero_notes(void)
{
    cmd_play_sequence_payload_t in;
    memset(&in, 0, sizeof(in));
    in.note_count = 0;
    in.note_duration_ms = 200;
    in.semitone_shift = 0;

    thinkpack_packet_t pkt;
    command_build_play_sequence(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_PLAY_SEQUENCE, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_play_sequence_payload_t), cmd->length);

    cmd_play_sequence_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(0, out.note_count);
}

static void test_build_play_sequence_max_notes(void)
{
    cmd_play_sequence_payload_t in;
    memset(&in, 0, sizeof(in));
    in.note_count = CMD_SEQUENCE_MAX_NOTES;
    in.note_duration_ms = 150;
    in.semitone_shift = 2;
    for (uint8_t i = 0; i < CMD_SEQUENCE_MAX_NOTES; i++) {
        in.notes[i] = i;
    }

    thinkpack_packet_t pkt;
    command_build_play_sequence(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_PLAY_SEQUENCE, cmd->command_id);

    cmd_play_sequence_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(CMD_SEQUENCE_MAX_NOTES, out.note_count);
    TEST_ASSERT_EQUAL(150, out.note_duration_ms);
    TEST_ASSERT_EQUAL(2, (int8_t)out.semitone_shift);
    for (uint8_t i = 0; i < CMD_SEQUENCE_MAX_NOTES; i++) {
        TEST_ASSERT_EQUAL(i, out.notes[i]);
    }
}

/* ------------------------------------------------------------------ */
/* command_build_display_line                                          */
/* ------------------------------------------------------------------ */

static void test_build_display_line_round_trip(void)
{
    cmd_display_line_payload_t in;
    memset(&in, 0, sizeof(in));
    in.line = 3;
    strncpy(in.text, "Hello ThinkPack", sizeof(in.text) - 1);

    thinkpack_packet_t pkt;
    command_build_display_line(&pkt, TEST_SEQ, TEST_MAC, &in);

    assert_packet_valid(&pkt);
    const thinkpack_command_data_t *cmd = get_cmd(&pkt);
    TEST_ASSERT_EQUAL(CMD_DISPLAY_LINE, cmd->command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_display_line_payload_t), cmd->length);

    cmd_display_line_payload_t out;
    memcpy(&out, cmd->payload, sizeof(out));
    TEST_ASSERT_EQUAL(3, out.line);
    TEST_ASSERT_EQUAL_STRING("Hello ThinkPack", out.text);
}

/* ------------------------------------------------------------------ */
/* command_executor_register                                           */
/* ------------------------------------------------------------------ */

static void dummy_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                          void *user_ctx)
{
    (void)command_id;
    (void)payload;
    (void)length;
    (void)user_ctx;
}

static void test_executor_register_rejects_duplicate(void)
{
    command_executor_reset();

    esp_err_t r1 = command_executor_register(CMD_LED_PATTERN, dummy_handler, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, r1);

    esp_err_t r2 = command_executor_register(CMD_LED_PATTERN, dummy_handler, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, r2);
}

static void test_executor_register_rejects_when_full(void)
{
    command_executor_reset();

    /* Fill all 16 slots with distinct command IDs */
    for (int i = 0; i < 16; i++) {
        esp_err_t r = command_executor_register((uint8_t)(0x50 + i), dummy_handler, NULL);
        TEST_ASSERT_EQUAL(ESP_OK, r);
    }

    /* 17th registration must fail with ESP_ERR_NO_MEM */
    esp_err_t r = command_executor_register(0xFE, dummy_handler, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, r);
}

/* ------------------------------------------------------------------ */
/* command_executor_dispatch — routing and argument fidelity           */
/* ------------------------------------------------------------------ */

typedef struct {
    int call_count;
    uint8_t last_command_id;
    uint8_t last_payload[48];
    uint8_t last_length;
    void *last_ctx;
} capture_t;

static void capturing_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                              void *user_ctx)
{
    capture_t *cap = (capture_t *)user_ctx;
    cap->call_count++;
    cap->last_command_id = command_id;
    cap->last_length = length;
    cap->last_ctx = user_ctx;
    if (length > 0 && length <= 48) {
        memcpy(cap->last_payload, payload, length);
    }
}

static void test_executor_dispatch_routes_correctly(void)
{
    command_executor_reset();

    capture_t cap;
    memset(&cap, 0, sizeof(cap));

    esp_err_t r = command_executor_register(CMD_BUZZ, capturing_handler, &cap);
    TEST_ASSERT_EQUAL(ESP_OK, r);

    cmd_buzz_payload_t in = {.note = 5, .duration_ms = 300, .semitone_shift = -1};
    thinkpack_packet_t pkt;
    command_build_buzz(&pkt, TEST_SEQ, TEST_MAC, &in);

    command_executor_dispatch(&pkt);

    TEST_ASSERT_EQUAL(1, cap.call_count);
    TEST_ASSERT_EQUAL(CMD_BUZZ, cap.last_command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_buzz_payload_t), cap.last_length);

    cmd_buzz_payload_t out;
    memcpy(&out, cap.last_payload, sizeof(out));
    TEST_ASSERT_EQUAL(5, out.note);
    TEST_ASSERT_EQUAL(300, out.duration_ms);
    TEST_ASSERT_EQUAL(-1, (int8_t)out.semitone_shift);
}

static void test_executor_dispatch_unknown_id_does_not_crash(void)
{
    command_executor_reset();

    /* Build a packet for CMD_LED_PATTERN but register nothing */
    cmd_led_pattern_payload_t in = {.r = 1, .g = 2, .b = 3, .pattern = LED_PATTERN_SOLID};
    thinkpack_packet_t pkt;
    command_build_led_pattern(&pkt, TEST_SEQ, TEST_MAC, &in);

    /* Must not crash; no handler is registered */
    command_executor_dispatch(&pkt);

    /* If we reach here the test passed */
    TEST_ASSERT_TRUE(1);
}

static void test_executor_dispatch_wrong_msg_type_ignored(void)
{
    command_executor_reset();

    capture_t cap;
    memset(&cap, 0, sizeof(cap));
    command_executor_register(CMD_PLAY_MELODY, capturing_handler, &cap);

    /* Build a beacon packet — wrong msg_type */
    thinkpack_beacon_data_t beacon;
    memset(&beacon, 0, sizeof(beacon));
    thinkpack_packet_t pkt;
    thinkpack_prepare_beacon(&pkt, TEST_SEQ, TEST_MAC, &beacon);

    command_executor_dispatch(&pkt);

    TEST_ASSERT_EQUAL(0, cap.call_count);
}

/* ------------------------------------------------------------------ */
/* command_executor_reset                                              */
/* ------------------------------------------------------------------ */

static void test_executor_reset_clears_all(void)
{
    command_executor_reset();

    /* Fill the registry */
    for (int i = 0; i < 16; i++) {
        command_executor_register((uint8_t)(0x60 + i), dummy_handler, NULL);
    }

    /* Reset */
    command_executor_reset();

    /* After reset we should be able to register 16 entries again */
    for (int i = 0; i < 16; i++) {
        esp_err_t r = command_executor_register((uint8_t)(0x60 + i), dummy_handler, NULL);
        TEST_ASSERT_EQUAL(ESP_OK, r);
    }
}

static void test_executor_reset_stops_dispatch(void)
{
    command_executor_reset();

    capture_t cap;
    memset(&cap, 0, sizeof(cap));
    command_executor_register(CMD_SET_MOOD, capturing_handler, &cap);

    command_executor_reset();

    /* Dispatch a CMD_SET_MOOD — handler was cleared, call_count must stay 0 */
    cmd_set_mood_payload_t in = {.hue = 10, .intensity = 50};
    thinkpack_packet_t pkt;
    command_build_set_mood(&pkt, TEST_SEQ, TEST_MAC, &in);
    command_executor_dispatch(&pkt);

    TEST_ASSERT_EQUAL(0, cap.call_count);
}

/* ------------------------------------------------------------------ */
/* Registration-flow integration: replicate glowbug+boombox pattern     */
/* ------------------------------------------------------------------ */

/* Capture context for the LED handler (glowbug-style). */
static capture_t s_led_cap;

static void led_pattern_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                                void *user_ctx)
{
    (void)user_ctx;
    s_led_cap.call_count++;
    s_led_cap.last_command_id = command_id;
    s_led_cap.last_length = length;
    if (length <= sizeof(s_led_cap.last_payload)) {
        memcpy(s_led_cap.last_payload, payload, length);
    }
}

/* Capture context for the melody handler (boombox-style). */
static capture_t s_melody_cap;

static void play_melody_handler(uint8_t command_id, const uint8_t *payload, uint8_t length,
                                void *user_ctx)
{
    (void)user_ctx;
    s_melody_cap.call_count++;
    s_melody_cap.last_command_id = command_id;
    s_melody_cap.last_length = length;
    if (length <= sizeof(s_melody_cap.last_payload)) {
        memcpy(s_melody_cap.last_payload, payload, length);
    }
}

/**
 * @brief End-to-end: two followers register disjoint handlers; each
 *        receives only the command it registered for.
 *
 * This mirrors how packages/thinkpack/glowbug and packages/thinkpack/boombox
 * wire group_mode_init() → command_executor_register() at startup.  The
 * executor is a shared singleton per binary, so this test instantiates it
 * once with both handlers (as if a single hypothetical follower was handling
 * both domains) and verifies routing fidelity.
 */
static void test_registration_flow_routes_to_registered_handlers(void)
{
    command_executor_reset();
    memset(&s_led_cap, 0, sizeof(s_led_cap));
    memset(&s_melody_cap, 0, sizeof(s_melody_cap));

    /* Register both handlers, each for a different command id. */
    TEST_ASSERT_EQUAL(ESP_OK,
                      command_executor_register(CMD_LED_PATTERN, led_pattern_handler, NULL));
    TEST_ASSERT_EQUAL(ESP_OK,
                      command_executor_register(CMD_PLAY_MELODY, play_melody_handler, NULL));

    /* Dispatch an LED packet — only led handler should fire. */
    cmd_led_pattern_payload_t led_in = {.r = 10, .g = 20, .b = 30, .pattern = LED_PATTERN_BREATHE};
    thinkpack_packet_t led_pkt;
    command_build_led_pattern(&led_pkt, TEST_SEQ, TEST_MAC, &led_in);
    command_executor_dispatch(&led_pkt);

    TEST_ASSERT_EQUAL(1, s_led_cap.call_count);
    TEST_ASSERT_EQUAL(0, s_melody_cap.call_count);
    TEST_ASSERT_EQUAL(CMD_LED_PATTERN, s_led_cap.last_command_id);
    TEST_ASSERT_EQUAL((uint8_t)sizeof(cmd_led_pattern_payload_t), s_led_cap.last_length);

    cmd_led_pattern_payload_t led_out;
    memcpy(&led_out, s_led_cap.last_payload, sizeof(led_out));
    TEST_ASSERT_EQUAL(10, led_out.r);
    TEST_ASSERT_EQUAL(20, led_out.g);
    TEST_ASSERT_EQUAL(30, led_out.b);
    TEST_ASSERT_EQUAL(LED_PATTERN_BREATHE, led_out.pattern);

    /* Dispatch a melody packet — only melody handler should fire. */
    cmd_play_melody_payload_t mel_in = {.pattern_id = 1, .repeat_count = 4};
    thinkpack_packet_t mel_pkt;
    command_build_play_melody(&mel_pkt, TEST_SEQ, TEST_MAC, &mel_in);
    command_executor_dispatch(&mel_pkt);

    TEST_ASSERT_EQUAL(1, s_led_cap.call_count); /* still only the earlier LED dispatch */
    TEST_ASSERT_EQUAL(1, s_melody_cap.call_count);
    TEST_ASSERT_EQUAL(CMD_PLAY_MELODY, s_melody_cap.last_command_id);

    cmd_play_melody_payload_t mel_out;
    memcpy(&mel_out, s_melody_cap.last_payload, sizeof(mel_out));
    TEST_ASSERT_EQUAL(1, mel_out.pattern_id);
    TEST_ASSERT_EQUAL(4, mel_out.repeat_count);

    /* Unregistered command_id — neither handler fires. */
    cmd_set_mood_payload_t mood_in = {.hue = 100, .intensity = 50};
    thinkpack_packet_t mood_pkt;
    command_build_set_mood(&mood_pkt, TEST_SEQ, TEST_MAC, &mood_in);
    command_executor_dispatch(&mood_pkt);

    TEST_ASSERT_EQUAL(1, s_led_cap.call_count);
    TEST_ASSERT_EQUAL(1, s_melody_cap.call_count);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    /* Dispatcher round-trips */
    RUN_TEST(test_build_led_pattern_round_trip);
    RUN_TEST(test_build_set_mood_round_trip);
    RUN_TEST(test_build_play_melody_round_trip);
    RUN_TEST(test_build_buzz_round_trip);
    RUN_TEST(test_build_buzz_negative_semitone);
    RUN_TEST(test_build_play_sequence_zero_notes);
    RUN_TEST(test_build_play_sequence_max_notes);
    RUN_TEST(test_build_display_line_round_trip);

    /* Executor registration */
    RUN_TEST(test_executor_register_rejects_duplicate);
    RUN_TEST(test_executor_register_rejects_when_full);

    /* Executor dispatch */
    RUN_TEST(test_executor_dispatch_routes_correctly);
    RUN_TEST(test_executor_dispatch_unknown_id_does_not_crash);
    RUN_TEST(test_executor_dispatch_wrong_msg_type_ignored);

    /* Executor reset */
    RUN_TEST(test_executor_reset_clears_all);
    RUN_TEST(test_executor_reset_stops_dispatch);

    /* End-to-end registration flow (PR A: glowbug+boombox migration) */
    RUN_TEST(test_registration_flow_routes_to_registered_handlers);

    int failures = UNITY_END();
    return failures == 0 ? 0 : 1;
}
