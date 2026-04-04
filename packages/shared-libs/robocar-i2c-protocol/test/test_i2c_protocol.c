/**
 * @file test_i2c_protocol.c
 * @brief Host-based unit tests for the robocar i2c_protocol component.
 *
 * Tests pure protocol logic: XOR checksum, packet construction, value
 * clamping, null-safety, and struct sizes. No hardware required.
 *
 * Build and run: make test
 */

#include <string.h>

#include "i2c_protocol.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/** Verify that a packet's checksum field matches its content. */
static void assert_packet_checksum_valid(const i2c_command_packet_t *p)
{
    uint8_t computed = calculate_checksum((const uint8_t *)p, sizeof(*p) - 1);
    TEST_ASSERT_EQUAL(computed, p->checksum);
}

/* ------------------------------------------------------------------ */
/* calculate_checksum                                                  */
/* ------------------------------------------------------------------ */

static void test_checksum_empty_buffer(void)
{
    /* XOR identity: empty input → 0 */
    TEST_ASSERT_EQUAL(0, calculate_checksum((const uint8_t *)"", 0));
}

static void test_checksum_single_byte(void)
{
    uint8_t buf[] = {0xAB};
    TEST_ASSERT_EQUAL(0xAB, calculate_checksum(buf, 1));
}

static void test_checksum_xor_cancels(void)
{
    /* 0xFF ^ 0xFF == 0x00 */
    uint8_t buf[] = {0xFF, 0xFF};
    TEST_ASSERT_EQUAL(0x00, calculate_checksum(buf, 2));
}

static void test_checksum_multi_byte(void)
{
    /* 0x01 ^ 0x02 = 0x03, 0x03 ^ 0x04 = 0x07 */
    uint8_t buf[] = {0x01, 0x02, 0x04};
    TEST_ASSERT_EQUAL(0x07, calculate_checksum(buf, 3));
}

static void test_checksum_full_byte_range(void)
{
    uint8_t buf[] = {0x00, 0xFF, 0xAA, 0x55};
    /* 0x00 ^ 0xFF = 0xFF, 0xFF ^ 0xAA = 0x55, 0x55 ^ 0x55 = 0x00 */
    TEST_ASSERT_EQUAL(0x00, calculate_checksum(buf, 4));
}

/* ------------------------------------------------------------------ */
/* verify_checksum                                                     */
/* ------------------------------------------------------------------ */

static void test_verify_checksum_correct(void)
{
    uint8_t buf[] = {0x01, 0x02, 0x04};
    uint8_t cs = calculate_checksum(buf, 3);
    TEST_ASSERT_TRUE(verify_checksum(buf, 3, cs));
}

static void test_verify_checksum_wrong(void)
{
    uint8_t buf[] = {0x01, 0x02, 0x04};
    TEST_ASSERT_FALSE(verify_checksum(buf, 3, 0xFF));
}

static void test_verify_checksum_zero_length(void)
{
    /* Empty buffer: XOR of nothing is 0, checksum 0 is correct */
    TEST_ASSERT_TRUE(verify_checksum((const uint8_t *)"", 0, 0));
}

static void test_verify_checksum_round_trip(void)
{
    /* Any buffer should round-trip correctly */
    uint8_t buf[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x42};
    uint8_t cs = calculate_checksum(buf, sizeof(buf));
    TEST_ASSERT_TRUE(verify_checksum(buf, sizeof(buf), cs));
}

/* ------------------------------------------------------------------ */
/* prepare_movement_command                                            */
/* ------------------------------------------------------------------ */

static void test_movement_command_fields(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_movement_command(&pkt, MOVE_FORWARD, 128, 0x01);

    TEST_ASSERT_EQUAL(CMD_TYPE_MOVEMENT, pkt.command_type);
    TEST_ASSERT_EQUAL(0x01, pkt.sequence_number);
    TEST_ASSERT_EQUAL(sizeof(movement_data_t), pkt.data_length);

    const movement_data_t *d = (const movement_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(MOVE_FORWARD, d->movement);
    TEST_ASSERT_EQUAL(128, d->speed);
}

static void test_movement_command_checksum_valid(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_movement_command(&pkt, MOVE_BACKWARD, 255, 0x42);
    assert_packet_checksum_valid(&pkt);
}

static void test_movement_command_null_packet(void)
{
    /* Must not crash */
    prepare_movement_command(NULL, MOVE_FORWARD, 100, 0);
}

static void test_movement_all_commands_round_trip(void)
{
    movement_command_t cmds[] = {MOVE_FORWARD,   MOVE_BACKWARD,   MOVE_LEFT, MOVE_RIGHT,
                                 MOVE_ROTATE_CW, MOVE_ROTATE_CCW, MOVE_STOP};
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        i2c_command_packet_t pkt = {0};
        prepare_movement_command(&pkt, cmds[i], (uint8_t)(i * 32), (uint8_t)i);

        const movement_data_t *d = (const movement_data_t *)pkt.data;
        TEST_ASSERT_EQUAL(cmds[i], d->movement);
        assert_packet_checksum_valid(&pkt);
    }
}

static void test_movement_speed_max(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_movement_command(&pkt, MOVE_FORWARD, 255, 0x00);

    const movement_data_t *d = (const movement_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(255, d->speed);
    assert_packet_checksum_valid(&pkt);
}

static void test_movement_speed_zero(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_movement_command(&pkt, MOVE_STOP, 0, 0x00);

    const movement_data_t *d = (const movement_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(0, d->speed);
    assert_packet_checksum_valid(&pkt);
}

/* ------------------------------------------------------------------ */
/* prepare_sound_command                                               */
/* ------------------------------------------------------------------ */

static void test_sound_command_beep(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_sound_command(&pkt, SOUND_BEEP, 0x10);

    TEST_ASSERT_EQUAL(CMD_TYPE_SOUND, pkt.command_type);
    TEST_ASSERT_EQUAL(0x10, pkt.sequence_number);
    TEST_ASSERT_EQUAL(sizeof(sound_data_t), pkt.data_length);

    const sound_data_t *d = (const sound_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(SOUND_BEEP, d->sound_type);
    assert_packet_checksum_valid(&pkt);
}

static void test_sound_command_all_types(void)
{
    sound_command_t sounds[] = {SOUND_BEEP, SOUND_MELODY, SOUND_ALERT};
    for (size_t i = 0; i < sizeof(sounds) / sizeof(sounds[0]); i++) {
        i2c_command_packet_t pkt = {0};
        prepare_sound_command(&pkt, sounds[i], (uint8_t)i);

        const sound_data_t *d = (const sound_data_t *)pkt.data;
        TEST_ASSERT_EQUAL(sounds[i], d->sound_type);
        assert_packet_checksum_valid(&pkt);
    }
}

/* ------------------------------------------------------------------ */
/* prepare_servo_command — angle clamping                             */
/* ------------------------------------------------------------------ */

static void test_servo_command_normal_angle(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_servo_command(&pkt, SERVO_PAN, 90, 0x05);

    TEST_ASSERT_EQUAL(CMD_TYPE_SERVO, pkt.command_type);
    const servo_data_t *d = (const servo_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(SERVO_PAN, d->servo_type);
    TEST_ASSERT_EQUAL(90, d->angle);
    assert_packet_checksum_valid(&pkt);
}

static void test_servo_command_clamps_over_180(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_servo_command(&pkt, SERVO_TILT, 200, 0x06);

    const servo_data_t *d = (const servo_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(180, d->angle);
    assert_packet_checksum_valid(&pkt);
}

static void test_servo_command_accepts_180(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_servo_command(&pkt, SERVO_TILT, 180, 0x07);

    const servo_data_t *d = (const servo_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(180, d->angle);
}

static void test_servo_command_accepts_0(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_servo_command(&pkt, SERVO_PAN, 0, 0x08);

    const servo_data_t *d = (const servo_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(0, d->angle);
    assert_packet_checksum_valid(&pkt);
}

static void test_servo_command_extreme_over_180(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_servo_command(&pkt, SERVO_PAN, 255, 0x09);

    const servo_data_t *d = (const servo_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(180, d->angle);
}

/* ------------------------------------------------------------------ */
/* prepare_display_command — line clamping, string handling           */
/* ------------------------------------------------------------------ */

static void test_display_command_normal(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 3, "Hello", 0x20);

    TEST_ASSERT_EQUAL(CMD_TYPE_DISPLAY, pkt.command_type);
    TEST_ASSERT_EQUAL(0x20, pkt.sequence_number);

    const display_data_t *d = (const display_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(3, d->line);
    TEST_ASSERT_EQUAL_STRING("Hello", d->message);
    assert_packet_checksum_valid(&pkt);
}

static void test_display_command_clamps_line_over_7(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 10, "Test", 0x21);

    const display_data_t *d = (const display_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(7, d->line);
    assert_packet_checksum_valid(&pkt);
}

static void test_display_command_accepts_line_7(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 7, "Max line", 0x22);

    const display_data_t *d = (const display_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(7, d->line);
}

static void test_display_command_accepts_line_0(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 0, "Min line", 0x23);

    const display_data_t *d = (const display_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(0, d->line);
}

static void test_display_command_truncates_long_message(void)
{
    i2c_command_packet_t pkt = {0};
    /* 30 'A' chars — longer than the message field allows */
    prepare_display_command(&pkt, 0, "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", 0x24);

    const display_data_t *d = (const display_data_t *)pkt.data;
    /* Message must always be null-terminated within its array */
    TEST_ASSERT_EQUAL('\0', d->message[sizeof(d->message) - 1]);
    assert_packet_checksum_valid(&pkt);
}

static void test_display_command_empty_message(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 2, "", 0x25);

    const display_data_t *d = (const display_data_t *)pkt.data;
    TEST_ASSERT_EQUAL('\0', d->message[0]);
}

static void test_display_command_null_message_no_crash(void)
{
    /* Should return early without crashing */
    i2c_command_packet_t pkt = {0};
    prepare_display_command(&pkt, 0, NULL, 0x26);
}

static void test_display_command_null_packet_no_crash(void)
{
    prepare_display_command(NULL, 0, "test", 0x27);
}

/* ------------------------------------------------------------------ */
/* prepare_ping_command                                                */
/* ------------------------------------------------------------------ */

static void test_ping_command(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_ping_command(&pkt, 0xFF);

    TEST_ASSERT_EQUAL(CMD_TYPE_PING, pkt.command_type);
    TEST_ASSERT_EQUAL(0xFF, pkt.sequence_number);
    TEST_ASSERT_EQUAL(0, pkt.data_length);
    assert_packet_checksum_valid(&pkt);
}

static void test_ping_command_null_no_crash(void)
{
    prepare_ping_command(NULL, 0);
}

/* ------------------------------------------------------------------ */
/* OTA commands                                                        */
/* ------------------------------------------------------------------ */

static void test_enter_maintenance_command(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_enter_maintenance_command(&pkt, 0x30);

    TEST_ASSERT_EQUAL(CMD_TYPE_ENTER_MAINTENANCE_MODE, pkt.command_type);
    TEST_ASSERT_EQUAL(0x30, pkt.sequence_number);
    TEST_ASSERT_EQUAL(0, pkt.data_length);
    assert_packet_checksum_valid(&pkt);
}

static void test_begin_ota_command_short_tag(void)
{
    i2c_command_packet_t pkt = {0};
    uint8_t hash[] = {0xAB, 0xCD, 0xEF, 0x12};
    prepare_begin_ota_command(&pkt, "v1.2.3", hash, 0x31);

    TEST_ASSERT_EQUAL(CMD_TYPE_BEGIN_OTA, pkt.command_type);
    TEST_ASSERT_EQUAL(0x31, pkt.sequence_number);

    const ota_begin_data_t *d = (const ota_begin_data_t *)pkt.data;
    TEST_ASSERT_EQUAL(6, d->tag_length); /* strlen("v1.2.3") = 6 */
    TEST_ASSERT_EQUAL_MEMORY(hash, d->hash, OTA_HASH_LEN);
    assert_packet_checksum_valid(&pkt);
}

static void test_begin_ota_command_null_hash_zeroed(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_begin_ota_command(&pkt, "v0.1.0", NULL, 0x32);

    const ota_begin_data_t *d = (const ota_begin_data_t *)pkt.data;
    uint8_t zeros[OTA_HASH_LEN] = {0};
    TEST_ASSERT_EQUAL_MEMORY(zeros, d->hash, OTA_HASH_LEN);
    assert_packet_checksum_valid(&pkt);
}

static void test_begin_ota_command_null_tag_no_crash(void)
{
    i2c_command_packet_t pkt = {0};
    uint8_t hash[] = {0, 0, 0, 0};
    prepare_begin_ota_command(&pkt, NULL, hash, 0x33);
}

static void test_get_ota_status_command(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_get_ota_status_command(&pkt, 0x40);

    TEST_ASSERT_EQUAL(CMD_TYPE_GET_OTA_STATUS, pkt.command_type);
    TEST_ASSERT_EQUAL(0x40, pkt.sequence_number);
    TEST_ASSERT_EQUAL(0, pkt.data_length);
    assert_packet_checksum_valid(&pkt);
}

static void test_get_version_command(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_get_version_command(&pkt, 0x41);

    TEST_ASSERT_EQUAL(CMD_TYPE_GET_VERSION, pkt.command_type);
    TEST_ASSERT_EQUAL(0x41, pkt.sequence_number);
    assert_packet_checksum_valid(&pkt);
}

static void test_reboot_command(void)
{
    i2c_command_packet_t pkt = {0};
    prepare_reboot_command(&pkt, 0x42);

    TEST_ASSERT_EQUAL(CMD_TYPE_REBOOT, pkt.command_type);
    TEST_ASSERT_EQUAL(0x42, pkt.sequence_number);
    TEST_ASSERT_EQUAL(0, pkt.data_length);
    assert_packet_checksum_valid(&pkt);
}

/* ------------------------------------------------------------------ */
/* Packet layout and sizes                                             */
/* ------------------------------------------------------------------ */

static void test_command_packet_size(void)
{
    /* command_type(1) + sequence_number(1) + data_length(1)
     * + data[26](26) + checksum(1) = 30 bytes */
    TEST_ASSERT_EQUAL(30, (int)sizeof(i2c_command_packet_t));
}

static void test_response_packet_size(void)
{
    /* status(1) + sequence_number(1) + data_length(1)
     * + data[13](13) + checksum(1) = 17 bytes */
    TEST_ASSERT_EQUAL(17, (int)sizeof(i2c_response_packet_t));
}

static void test_data_structs_fit_in_max_data_len(void)
{
    TEST_ASSERT_TRUE((int)sizeof(movement_data_t) <= I2C_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(sound_data_t) <= I2C_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(servo_data_t) <= I2C_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(display_data_t) <= I2C_MAX_DATA_LEN);
    TEST_ASSERT_TRUE((int)sizeof(ota_begin_data_t) <= I2C_MAX_DATA_LEN);
}

static void test_sequence_number_stored_correctly(void)
{
    i2c_command_packet_t pkt = {0};

    prepare_ping_command(&pkt, 0xAB);
    TEST_ASSERT_EQUAL(0xAB, pkt.sequence_number);

    prepare_ping_command(&pkt, 0x00);
    TEST_ASSERT_EQUAL(0x00, pkt.sequence_number);

    prepare_ping_command(&pkt, 0xFF);
    TEST_ASSERT_EQUAL(0xFF, pkt.sequence_number);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    /* calculate_checksum */
    RUN_TEST(test_checksum_empty_buffer);
    RUN_TEST(test_checksum_single_byte);
    RUN_TEST(test_checksum_xor_cancels);
    RUN_TEST(test_checksum_multi_byte);
    RUN_TEST(test_checksum_full_byte_range);

    /* verify_checksum */
    RUN_TEST(test_verify_checksum_correct);
    RUN_TEST(test_verify_checksum_wrong);
    RUN_TEST(test_verify_checksum_zero_length);
    RUN_TEST(test_verify_checksum_round_trip);

    /* Movement command */
    RUN_TEST(test_movement_command_fields);
    RUN_TEST(test_movement_command_checksum_valid);
    RUN_TEST(test_movement_command_null_packet);
    RUN_TEST(test_movement_all_commands_round_trip);
    RUN_TEST(test_movement_speed_max);
    RUN_TEST(test_movement_speed_zero);

    /* Sound command */
    RUN_TEST(test_sound_command_beep);
    RUN_TEST(test_sound_command_all_types);

    /* Servo command */
    RUN_TEST(test_servo_command_normal_angle);
    RUN_TEST(test_servo_command_clamps_over_180);
    RUN_TEST(test_servo_command_accepts_180);
    RUN_TEST(test_servo_command_accepts_0);
    RUN_TEST(test_servo_command_extreme_over_180);

    /* Display command */
    RUN_TEST(test_display_command_normal);
    RUN_TEST(test_display_command_clamps_line_over_7);
    RUN_TEST(test_display_command_accepts_line_7);
    RUN_TEST(test_display_command_accepts_line_0);
    RUN_TEST(test_display_command_truncates_long_message);
    RUN_TEST(test_display_command_empty_message);
    RUN_TEST(test_display_command_null_message_no_crash);
    RUN_TEST(test_display_command_null_packet_no_crash);

    /* Ping command */
    RUN_TEST(test_ping_command);
    RUN_TEST(test_ping_command_null_no_crash);

    /* OTA commands */
    RUN_TEST(test_enter_maintenance_command);
    RUN_TEST(test_begin_ota_command_short_tag);
    RUN_TEST(test_begin_ota_command_null_hash_zeroed);
    RUN_TEST(test_begin_ota_command_null_tag_no_crash);
    RUN_TEST(test_get_ota_status_command);
    RUN_TEST(test_get_version_command);
    RUN_TEST(test_reboot_command);

    /* Packet layout */
    RUN_TEST(test_command_packet_size);
    RUN_TEST(test_response_packet_size);
    RUN_TEST(test_data_structs_fit_in_max_data_len);
    RUN_TEST(test_sequence_number_stored_correctly);

    int failures = UNITY_END();
    return (failures > 0) ? 1 : 0;
}
