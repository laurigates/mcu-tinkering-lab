// Host unit tests for param_record.c: v5 round trip and the migrations from
// v1, v2, v3 and v4 (no Pico SDK). The v1, v2 and v3 blobs are built byte by byte
// from their own layouts (v1: magic, version 1, seven floats, CRC32 over the
// first 36 bytes; v2: magic, version 2, sixteen four-byte fields, CRC32 over
// the first 72 bytes; v3: magic, version 3, seventeen four-byte fields, CRC32
// over the first 76 bytes; v4: magic, version 4, eighteen four-byte fields,
// CRC32 over the first 80 bytes), not from today's struct, so a change to
// balance_params_t cannot make the test agree with itself.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "crc32.h"
#include "param_record.h"

static void put32(uint8_t *buf, size_t off, uint32_t v)
{
    memcpy(buf + off, &v, 4);
}

static void putf(uint8_t *buf, size_t off, float v)
{
    memcpy(buf + off, &v, 4);
}

// Saved PID gains, deliberately unlike the defaults.
static const float V1_FLOATS[7] = {41.5f, 0.25f, 2.75f, 0.006f, 0.0007f, 0.97f, 3500.0f};

static size_t build_v1(uint8_t *buf)
{
    memset(buf, 0xFF, 256);
    put32(buf, 0, 0x42414C42u);  // "BALB"
    put32(buf, 4, 1u);
    for (int i = 0; i < 7; i++) {
        putf(buf, 8u + 4u * (size_t)i, V1_FLOATS[i]);
    }
    put32(buf, 36, crc32_calc(buf, 36));
    return 40;
}

// A saved v2 record: PID gains, LQR mode and model, all unlike the defaults.
// Field 7 is ctrl_mode (uint32), the rest are floats.
static const float V2_FLOATS[16] = {41.5f, 0.25f, 2.75f,   0.006f, 0.0007f, 0.97f, 3500.0f, 0.0f,
                                    0.17f, 0.1f,  3200.0f, 300.0f, 2.0f,    16.0f, 3.0f,    0.08f};

static size_t build_v2(uint8_t *buf)
{
    memset(buf, 0xFF, 256);
    put32(buf, 0, 0x42414C42u);  // "BALB"
    put32(buf, 4, 2u);
    for (int i = 0; i < 16; i++) {
        putf(buf, 8u + 4u * (size_t)i, V2_FLOATS[i]);
    }
    put32(buf, 8u + 4u * 7u, 1u);  // ctrl_mode = LQR
    put32(buf, 72, crc32_calc(buf, 72));
    return 76;
}

// A saved v3 record: the v2 fields above plus imu_mount = down (field 16).
static size_t build_v3(uint8_t *buf)
{
    memset(buf, 0xFF, 256);
    put32(buf, 0, 0x42414C42u);  // "BALB"
    put32(buf, 4, 3u);
    for (int i = 0; i < 16; i++) {
        putf(buf, 8u + 4u * (size_t)i, V2_FLOATS[i]);
    }
    put32(buf, 8u + 4u * 7u, 1u);   // ctrl_mode = LQR
    put32(buf, 8u + 4u * 16u, 1u);  // imu_mount = down
    put32(buf, 76, crc32_calc(buf, 76));
    return 80;
}

// A saved v4 record: the v3 fields plus pitch_trim_deg = -1.5 (field 17).
static size_t build_v4(uint8_t *buf)
{
    memset(buf, 0xFF, 256);
    put32(buf, 0, 0x42414C42u);  // "BALB"
    put32(buf, 4, 4u);
    for (int i = 0; i < 16; i++) {
        putf(buf, 8u + 4u * (size_t)i, V2_FLOATS[i]);
    }
    put32(buf, 8u + 4u * 7u, 1u);   // ctrl_mode = LQR
    put32(buf, 8u + 4u * 16u, 1u);  // imu_mount = down
    putf(buf, 8u + 4u * 17u, -1.5f);
    put32(buf, 80, crc32_calc(buf, 80));
    return 84;
}

static void test_layout_sizes(void)
{
    assert(PARAM_V1_SIZE == 40u);
    assert(PARAM_V2_SIZE == 4u + 4u + 64u + 4u);
    assert(PARAM_V3_SIZE == 4u + 4u + 68u + 4u);
    assert(PARAM_V4_SIZE == 4u + 4u + 72u + 4u);
    assert(PARAM_V5_SIZE == 4u + 4u + 80u + 4u);
}

static void test_v4_migrates_keeping_fields(void)
{
    uint8_t buf[256];
    build_v4(buf);
    balance_params_t p;
    memset(&p, 0, sizeof p);
    uint32_t ver = 0;
    assert(param_record_decode(buf, sizeof buf, &p, &ver));
    assert(ver == 4u);
    assert(p.angle_kp == V2_FLOATS[0]);
    assert(p.ctrl_mode == CTRL_LQR);
    assert(p.imu_mount == IMU_MOUNT_DOWN);
    assert(p.pitch_trim_deg == -1.5f);
    // v4 firmware did not read the pack: a robot saved before keeps the
    // delivered boards' 27 kOhm divider and 3S, not the 0xFF bytes after the
    // v4 CRC (a NaN as a float).
    assert(p.pack_r_low_kohm == 27.0f);
    assert(p.pack_cells == 3.0f);
}

static void test_v4_bad_crc_rejected(void)
{
    uint8_t buf[256];
    build_v4(buf);
    buf[8u + 4u * 17u] ^= 0x01;  // flip a bit inside the saved trim
    balance_params_t p;
    memset(&p, 0xAB, sizeof p);
    balance_params_t before = p;
    assert(!param_record_decode(buf, sizeof buf, &p, NULL));
    assert(memcmp(&p, &before, sizeof p) == 0);
    build_v4(buf);
    assert(!param_record_decode(buf, PARAM_V4_SIZE - 1u, &p, NULL));
}

static void test_pack_defaults(void)
{
    balance_params_t d;
    param_defaults(&d);
    assert(d.pack_r_low_kohm == 27.0f);  // the delivered boards (8d18cd8d layout)
    assert(d.pack_cells == 3.0f);
}

static void test_v1_migrates_keeping_pid_gains(void)
{
    uint8_t buf[256];
    build_v1(buf);
    balance_params_t p;
    memset(&p, 0, sizeof p);
    uint32_t ver = 0;
    assert(param_record_decode(buf, sizeof buf, &p, &ver));
    assert(ver == 1u);
    assert(p.angle_kp == V1_FLOATS[0]);
    assert(p.angle_ki == V1_FLOATS[1]);
    assert(p.angle_kd == V1_FLOATS[2]);
    assert(p.vel_kp == V1_FLOATS[3]);
    assert(p.vel_ki == V1_FLOATS[4]);
    assert(p.filter_alpha == V1_FLOATS[5]);
    assert(p.max_rate_sps == V1_FLOATS[6]);
    // Every new field takes its default; a v1 robot boots on the PID.
    assert(p.ctrl_mode == CTRL_PID);
    assert(p.l_eff_m == DEFAULT_L_EFF_M);
    assert(p.wheel_diam_m == DEFAULT_WHEEL_DIAM_M);
    assert(p.steps_per_rev == DEFAULT_STEPS_PER_REV);
    assert(p.q_theta == DEFAULT_Q_THETA);
    assert(p.q_dtheta == DEFAULT_Q_DTHETA);
    assert(p.q_x == DEFAULT_Q_X);
    assert(p.q_dx == DEFAULT_Q_DX);
    assert(p.r == DEFAULT_R);
    assert(p.imu_mount == IMU_MOUNT_UP);
    assert(p.pitch_trim_deg == 0.0f);
}

static void test_v2_migrates_keeping_fields(void)
{
    uint8_t buf[256];
    build_v2(buf);
    balance_params_t p;
    memset(&p, 0, sizeof p);
    uint32_t ver = 0;
    assert(param_record_decode(buf, sizeof buf, &p, &ver));
    assert(ver == 2u);
    assert(p.angle_kp == V2_FLOATS[0]);
    assert(p.angle_ki == V2_FLOATS[1]);
    assert(p.angle_kd == V2_FLOATS[2]);
    assert(p.vel_kp == V2_FLOATS[3]);
    assert(p.vel_ki == V2_FLOATS[4]);
    assert(p.filter_alpha == V2_FLOATS[5]);
    assert(p.max_rate_sps == V2_FLOATS[6]);
    assert(p.ctrl_mode == CTRL_LQR);
    assert(p.l_eff_m == V2_FLOATS[8]);
    assert(p.wheel_diam_m == V2_FLOATS[9]);
    assert(p.steps_per_rev == V2_FLOATS[10]);
    assert(p.q_theta == V2_FLOATS[11]);
    assert(p.q_dtheta == V2_FLOATS[12]);
    assert(p.q_x == V2_FLOATS[13]);
    assert(p.q_dx == V2_FLOATS[14]);
    assert(p.r == V2_FLOATS[15]);
    // v2 firmware assumed component side up; a v2 record keeps that, even
    // though the bytes after the v2 CRC are 0xFF (erased flash).
    assert(p.imu_mount == IMU_MOUNT_UP);
    assert(p.pitch_trim_deg == 0.0f);
}

static void test_v3_migrates_keeping_fields(void)
{
    uint8_t buf[256];
    build_v3(buf);
    balance_params_t p;
    memset(&p, 0, sizeof p);
    uint32_t ver = 0;
    assert(param_record_decode(buf, sizeof buf, &p, &ver));
    assert(ver == 3u);
    assert(p.angle_kp == V2_FLOATS[0]);
    assert(p.max_rate_sps == V2_FLOATS[6]);
    assert(p.ctrl_mode == CTRL_LQR);
    assert(p.l_eff_m == V2_FLOATS[8]);
    assert(p.r == V2_FLOATS[15]);
    assert(p.imu_mount == IMU_MOUNT_DOWN);
    // v3 firmware applied no trim; the 0xFF bytes after the v3 CRC (a NaN as
    // a float) must not become one.
    assert(p.pitch_trim_deg == 0.0f);
}

static void test_v3_bad_crc_rejected(void)
{
    uint8_t buf[256];
    build_v3(buf);
    buf[8u + 4u * 16u] ^= 0x01;  // flip a bit inside the saved imu_mount
    balance_params_t p;
    memset(&p, 0xAB, sizeof p);
    balance_params_t before = p;
    assert(!param_record_decode(buf, sizeof buf, &p, NULL));
    assert(memcmp(&p, &before, sizeof p) == 0);
    build_v3(buf);
    assert(!param_record_decode(buf, PARAM_V3_SIZE - 1u, &p, NULL));
}

static void test_v2_bad_crc_rejected(void)
{
    uint8_t buf[256];
    build_v2(buf);
    buf[40] ^= 0x01;  // flip a bit inside the saved l_eff_m
    balance_params_t p;
    memset(&p, 0xAB, sizeof p);
    balance_params_t before = p;
    assert(!param_record_decode(buf, sizeof buf, &p, NULL));
    assert(memcmp(&p, &before, sizeof p) == 0);
    // A v2 record is 76 bytes; one short is rejected.
    build_v2(buf);
    assert(!param_record_decode(buf, PARAM_V2_SIZE - 1u, &p, NULL));
}

static void test_v1_bad_crc_rejected(void)
{
    uint8_t buf[256];
    build_v1(buf);
    buf[12] ^= 0x01;  // flip a bit inside the saved kd
    balance_params_t p;
    memset(&p, 0xAB, sizeof p);
    balance_params_t before = p;
    assert(!param_record_decode(buf, sizeof buf, &p, NULL));
    assert(memcmp(&p, &before, sizeof p) == 0);  // untouched on failure
}

static void test_v5_round_trip(void)
{
    balance_params_t in;
    param_defaults(&in);
    in.angle_kp = 55.0f;
    in.ctrl_mode = CTRL_LQR;
    in.l_eff_m = 0.17f;
    in.q_x = 9.0f;
    in.r = 0.5f;
    in.imu_mount = IMU_MOUNT_DOWN;
    in.pitch_trim_deg = -2.5f;
    in.pack_r_low_kohm = 22.0f;
    in.pack_cells = 4.0f;
    uint8_t buf[256];
    memset(buf, 0xFF, sizeof buf);
    assert(param_record_encode(&in, buf, sizeof buf) == PARAM_V5_SIZE);
    balance_params_t out;
    uint32_t ver = 0;
    assert(param_record_decode(buf, sizeof buf, &out, &ver));
    assert(ver == PARAM_VERSION);
    assert(memcmp(&in, &out, sizeof in) == 0);
}

static void test_encode_needs_room(void)
{
    balance_params_t in;
    param_defaults(&in);
    uint8_t buf[PARAM_V5_SIZE];
    assert(param_record_encode(&in, buf, sizeof buf - 1u) == 0);
    assert(param_record_encode(&in, buf, sizeof buf) == PARAM_V5_SIZE);
}

static void test_rejects_bad_framing(void)
{
    balance_params_t in;
    param_defaults(&in);
    uint8_t buf[256];
    balance_params_t out;

    // Erased flash.
    memset(buf, 0xFF, sizeof buf);
    assert(!param_record_decode(buf, sizeof buf, &out, NULL));

    // Unknown (future) version with an otherwise valid CRC.
    param_record_encode(&in, buf, sizeof buf);
    put32(buf, 4, 6u);
    put32(buf, PARAM_V5_SIZE - 4u, crc32_calc(buf, PARAM_V5_SIZE - 4u));
    assert(!param_record_decode(buf, sizeof buf, &out, NULL));

    // v5 payload corruption.
    param_record_encode(&in, buf, sizeof buf);
    buf[20] ^= 0x80;
    assert(!param_record_decode(buf, sizeof buf, &out, NULL));

    // Truncated buffer.
    param_record_encode(&in, buf, sizeof buf);
    assert(!param_record_decode(buf, PARAM_V5_SIZE - 1u, &out, NULL));
}

static void test_pitch_trim_valid(void)
{
    assert(pitch_trim_valid(0.0f));
    assert(pitch_trim_valid(PITCH_TRIM_MAX_DEG));
    assert(pitch_trim_valid(-PITCH_TRIM_MAX_DEG));
    assert(!pitch_trim_valid(PITCH_TRIM_MAX_DEG + 0.01f));
    assert(!pitch_trim_valid(-PITCH_TRIM_MAX_DEG - 0.01f));
    assert(!pitch_trim_valid(NAN));
    assert(!pitch_trim_valid(INFINITY));
}

// A v5 record with a valid CRC but values the solver or unit conversion would
// reject: each bad field falls back to its default, ctrl_mode to PID.
static void test_v5_out_of_range_fields_defaulted(void)
{
    balance_params_t in;
    param_defaults(&in);
    in.angle_kp = 33.0f;
    in.ctrl_mode = 7u;
    in.l_eff_m = -0.1f;
    in.wheel_diam_m = 0.0f;
    in.steps_per_rev = NAN;
    in.q_x = -1.0f;
    in.q_dx = INFINITY;
    in.r = 0.0f;
    in.q_theta = 123.0f;  // valid: kept
    in.imu_mount = 2u;
    in.pitch_trim_deg = 12.0f;
    in.pack_r_low_kohm = 0.0f;
    in.pack_cells = 3.5f;
    uint8_t buf[256];
    param_record_encode(&in, buf, sizeof buf);
    balance_params_t out;
    assert(param_record_decode(buf, sizeof buf, &out, NULL));
    assert(out.angle_kp == 33.0f);
    assert(out.ctrl_mode == CTRL_PID);
    assert(out.l_eff_m == DEFAULT_L_EFF_M);
    assert(out.wheel_diam_m == DEFAULT_WHEEL_DIAM_M);
    assert(out.steps_per_rev == DEFAULT_STEPS_PER_REV);
    assert(out.q_x == DEFAULT_Q_X);
    assert(out.q_dx == DEFAULT_Q_DX);
    assert(out.r == DEFAULT_R);
    assert(out.q_theta == 123.0f);
    assert(out.imu_mount == IMU_MOUNT_UP);
    assert(out.pitch_trim_deg == DEFAULT_PITCH_TRIM_DEG);
    assert(out.pack_r_low_kohm == DEFAULT_PACK_R_LOW_KOHM);
    assert(out.pack_cells == DEFAULT_PACK_CELLS);
}

int main(void)
{
    test_layout_sizes();
    test_v1_migrates_keeping_pid_gains();
    test_v1_bad_crc_rejected();
    test_v2_migrates_keeping_fields();
    test_v2_bad_crc_rejected();
    test_v3_migrates_keeping_fields();
    test_v3_bad_crc_rejected();
    test_v4_migrates_keeping_fields();
    test_v4_bad_crc_rejected();
    test_pack_defaults();
    test_v5_round_trip();
    test_encode_needs_room();
    test_rejects_bad_framing();
    test_pitch_trim_valid();
    test_v5_out_of_range_fields_defaulted();
    printf("test_param_record: all tests passed\n");
    return 0;
}
