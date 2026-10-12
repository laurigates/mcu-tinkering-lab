// Host unit tests for param_record.c: v2 round trip and the v1 -> v2
// migration (no Pico SDK). The v1 blob is built byte by byte from the v1
// layout (magic, version 1, seven floats, CRC32 over the first 36 bytes), not
// from today's struct, so a change to balance_params_t cannot make the test
// agree with itself.
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

static void test_layout_sizes(void)
{
    assert(PARAM_V1_SIZE == 40u);
    assert(PARAM_V2_SIZE == 4u + 4u + 64u + 4u);
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

static void test_v2_round_trip(void)
{
    balance_params_t in;
    param_defaults(&in);
    in.angle_kp = 55.0f;
    in.ctrl_mode = CTRL_LQR;
    in.l_eff_m = 0.17f;
    in.q_x = 9.0f;
    in.r = 0.5f;
    uint8_t buf[256];
    memset(buf, 0xFF, sizeof buf);
    assert(param_record_encode(&in, buf, sizeof buf) == PARAM_V2_SIZE);
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
    uint8_t buf[PARAM_V2_SIZE];
    assert(param_record_encode(&in, buf, sizeof buf - 1u) == 0);
    assert(param_record_encode(&in, buf, sizeof buf) == PARAM_V2_SIZE);
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
    put32(buf, 4, 3u);
    put32(buf, PARAM_V2_SIZE - 4u, crc32_calc(buf, PARAM_V2_SIZE - 4u));
    assert(!param_record_decode(buf, sizeof buf, &out, NULL));

    // v2 payload corruption.
    param_record_encode(&in, buf, sizeof buf);
    buf[20] ^= 0x80;
    assert(!param_record_decode(buf, sizeof buf, &out, NULL));

    // Truncated buffer.
    param_record_encode(&in, buf, sizeof buf);
    assert(!param_record_decode(buf, PARAM_V2_SIZE - 1u, &out, NULL));
}

// A v2 record with a valid CRC but values the solver or unit conversion would
// reject: each bad field falls back to its default, ctrl_mode to PID.
static void test_v2_out_of_range_fields_defaulted(void)
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
}

int main(void)
{
    test_layout_sizes();
    test_v1_migrates_keeping_pid_gains();
    test_v1_bad_crc_rejected();
    test_v2_round_trip();
    test_encode_needs_room();
    test_rejects_bad_framing();
    test_v2_out_of_range_fields_defaulted();
    printf("test_param_record: all tests passed\n");
    return 0;
}
