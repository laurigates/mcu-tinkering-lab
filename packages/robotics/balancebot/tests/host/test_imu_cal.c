// Host unit tests for imu_cal.c: the gyro calibration's stillness judgement
// and the pitch-trim capture (no Pico SDK).
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "imu_cal.h"

static const float LEVEL_G[3] = {0.0f, 0.0f, 1.0f};

// A still window: a 4 deg/s bias on Y with +/-0.2 deg/s noise.
static void feed_still(gyro_cal_t *c, int n)
{
    for (int i = 0; i < n; i++) {
        const float wobble = (i % 2 == 0) ? 0.2f : -0.2f;
        const float g[3] = {-1.0f + wobble, 4.0f - wobble, 0.5f};
        gyro_cal_add(c, LEVEL_G, g);
    }
}

static void test_gyro_cal_still_applies_mean(void)
{
    gyro_cal_t c;
    gyro_cal_init(&c);
    feed_still(&c, 500);
    float bias[3] = {9.0f, 9.0f, 9.0f};
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_OK);
    assert(fabsf(bias[0] - -1.0f) < 1e-4f);
    assert(fabsf(bias[1] - 4.0f) < 1e-4f);
    assert(fabsf(bias[2] - 0.5f) < 1e-4f);
}

// A bias of tens of deg/s is not motion: only the spread counts.
static void test_gyro_cal_large_bias_is_still(void)
{
    gyro_cal_t c;
    gyro_cal_init(&c);
    for (int i = 0; i < 500; i++) {
        const float g[3] = {25.0f, -30.0f, 18.0f};
        gyro_cal_add(&c, LEVEL_G, g);
    }
    float bias[3] = {0};
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_OK);
    assert(bias[1] == -30.0f);
}

static void test_gyro_cal_rotation_rejected(void)
{
    gyro_cal_t c;
    gyro_cal_init(&c);
    feed_still(&c, 400);
    // A hand turning the robot: one axis swings past the spread limit.
    const float turn[3] = {-1.0f, 4.0f + GYRO_CAL_MAX_SPREAD_DPS + 1.0f, 0.5f};
    gyro_cal_add(&c, LEVEL_G, turn);
    float bias[3] = {7.0f, 7.0f, 7.0f};
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_MOVED);
    assert(bias[0] == 7.0f && bias[1] == 7.0f && bias[2] == 7.0f);  // untouched
}

static void test_gyro_cal_shake_rejected(void)
{
    gyro_cal_t c;
    gyro_cal_init(&c);
    feed_still(&c, 400);
    // Linear acceleration with no rotation: accel magnitude jumps.
    const float bump[3] = {0.0f, 0.0f, 1.0f + GYRO_CAL_MAX_ACCEL_SPREAD_G + 0.05f};
    const float g[3] = {-1.0f, 4.0f, 0.5f};
    gyro_cal_add(&c, bump, g);
    float bias[3] = {7.0f, 7.0f, 7.0f};
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_MOVED);
    assert(bias[1] == 7.0f);
}

static void test_gyro_cal_too_few_samples(void)
{
    gyro_cal_t c;
    gyro_cal_init(&c);
    float bias[3] = {7.0f, 7.0f, 7.0f};
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_NO_DATA);
    feed_still(&c, 250);  // exactly half read: not enough
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_NO_DATA);
    feed_still(&c, 1);
    assert(gyro_cal_finish(&c, 500, bias) == IMU_CAL_OK);
}

static void test_trim_capture_adds_residual_to_current_trim(void)
{
    trim_capture_t t;
    trim_capture_init(&t);
    for (int i = 0; i < 200; i++) {
        trim_capture_add(&t, (i % 2 == 0) ? 1.7f : 1.9f, true, true);
    }
    float trim = 99.0f;
    // With 0.5 deg already trimmed, the pose reads 1.8: the new trim is 2.3.
    assert(trim_capture_finish(&t, 0.5f, &trim) == IMU_CAL_OK);
    assert(fabsf(trim - 2.3f) < 1e-4f);
}

static void test_trim_capture_moved_rejected(void)
{
    trim_capture_t t;
    trim_capture_init(&t);
    for (int i = 0; i < 200; i++) {
        trim_capture_add(&t, 1.8f, true, i != 100);
    }
    float trim = 99.0f;
    assert(trim_capture_finish(&t, 0.0f, &trim) == IMU_CAL_MOVED);
    assert(trim == 99.0f);
}

static void test_trim_capture_skips_failed_reads(void)
{
    trim_capture_t t;
    trim_capture_init(&t);
    for (int i = 0; i < 200; i++) {
        // Every fourth read fails; its pitch (stale) must not count.
        trim_capture_add(&t, (i % 4 == 0) ? 40.0f : 1.0f, i % 4 != 0, true);
    }
    float trim = 99.0f;
    assert(trim_capture_finish(&t, 0.0f, &trim) == IMU_CAL_OK);
    assert(fabsf(trim - 1.0f) < 1e-4f);

    trim_capture_init(&t);
    for (int i = 0; i < 200; i++) {
        trim_capture_add(&t, 1.0f, false, true);
    }
    assert(trim_capture_finish(&t, 0.0f, &trim) == IMU_CAL_NO_DATA);
}

static void test_trim_capture_out_of_range(void)
{
    trim_capture_t t;
    trim_capture_init(&t);
    for (int i = 0; i < 200; i++) {
        trim_capture_add(&t, 6.0f, true, true);
    }
    float trim = 99.0f;
    // 6 read on top of a 5 deg trim is 11, past PITCH_TRIM_MAX_DEG.
    assert(trim_capture_finish(&t, 5.0f, &trim) == IMU_CAL_RANGE);
    assert(trim == 99.0f);
    assert(trim_capture_finish(&t, 4.0f, &trim) == IMU_CAL_OK);
    assert(trim == PITCH_TRIM_MAX_DEG);
}

int main(void)
{
    test_gyro_cal_still_applies_mean();
    test_gyro_cal_large_bias_is_still();
    test_gyro_cal_rotation_rejected();
    test_gyro_cal_shake_rejected();
    test_gyro_cal_too_few_samples();
    test_trim_capture_adds_residual_to_current_trim();
    test_trim_capture_moved_rejected();
    test_trim_capture_skips_failed_reads();
    test_trim_capture_out_of_range();
    printf("test_imu_cal: all tests passed\n");
    return 0;
}
