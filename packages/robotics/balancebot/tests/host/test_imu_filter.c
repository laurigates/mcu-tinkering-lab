// Host unit tests for imu_filter.c (no Pico SDK).
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "imu_filter.h"

static void test_accel_pitch(void)
{
    // Level: gravity entirely on the up axis.
    assert(fabsf(imu_filter_accel_pitch_deg(0.0f, 1.0f)) < 1e-4f);
    // 45 degrees: equal forward and up components.
    assert(fabsf(imu_filter_accel_pitch_deg(1.0f, 1.0f) - 45.0f) < 1e-3f);
    // Sign follows the forward axis.
    assert(imu_filter_accel_pitch_deg(-0.5f, 1.0f) < 0.0f);
}

static void test_first_sample_snaps(void)
{
    imu_filter_t f;
    imu_filter_init(&f, 0.98f);
    float pitch = imu_filter_update(&f, 1.0f, 1.0f, 0.0f, 0.002f);
    assert(fabsf(pitch - 45.0f) < 1e-3f);
}

static void test_gyro_integration(void)
{
    imu_filter_t f;
    imu_filter_init(&f, 1.0f);  // pure gyro (alpha = 1)
    imu_filter_update(&f, 0.0f, 1.0f, 0.0f, 0.002f);
    // 100 dps for 0.5 s => 50 degrees.
    float pitch = 0.0f;
    for (int i = 0; i < 250; i++) {
        pitch = imu_filter_update(&f, 0.0f, 1.0f, 100.0f, 0.002f);
    }
    assert(fabsf(pitch - 50.0f) < 1e-2f);
}

static void test_accel_correction_pulls_drift(void)
{
    imu_filter_t f;
    imu_filter_init(&f, 0.98f);
    imu_filter_update(&f, 0.0f, 1.0f, 0.0f, 0.002f);
    f.pitch_deg = 10.0f;  // inject drift; accel says level
    float pitch = 10.0f;
    for (int i = 0; i < 2000; i++) {  // 4 s at 500 Hz
        pitch = imu_filter_update(&f, 0.0f, 1.0f, 0.0f, 0.002f);
    }
    assert(fabsf(pitch) < 0.05f);  // converged back to the accel angle
}

static void test_reset(void)
{
    imu_filter_t f;
    imu_filter_init(&f, 0.98f);
    imu_filter_update(&f, 1.0f, 1.0f, 0.0f, 0.002f);
    imu_filter_reset(&f);
    assert(f.initialized == 0);
    float pitch = imu_filter_update(&f, 0.0f, 1.0f, 0.0f, 0.002f);
    assert(fabsf(pitch) < 1e-4f);  // snapped to accel again
}

int main(void)
{
    test_accel_pitch();
    test_first_sample_snaps();
    test_gyro_integration();
    test_accel_correction_pulls_drift();
    test_reset();
    printf("test_imu_filter: all tests passed\n");
    return 0;
}
