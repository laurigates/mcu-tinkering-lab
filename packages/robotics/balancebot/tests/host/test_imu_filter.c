// Host unit tests for imu_filter.c (no Pico SDK).
#include <assert.h>
#include <math.h>
#include <stdint.h>
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

// What the accelerometer's X and Z read for a body tilt `theta_rad` (forward
// positive), with the sensor turned `mount_rad` about Y relative to the body.
// Built from the rotation, not from imu_mount_accel(), so the test cannot agree
// with itself: the body reads (sin, cos) on (forward, up); a sensor turned by
// phi about Y reads x' = x cos(phi) + z sin(phi), z' = -x sin(phi) + z cos(phi).
typedef struct {
    float x;
    float z;
} accel_xz_t;

static accel_xz_t sensor_accel(double theta_rad, double mount_rad)
{
    const double fwd = sin(theta_rad);
    const double up = cos(theta_rad);
    return (accel_xz_t){
        .x = (float)(fwd * cos(mount_rad) + up * sin(mount_rad)),
        .z = (float)(-fwd * sin(mount_rad) + up * cos(mount_rad)),
    };
}

static const double PI = 3.14159265358979323846;
static const double DEG = 3.14159265358979323846 / 180.0;

static void test_mount_up_is_identity(void)
{
    imu_body_accel_t b = imu_mount_accel(IMU_MOUNT_UP, 0.25f, 0.9f);
    assert(b.forward_g == 0.25f && b.up_g == 0.9f);
    // An unknown value maps as component side up.
    b = imu_mount_accel(7u, 0.25f, 0.9f);
    assert(b.forward_g == 0.25f && b.up_g == 0.9f);
}

// Why IMU_PITCH_SIGN cannot absorb the flip: unmapped, a level board fitted
// component side down reads about 180 degrees, not 0.
static void test_flipped_unmapped_reads_180(void)
{
    accel_xz_t a = sensor_accel(0.0, PI);
    assert(fabsf(fabsf(imu_filter_accel_pitch_deg(a.x, a.z)) - 180.0f) < 1e-3f);
}

// The same physical motion seen by a board fitted component side up and one
// fitted component side down (turned 180 degrees about Y) gives the same
// pitch once each is mapped with its own mounting, tick for tick. Gyro Y is
// the rotation axis, so both boards read the same rate.
static void test_flipped_mount_matches_normal(void)
{
    imu_filter_t up;
    imu_filter_t down;
    imu_filter_init(&up, 0.98f);
    imu_filter_init(&down, 0.98f);
    const float dt = 0.002f;
    double theta_deg = 3.0;  // start leaning forward
    for (int i = 0; i < 1000; i++) {
        // Swing forward then back through level: positive, then negative.
        const double rate_dps = (i < 500) ? 20.0 : -40.0;
        theta_deg += rate_dps * dt;
        accel_xz_t au = sensor_accel(theta_deg * DEG, 0.0);
        accel_xz_t ad = sensor_accel(theta_deg * DEG, PI);
        imu_body_accel_t bu = imu_mount_accel(IMU_MOUNT_UP, au.x, au.z);
        imu_body_accel_t bd = imu_mount_accel(IMU_MOUNT_DOWN, ad.x, ad.z);
        float pu = imu_filter_update(&up, bu.forward_g, bu.up_g, (float)rate_dps, dt);
        float pd = imu_filter_update(&down, bd.forward_g, bd.up_g, (float)rate_dps, dt);
        assert(fabsf(pu - pd) < 1e-3f);
        if (i == 0) {
            assert(fabsf(pd - (float)theta_deg) < 1e-3f);  // snaps to the true tilt
        }
    }
    // 1 s at +20 deg/s, then 1 s at -40 deg/s: ended leaning back at
    // 3 + 20 - 40 = -17 degrees, and the flipped board reads it with the
    // forward-positive sign.
    assert(fabsf(down.pitch_deg - (-17.0f)) < 0.05f);
}

// assert() aborts without flushing a redirected stdout, so print and flush
// the failing case's name first (.claude/rules/testing.md).
static void check(int cond, const char *what)
{
    if (!cond) {
        printf("FAIL: %s\n", what);
        fflush(stdout);
    }
    assert(cond);
}

// The gravity check for a robot tilted `theta_deg` (forward positive) with the
// GY-521 turned `fitted_rad` about Y, read with the `mount` setting. The
// sample comes from the physical rotation (sensor_accel), scaled by `accel_g`
// (1 = gravity alone) and with `rate_dps` about the pitch axis.
static imu_gravity_check_t gravity_for(uint32_t mount, double theta_deg, double fitted_rad,
                                       float accel_g, float rate_dps)
{
    accel_xz_t a = sensor_accel(theta_deg * DEG, fitted_rad);
    const float accel[3] = {accel_g * a.x, 0.0f, accel_g * a.z};
    const float gyro[3] = {0.0f, rate_dps, 0.0f};
    return imu_gravity_check(mount, accel, gyro);
}

static void test_gravity_mount_up_upright_allowed(void)
{
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_UP, 0.0, 0.0, 1.0f, 0.0f);
    check(g.still, "up/upright: still");
    check(!g.upside_down, "up/upright: not upside down");
    check(fabsf(g.up_g - 1.0f) < 1e-4f, "up/upright: body Z = +1 g");
    check(!imu_gravity_refuses_arm(&g), "up/upright: arm allowed");
}

// The delivered 8d18cd8d boards: GY-521 fitted face down, setting left at up.
static void test_gravity_mount_up_face_down_refused(void)
{
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_UP, 0.0, PI, 1.0f, 0.0f);
    check(g.still, "up/face down: still");
    check(g.upside_down, "up/face down: upside down");
    check(fabsf(g.up_g + 1.0f) < 1e-4f, "up/face down: body Z = -1 g");
    check(imu_gravity_refuses_arm(&g), "up/face down: arm refused");
}

static void test_gravity_mount_down_face_down_allowed(void)
{
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_DOWN, 0.0, PI, 1.0f, 0.0f);
    check(g.still, "down/face down: still");
    check(!g.upside_down, "down/face down: not upside down");
    check(fabsf(g.up_g - 1.0f) < 1e-4f, "down/face down: body Z = +1 g");
    check(!imu_gravity_refuses_arm(&g), "down/face down: arm allowed");
}

// The reverse mistake: `mount down` on a board fitted face up.
static void test_gravity_mount_down_face_up_refused(void)
{
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_DOWN, 0.0, 0.0, 1.0f, 0.0f);
    check(imu_gravity_refuses_arm(&g), "down/face up: arm refused");
}

// Upside down but moving says nothing about the mounting: not refused here.
static void test_gravity_moving_not_refused(void)
{
    // Being turned at 60 deg/s.
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_UP, 0.0, PI, 1.0f, 60.0f);
    check(!g.still, "turning: not still");
    check(g.upside_down, "turning: still reads upside down");
    check(!imu_gravity_refuses_arm(&g), "turning: arm not refused");
    // Shaken: 1.5 g on the accel, gyro quiet this sample.
    g = gravity_for(IMU_MOUNT_UP, 0.0, PI, 1.5f, 0.0f);
    check(!g.still, "shaken: not still");
    check(!imu_gravity_refuses_arm(&g), "shaken: arm not refused");
    // Falling freely: well under 1 g.
    g = gravity_for(IMU_MOUNT_UP, 0.0, PI, 0.3f, 0.0f);
    check(!g.still, "free fall: not still");
    check(!imu_gravity_refuses_arm(&g), "free fall: arm not refused");
}

// A robot resting on its stand at a large lean is still the right way up.
static void test_gravity_tilted_resting_allowed(void)
{
    imu_gravity_check_t g = gravity_for(IMU_MOUNT_UP, 50.0, 0.0, 1.0f, 0.0f);
    check(g.still, "tilted 50: still");
    check(!g.upside_down, "tilted 50: not upside down");
    check(!imu_gravity_refuses_arm(&g), "tilted 50: arm allowed");
    g = gravity_for(IMU_MOUNT_DOWN, -50.0, PI, 1.0f, 0.0f);
    check(!imu_gravity_refuses_arm(&g), "tilted -50, flipped board: arm allowed");
    // Past horizontal but short of 120 degrees (body Z = -0.34 g): fallen
    // over, which the tilt cutoff handles, not a wrong mounting.
    g = gravity_for(IMU_MOUNT_UP, 110.0, 0.0, 1.0f, 0.0f);
    check(g.still && !g.upside_down, "tilted 110: not upside down");
    check(!imu_gravity_refuses_arm(&g), "tilted 110: arm not refused");
    // Beyond 120 degrees (body Z = -0.64 g) at rest reads as upside down.
    g = gravity_for(IMU_MOUNT_UP, 130.0, 0.0, 1.0f, 0.0f);
    check(imu_gravity_refuses_arm(&g), "tilted 130: arm refused");
}

int main(void)
{
    test_accel_pitch();
    test_first_sample_snaps();
    test_gyro_integration();
    test_accel_correction_pulls_drift();
    test_reset();
    test_mount_up_is_identity();
    test_flipped_unmapped_reads_180();
    test_flipped_mount_matches_normal();
    test_gravity_mount_up_upright_allowed();
    test_gravity_mount_up_face_down_refused();
    test_gravity_mount_down_face_down_allowed();
    test_gravity_mount_down_face_up_refused();
    test_gravity_moving_not_refused();
    test_gravity_tilted_resting_allowed();
    printf("test_imu_filter: all tests passed\n");
    return 0;
}
