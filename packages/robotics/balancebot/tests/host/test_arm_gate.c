// Host unit tests for arm_gate.c (no Pico SDK): core 1's CMD_ARM gravity gate
// and the CMD_SET_IMU_MOUNT acceptance, driven in the order core 1's loop
// runs them: the queue drains at the start of a tick, the IMU is read after.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_gate.h"

static const double PI = 3.14159265358979323846;

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

// A raw sample for a robot standing upright and still, with the GY-521 turned
// `fitted_rad` about Y (the axle). Built from the rotation, as in
// test_imu_filter.c, not from imu_mount_accel(): a sensor turned by phi about
// Y reads x' = x cos(phi) + z sin(phi), z' = -x sin(phi) + z cos(phi), with
// the body reading (0, 1) on (forward, up) when upright.
typedef struct {
    float accel_g[3];
    float gyro_dps[3];
} sample_t;

static sample_t upright_sample(double fitted_rad, float rate_dps)
{
    return (sample_t){
        .accel_g = {(float)sin(fitted_rad), 0.0f, (float)cos(fitted_rad)},
        .gyro_dps = {0.0f, rate_dps, 0.0f},
    };
}

// One tick's drain with 'mount <requested>' and 'arm' queued back to back (a
// paste or a script), as handle_command() runs them: the mount first, then
// CMD_ARM from IDLE under whatever mount that left. Returns the arm decision.
static bool drain_mount_then_arm(arm_gate_t *g, uint32_t *mount, uint32_t requested)
{
    if (arm_gate_mount_accepted(*mount, requested, false)) {
        *mount = requested;
    }
    return arm_gate_allows(g, *mount);
}

// Face-up board, mount up, then 'mount down' + 'arm' in one tick. The check
// published on the last tick (under up) passes; the arm must be judged under
// down, which reads the board upside down, and the refusal is counted.
static void test_same_tick_mount_change_refuses(void)
{
    arm_gate_t g = {0};
    uint32_t mount = IMU_MOUNT_UP;
    const sample_t s = upright_sample(0.0, 0.0f);
    arm_gate_observe(&g, mount, true, s.accel_g, s.gyro_dps);  // last tick
    check(!imu_gravity_refuses_arm(&g.check), "published check under up: no refusal");

    check(!drain_mount_then_arm(&g, &mount, IMU_MOUNT_DOWN), "arm after 'mount down': refused");
    check(mount == IMU_MOUNT_DOWN, "mount applied before the arm");
    check(g.refusals == 1, "refusal counted for core 0");
    check(fabsf(g.refused_up_g - (-1.0f)) < 1e-3f, "refusal carries body Z = -1 g");
}

// The reverse: a face-down board (the 8d18cd8d carrier) with mount up, fixed
// by 'mount down' in the same tick as 'arm'. The published check still says
// upside down; the arm under down must go ahead.
static void test_same_tick_mount_fix_allows(void)
{
    arm_gate_t g = {0};
    uint32_t mount = IMU_MOUNT_UP;
    const sample_t s = upright_sample(PI, 0.0f);
    arm_gate_observe(&g, mount, true, s.accel_g, s.gyro_dps);
    check(imu_gravity_refuses_arm(&g.check), "published check under up: refusal");

    check(drain_mount_then_arm(&g, &mount, IMU_MOUNT_DOWN), "arm after fixing mount: allowed");
    check(g.refusals == 0, "no refusal counted");
}

// Without a mount change the gate agrees with the published check.
static void test_unchanged_mount(void)
{
    arm_gate_t g = {0};
    sample_t s = upright_sample(0.0, 0.0f);
    arm_gate_observe(&g, IMU_MOUNT_UP, true, s.accel_g, s.gyro_dps);
    check(arm_gate_allows(&g, IMU_MOUNT_UP), "face up, mount up: allowed");
    s = upright_sample(PI, 0.0f);
    arm_gate_observe(&g, IMU_MOUNT_UP, true, s.accel_g, s.gyro_dps);
    check(!arm_gate_allows(&g, IMU_MOUNT_UP), "face down, mount up: refused");
    check(!arm_gate_allows(&g, IMU_MOUNT_UP), "and again");
    check(g.refusals == 2, "each refused CMD_ARM counted");
}

// A failed read clears the published check and refuses nothing, even though
// the gate still holds the last good (upside-down) sample.
static void test_failed_read_refuses_nothing(void)
{
    arm_gate_t g = {0};
    const sample_t s = upright_sample(PI, 0.0f);
    arm_gate_observe(&g, IMU_MOUNT_UP, true, s.accel_g, s.gyro_dps);
    arm_gate_observe(&g, IMU_MOUNT_UP, false, s.accel_g, s.gyro_dps);
    check(!g.check.still && !g.check.upside_down, "failed read: check cleared");
    check(arm_gate_allows(&g, IMU_MOUNT_UP), "failed read: arm not refused");
    check(g.refusals == 0, "failed read: nothing counted");
}

// A sample being turned says nothing about the mounting.
static void test_moving_sample_not_refused(void)
{
    arm_gate_t g = {0};
    const sample_t s = upright_sample(PI, 60.0f);
    arm_gate_observe(&g, IMU_MOUNT_UP, true, s.accel_g, s.gyro_dps);
    check(arm_gate_allows(&g, IMU_MOUNT_UP), "turning: arm not refused");
}

static void test_mount_acceptance(void)
{
    check(arm_gate_mount_accepted(IMU_MOUNT_UP, IMU_MOUNT_DOWN, false), "up->down idle");
    check(arm_gate_mount_accepted(IMU_MOUNT_DOWN, IMU_MOUNT_UP, false), "down->up idle");
    check(!arm_gate_mount_accepted(IMU_MOUNT_UP, IMU_MOUNT_DOWN, true), "refused while running");
    check(!arm_gate_mount_accepted(IMU_MOUNT_UP, IMU_MOUNT_UP, false), "unchanged: no reset");
    check(!arm_gate_mount_accepted(IMU_MOUNT_UP, 7u, false), "unknown mount refused");
}

int main(void)
{
    test_same_tick_mount_change_refuses();
    test_same_tick_mount_fix_allows();
    test_unchanged_mount();
    test_failed_read_refuses_nothing();
    test_moving_sample_not_refused();
    test_mount_acceptance();
    printf("test_arm_gate: all tests passed\n");
    return 0;
}
