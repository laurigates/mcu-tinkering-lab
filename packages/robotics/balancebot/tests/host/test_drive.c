// Host unit tests for drive.c (no Pico SDK): the speed/turn ramp, the
// timeout, the LQR reference lead clamp and the wheel mix.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "drive.h"

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

static int near(float a, float b)
{
    return fabsf(a - b) < 1e-3f;
}

static const float DT = CONTROL_DT_S;

static void step_n(drive_t *d, int n)
{
    for (int i = 0; i < n; i++) {
        drive_step(d, DT, DRIVE_SPEED_ACCEL_SPS2, DRIVE_TURN_ACCEL_SPS2);
    }
}

// The reference ramps at the configured rate, never jumps, and settles on the
// target exactly.
static void test_speed_ramps_to_target(void)
{
    drive_t d;
    drive_reset(&d);
    drive_command(&d, 1000.0f, 0.0f, 10.0f, 2000.0f, 1000.0f);
    drive_step(&d, DT, DRIVE_SPEED_ACCEL_SPS2, DRIVE_TURN_ACCEL_SPS2);
    check(near(d.speed_ref_sps, DRIVE_SPEED_ACCEL_SPS2 * DT), "first tick moves one ramp step");
    step_n(&d, CONTROL_LOOP_HZ / 2 - 1);  // 0.5 s total
    check(near(d.speed_ref_sps, 500.0f), "half a second at 1000 sps^2 is 500 sps");
    step_n(&d, CONTROL_LOOP_HZ);  // well past 1 s
    check(d.speed_ref_sps == 1000.0f, "settles on the target, no overshoot");
}

// Commands beyond the limits are clamped, and a non-finite one is a stop.
static void test_command_clamps(void)
{
    drive_t d;
    drive_reset(&d);
    drive_command(&d, 9000.0f, -9000.0f, 999.0f, 2000.0f, 1000.0f);
    check(d.speed_target_sps == 2000.0f, "speed clamped to speed_max");
    check(d.turn_target_sps == -1000.0f, "turn clamped to -turn_max");
    check(d.remaining_s == DRIVE_MAX_S, "duration clamped to DRIVE_MAX_S");
    drive_command(&d, NAN, INFINITY, NAN, 2000.0f, 1000.0f);
    check(d.speed_target_sps == 0.0f && d.turn_target_sps == 0.0f, "non-finite targets stop");
    check(d.remaining_s == 0.0f, "non-finite duration is 0");
    drive_command(&d, 100.0f, 0.0f, -1.0f, 2000.0f, 1000.0f);
    check(d.remaining_s == 0.0f, "negative duration is 0");
}

// When the timer runs out the targets drop to 0 and the references ramp down;
// they do not snap, so the balance law is not handed a step.
static void test_timeout_ramps_down(void)
{
    drive_t d;
    drive_reset(&d);
    drive_command(&d, 500.0f, 400.0f, 1.0f, 2000.0f, 1000.0f);
    step_n(&d, CONTROL_LOOP_HZ);  // 1 s: both refs reached their targets
    check(d.speed_ref_sps == 500.0f && d.turn_ref_sps == 400.0f, "targets reached before timeout");
    step_n(&d, 1);
    check(d.speed_target_sps == 0.0f && d.turn_target_sps == 0.0f, "timer expiry zeroes targets");
    check(d.speed_ref_sps > 0.0f && d.speed_ref_sps < 500.0f, "speed ramps down, not snaps");
    step_n(&d, CONTROL_LOOP_HZ);
    check(d.speed_ref_sps == 0.0f && d.turn_ref_sps == 0.0f, "both back at 0");
}

// A zero-duration command stops; reset stops at once.
static void test_stop_and_reset(void)
{
    drive_t d;
    drive_reset(&d);
    drive_command(&d, 500.0f, 0.0f, 5.0f, 2000.0f, 1000.0f);
    step_n(&d, 100);
    drive_command(&d, 500.0f, 0.0f, 0.0f, 2000.0f, 1000.0f);
    step_n(&d, 1);
    check(d.speed_target_sps == 0.0f, "zero duration stops");
    drive_reset(&d);
    check(d.speed_ref_sps == 0.0f && d.turn_ref_sps == 0.0f && d.remaining_s == 0.0f,
          "reset zeroes everything");
}

// The LQR reference advances with the speed but never leads the wheels by
// more than max_lead, in either direction.
static void test_reference_lead_clamp(void)
{
    check(near(drive_advance_ref(0.0f, 0.0f, 500.0f, DT, 100.0f), 1.0f), "advances speed * dt");
    check(drive_advance_ref(99.5f, 0.0f, 500.0f, DT, 100.0f) == 100.0f, "held at +max_lead");
    check(drive_advance_ref(-99.5f, 0.0f, -500.0f, DT, 100.0f) == -100.0f, "held at -max_lead");
    // A robot pushed back past a stopped reference: the reference follows it
    // to within max_lead rather than pulling from further away.
    check(drive_advance_ref(0.0f, -500.0f, 0.0f, DT, 100.0f) == -400.0f, "follows a pushed robot");
}

// The mix never changes the mean, and cuts the turn first when a wheel would
// pass the rate clamp.
static void test_mix_keeps_mean(void)
{
    drive_wheels_t w = drive_mix(1000.0f, 300.0f, 4000.0f);
    check(w.left_sps == 1300.0f && w.right_sps == 700.0f, "turn adds left, subtracts right");
    check(near((w.left_sps + w.right_sps) / 2.0f, 1000.0f), "mean unchanged");

    w = drive_mix(3800.0f, 1000.0f, 4000.0f);
    check(w.left_sps == 4000.0f && w.right_sps == 3600.0f, "turn cut to the headroom");

    w = drive_mix(-4000.0f, 1000.0f, 4000.0f);
    check(w.left_sps == -4000.0f && w.right_sps == -4000.0f, "no headroom, no turn");

    w = drive_mix(0.0f, -500.0f, 4000.0f);
    check(w.left_sps == -500.0f && w.right_sps == 500.0f, "spin in place");
}

int main(void)
{
    test_speed_ramps_to_target();
    test_command_clamps();
    test_timeout_ramps_down();
    test_stop_and_reset();
    test_reference_lead_clamp();
    test_mix_keeps_mean();
    printf("test_drive: all passed\n");
    return 0;
}
