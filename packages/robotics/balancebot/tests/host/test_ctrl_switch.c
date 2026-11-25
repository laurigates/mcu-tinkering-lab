// Host unit tests for ctrl_switch.c (no Pico SDK): the live PID/LQR switch
// driven through a PID -> LQR -> PID sequence the way core 1's RUN branch
// runs it.
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

#include "ctrl_switch.h"

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

static const float DT = 0.002f;
static const float VEL_DT = 0.002f * 5.0f;
static const float STEPS_PER_M = 7835.0f;

static void init_pids(pid_t *angle, pid_t *vel)
{
    pid_init(angle, 28.0f, 5.0f, 1.2f, 10000.0f, 40000.0f);
    pid_init(vel, 0.002f, 0.0005f, 0.0f, 6000.0f, 3.0f);
}

static void test_pid_lqr_pid_sequence(void)
{
    ctrl_switch_t sw;
    pid_t angle, vel;
    ctrl_switch_init(&sw);
    ctrl_switch_on_run(&sw);
    init_pids(&angle, &vel);
    check(sw.mode == CTRL_PID, "starts on the PID");

    // A few PID ticks leave history in both PIDs.
    float setpoint = 0.0f;
    float wheel_rate = 0.0f, wheel_pos = 0.0f;
    for (int i = 0; i < 50; i++) {
        setpoint = pid_update(&vel, 0.0f, wheel_rate, NAN, VEL_DT);
        float accel = pid_update(&angle, setpoint, 1.5f, -4.0f, DT);
        wheel_rate += accel * DT;
        wheel_pos += wheel_rate * DT;
    }
    check(wheel_pos != 0.0f, "the PID ticks moved the wheels");
    check(setpoint != 0.0f, "the velocity loop leans");

    // PID -> LQR on this tick: x_ref is the current position, lean zeroed.
    check(ctrl_switch_request(&sw, CTRL_LQR, true, STEPS_PER_M, true, wheel_pos, &setpoint),
          "PID->LQR taken");
    check(sw.mode == CTRL_LQR, "now on the LQR");
    check(sw.x_ref_steps == wheel_pos, "PID->LQR: x_ref equals the current position");
    check(setpoint == 0.0f, "PID->LQR: lean offset zeroed");
    check(!sw.pid_bumpless_pending, "PID->LQR: nothing pending");

    // A repeated LQR request moves nothing, even with the robot elsewhere.
    check(
        ctrl_switch_request(&sw, CTRL_LQR, true, STEPS_PER_M, true, wheel_pos + 500.0f, &setpoint),
        "LQR->LQR accepted");
    check(sw.x_ref_steps == wheel_pos, "LQR->LQR: x_ref kept");

    // LQR -> PID while running: pending until the next RUN tick.
    const float last_lqr_accel = -6543.25f;
    const float pitch = 2.5f, gyro = -18.0f;
    angle.integral = 123.0f;  // stale from before the LQR stint
    vel.integral = 4000.0f;
    check(ctrl_switch_request(&sw, CTRL_PID, true, STEPS_PER_M, true, wheel_pos, &setpoint),
          "LQR->PID taken");
    check(sw.mode == CTRL_PID, "back on the PID");
    check(sw.pid_bumpless_pending, "LQR->PID while running: back-calculation pending");

    // The switch tick: the PID's output equals the last LQR output.
    setpoint = ctrl_switch_resume_pid(&sw, &angle, &vel, 0.0f, wheel_rate, pitch, gyro, DT, VEL_DT,
                                      last_lqr_accel);
    check(!sw.pid_bumpless_pending, "resume clears the pending flag");
    float out = pid_update(&angle, setpoint, pitch, gyro, DT);
    float tol = 8.0f * FLT_EPSILON * (fabsf(last_lqr_accel) + 100.0f);
    if (fabsf(out - last_lqr_accel) > tol) {
        printf("LQR->PID: wanted %g, PID gave %g\n", (double)last_lqr_accel, (double)out);
    }
    check(fabsf(out - last_lqr_accel) <= tol, "LQR->PID: PID output equals the last LQR output");

    // The velocity PI restarted from a zero integral: its setpoint is what
    // a fresh PI gives for this error.
    pid_t fresh;
    pid_init(&fresh, vel.kp, vel.ki, 0.0f, vel.integral_max, vel.out_max);
    check(setpoint == pid_update(&fresh, 0.0f, wheel_rate, NAN, VEL_DT),
          "LQR->PID: velocity PI restarted from zero");
}

static void test_lqr_refused(void)
{
    ctrl_switch_t sw;
    ctrl_switch_init(&sw);
    float setpoint = 1.25f;
    check(!ctrl_switch_request(&sw, CTRL_LQR, false, STEPS_PER_M, true, 100.0f, &setpoint),
          "PID->LQR refused without valid gains");
    check(!ctrl_switch_request(&sw, CTRL_LQR, true, 0.0f, true, 100.0f, &setpoint),
          "PID->LQR refused with steps_per_m 0");
    check(!ctrl_switch_request(&sw, CTRL_LQR, true, NAN, true, 100.0f, &setpoint),
          "PID->LQR refused with steps_per_m NaN");
    check(sw.mode == CTRL_PID && sw.x_ref_steps == 0.0f && setpoint == 1.25f,
          "a refusal changes nothing");
}

static void test_lqr_to_pid_disarmed(void)
{
    ctrl_switch_t sw;
    ctrl_switch_init(&sw);
    float setpoint = 0.0f;
    check(ctrl_switch_request(&sw, CTRL_LQR, true, STEPS_PER_M, false, 0.0f, &setpoint),
          "PID->LQR in IDLE taken");
    check(ctrl_switch_request(&sw, CTRL_PID, true, STEPS_PER_M, false, 0.0f, &setpoint),
          "LQR->PID in IDLE taken");
    check(!sw.pid_bumpless_pending, "LQR->PID in IDLE: nothing to back-calculate");

    // Arming drops a pending back-calculation and the old reference.
    sw.pid_bumpless_pending = true;
    sw.x_ref_steps = 42.0f;
    ctrl_switch_on_run(&sw);
    check(!sw.pid_bumpless_pending && sw.x_ref_steps == 0.0f, "entering RUN resets the switch");
    check(sw.mode == CTRL_PID, "entering RUN keeps the law");
}

int main(void)
{
    test_pid_lqr_pid_sequence();
    test_lqr_refused();
    test_lqr_to_pid_disarmed();
    printf("test_ctrl_switch: all tests passed\n");
    return 0;
}
