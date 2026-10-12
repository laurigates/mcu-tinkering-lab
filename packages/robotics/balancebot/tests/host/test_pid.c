// Host unit tests for pid.c (no Pico SDK).
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

static void test_proportional(void)
{
    pid_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 100.0f, 1000.0f);
    float out = pid_update(&pid, 10.0f, 0.0f, 0.0f, 0.002f);
    assert(fabsf(out - 20.0f) < 1e-4f);
}

static void test_output_clamp(void)
{
    pid_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 100.0f, 50.0f);
    assert(pid_update(&pid, 10.0f, 0.0f, 0.0f, 0.002f) == 50.0f);
    assert(pid_update(&pid, -10.0f, 0.0f, 0.0f, 0.002f) == -50.0f);
}

static void test_integral_antiwindup(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 5.0f, 1000.0f);
    // Constant error of 100 for 1000 steps of 10 ms => unclamped integral 1000.
    float out = 0.0f;
    for (int i = 0; i < 1000; i++) {
        out = pid_update(&pid, 100.0f, 0.0f, 0.0f, 0.01f);
    }
    assert(fabsf(out - 5.0f) < 1e-4f);  // clamped at integral_max
}

static void test_derivative_on_measurement(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 100.0f, 1000.0f);
    // External rate: derivative acts on measurement, opposing its rise.
    float out = pid_update(&pid, 0.0f, 0.0f, 50.0f, 0.002f);
    assert(fabsf(out + 50.0f) < 1e-4f);
    // Setpoint steps do NOT kick the derivative (rate given as 0).
    pid_reset(&pid);
    pid_update(&pid, 0.0f, 0.0f, 0.0f, 0.002f);
    out = pid_update(&pid, 1000.0f, 0.0f, 0.0f, 0.002f);
    assert(fabsf(out) < 1e-4f);
}

static void test_numeric_derivative_fallback(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 100.0f, 1000.0f);
    pid_update(&pid, 0.0f, 0.0f, NAN, 0.01f);              // first call: no kick
    float out = pid_update(&pid, 0.0f, 1.0f, NAN, 0.01f);  // d = 100/s
    assert(fabsf(out + 100.0f) < 1e-3f);
}

static void test_reset(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 100.0f, 1000.0f);
    for (int i = 0; i < 100; i++) {
        pid_update(&pid, 10.0f, 0.0f, 0.0f, 0.01f);
    }
    pid_reset(&pid);
    assert(pid.integral == 0.0f);
    assert(pid.initialized == 0);
}

static void test_set_integral_clamps(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 5.0f, 1000.0f);
    pid_set_integral(&pid, 3.0f);
    assert(pid.integral == 3.0f);
    pid_set_integral(&pid, 50.0f);
    assert(pid.integral == 5.0f);
    pid_set_integral(&pid, -50.0f);
    assert(pid.integral == -5.0f);
}

// LQR -> PID switch: the first PID output must equal the last LQR output.
// Numbers are in the firmware's units (deg, deg/s, steps/s^2) and in the
// range a live switch sees; the gains are the angle PID's with a nonzero ki.
static void test_bumpless_first_output_matches_target(void)
{
    const float setpoint = 0.4f;  // velocity-loop lean offset, deg
    const float pitch = 2.5f;     // deg
    const float gyro = -18.0f;    // deg/s
    const float dt = 0.002f;
    const float targets[] = {-7321.5f, 0.0f, 1234.25f, 39000.0f};
    for (unsigned i = 0; i < sizeof targets / sizeof targets[0]; i++) {
        pid_t pid;
        pid_init(&pid, 28.0f, 5.0f, 1.2f, 10000.0f, 40000.0f);
        pid.integral = 123.0f;  // stale state from before the LQR stint
        float i0 = pid_bumpless_integral(&pid, setpoint, pitch, gyro, dt, targets[i]);
        pid_set_integral(&pid, i0);
        assert(pid.integral == i0);  // in range: not clamped
        float out = pid_update(&pid, setpoint, pitch, gyro, dt);
        float tol = 8.0f * FLT_EPSILON * (fabsf(targets[i]) + 100.0f);
        if (fabsf(out - targets[i]) > tol) {
            printf("bumpless: target %g got %g (tol %g)\n", (double)targets[i], (double)out,
                   (double)tol);
            assert(0);
        }
    }
}

// When the required integral exceeds the anti-windup clamp the integral
// saturates, and the output falls short of the target on the near side.
static void test_bumpless_clamped(void)
{
    const float setpoint = 0.0f, pitch = 1.0f, gyro = 0.0f, dt = 0.002f;
    for (int sign = -1; sign <= 1; sign += 2) {
        pid_t pid;
        pid_init(&pid, 28.0f, 0.01f, 1.2f, 100.0f, 40000.0f);
        const float target = (float)sign * 30000.0f;
        float i0 = pid_bumpless_integral(&pid, setpoint, pitch, gyro, dt, target);
        assert(fabsf(i0) > pid.integral_max);
        pid_set_integral(&pid, i0);
        assert(pid.integral == (float)sign * pid.integral_max);
        float out = pid_update(&pid, setpoint, pitch, gyro, dt);
        float error = setpoint - pitch;
        float expected = pid.kp * error + pid.ki * pid.integral - pid.kd * gyro;
        assert(fabsf(out - expected) < 1e-4f);
        assert(sign > 0 ? out < target : out > target);
    }
}

static void test_bumpless_without_ki_is_zero(void)
{
    pid_t pid;
    pid_init(&pid, 28.0f, 0.0f, 1.2f, 10000.0f, 40000.0f);
    assert(pid_bumpless_integral(&pid, 0.0f, 2.0f, 5.0f, 0.002f, 500.0f) == 0.0f);
}

int main(void)
{
    test_proportional();
    test_output_clamp();
    test_integral_antiwindup();
    test_derivative_on_measurement();
    test_numeric_derivative_fallback();
    test_reset();
    test_set_integral_clamps();
    test_bumpless_first_output_matches_target();
    test_bumpless_clamped();
    test_bumpless_without_ki_is_zero();
    printf("test_pid: all tests passed\n");
    return 0;
}
