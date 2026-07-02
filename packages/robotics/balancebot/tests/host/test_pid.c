// Host unit tests for pid.c (no Pico SDK).
#include <assert.h>
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

int main(void)
{
    test_proportional();
    test_output_clamp();
    test_integral_antiwindup();
    test_derivative_on_measurement();
    test_numeric_derivative_fallback();
    test_reset();
    printf("test_pid: all tests passed\n");
    return 0;
}
