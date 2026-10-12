// Generic PID controller. Pure C (no SDK dependencies) so it can be
// host-unit-tested; see tests/host/test_pid.c.
#ifndef BALANCEBOT_PID_H
#define BALANCEBOT_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;  // symmetric anti-windup clamp on the integral term
    float out_max;       // symmetric output clamp
    // state
    float integral;
    float prev_measurement;
    int initialized;  // first update skips the derivative kick
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_max, float out_max);

// Reset integral and derivative state (e.g. on arm) without touching gains.
void pid_reset(pid_t *pid);

// One controller step. Derivative acts on the measurement (not the error)
// so setpoint changes don't kick the output. `rate` is an optional externally
// measured derivative of the measurement (e.g. gyro rate); pass NAN to use
// the numeric difference instead.
float pid_update(pid_t *pid, float setpoint, float measurement, float rate, float dt);

// Set the integral state, clamped to +/- integral_max.
void pid_set_integral(pid_t *pid, float integral);

// Bumpless transfer (back-calculation): the integral to load with
// pid_set_integral() so that the NEXT pid_update() with these exact
// arguments returns `target_out`. Pure: reads gains and nothing else.
// With ki == 0 the integral cannot move the output and 0 is returned.
// Pass a measured `rate` (not NAN): the numeric-derivative path depends on
// history this function does not model. If the result is clamped by
// pid_set_integral(), the output saturates short of `target_out`.
float pid_bumpless_integral(const pid_t *pid, float setpoint, float measurement, float rate,
                            float dt, float target_out);

#endif  // BALANCEBOT_PID_H
