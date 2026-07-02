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

#endif  // BALANCEBOT_PID_H
