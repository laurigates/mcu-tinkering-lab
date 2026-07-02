#include "pid.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->initialized = 0;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float rate, float dt)
{
    float error = setpoint - measurement;

    pid->integral = clampf(pid->integral + error * dt, -pid->integral_max, pid->integral_max);

    float d_meas;
    if (!isnan(rate)) {
        d_meas = rate;
    } else if (pid->initialized && dt > 0.0f) {
        d_meas = (measurement - pid->prev_measurement) / dt;
    } else {
        d_meas = 0.0f;
    }
    pid->prev_measurement = measurement;
    pid->initialized = 1;

    // Derivative on measurement: a rising measurement opposes the output.
    float out = pid->kp * error + pid->ki * pid->integral - pid->kd * d_meas;
    return clampf(out, -pid->out_max, pid->out_max);
}
