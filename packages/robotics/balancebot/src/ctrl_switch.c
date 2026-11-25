#include "ctrl_switch.h"

#include <math.h>

void ctrl_switch_init(ctrl_switch_t *sw)
{
    sw->mode = CTRL_PID;
    ctrl_switch_on_run(sw);
}

void ctrl_switch_on_run(ctrl_switch_t *sw)
{
    sw->x_ref_steps = 0.0f;
    sw->pid_bumpless_pending = false;
}

bool ctrl_switch_request(ctrl_switch_t *sw, ctrl_mode_t want, bool lqr_k_valid, float steps_per_m,
                         bool running, float wheel_pos_steps, float *angle_setpoint)
{
    if (want == CTRL_LQR && sw->mode != CTRL_LQR) {
        if (!lqr_k_valid || !(isfinite(steps_per_m) && steps_per_m > 0.0f)) {
            return false;  // refused; core 0 checks first and reports why
        }
        // PID -> LQR, bumpless: hold the current position (error starts at
        // 0); the wheel rate carries over unchanged.
        sw->x_ref_steps = wheel_pos_steps;
        *angle_setpoint = 0.0f;
        sw->pid_bumpless_pending = false;
        sw->mode = CTRL_LQR;
    } else if (want == CTRL_PID && sw->mode != CTRL_PID) {
        sw->pid_bumpless_pending = running;
        sw->mode = CTRL_PID;
    }
    return true;
}

float ctrl_switch_resume_pid(ctrl_switch_t *sw, pid_t *angle_pid, pid_t *vel_pid,
                             float speed_ref_sps, float wheel_rate_sps, float pitch_deg,
                             float gyro_dps, float dt, float vel_dt, float last_accel_sps2)
{
    pid_reset(vel_pid);
    float setpoint = pid_update(vel_pid, speed_ref_sps, wheel_rate_sps, NAN, vel_dt);
    pid_set_integral(angle_pid, pid_bumpless_integral(angle_pid, setpoint, pitch_deg, gyro_dps, dt,
                                                      last_accel_sps2));
    sw->pid_bumpless_pending = false;
    return setpoint;
}
