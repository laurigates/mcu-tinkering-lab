// The live PID/LQR switch (ADR-026): what core 1 does on CMD_SET_CONTROLLER
// and on the first PID tick after an LQR stint. Pure C (no SDK includes) so
// the host tests can drive a switch sequence; see tests/host/test_ctrl_switch.c.
#ifndef BALANCEBOT_CTRL_SWITCH_H
#define BALANCEBOT_CTRL_SWITCH_H

#include <stdbool.h>

#include "ipc.h"
#include "pid.h"

typedef struct {
    ctrl_mode_t mode;           // law in use; starts PID, LQR only via a request
    float x_ref_steps;          // position the LQR holds (set on PID->LQR)
    bool pid_bumpless_pending;  // LQR->PID: back-calculate on the next RUN tick
} ctrl_switch_t;

// Fail closed: start on the PID.
void ctrl_switch_init(ctrl_switch_t *sw);

// Entering RUN: hold position 0 and drop a pending back-calculation.
void ctrl_switch_on_run(ctrl_switch_t *sw);

// CMD_SET_CONTROLLER. PID->LQR, bumpless: needs valid gains and a positive
// steps_per_m, else it is refused (returns false, nothing changes); on a
// switch the LQR holds the current position and *angle_setpoint (the PID's
// lean offset) is zeroed. LQR->PID: while running, the angle integral is
// back-calculated on the next RUN tick (ctrl_switch_resume_pid()), since it
// needs that tick's pitch. A request for the mode in use changes nothing.
bool ctrl_switch_request(ctrl_switch_t *sw, ctrl_mode_t want, bool lqr_k_valid, float steps_per_m,
                         bool running, float wheel_pos_steps, float *angle_setpoint);

// The first PID tick after LQR->PID, in place of the velocity loop's update:
// the velocity PI restarts from a zero integral, and the angle integral is
// loaded so that pid_update(angle_pid, <returned setpoint>, pitch, gyro_dps,
// dt) returns last_accel_sps2, the last LQR output. Returns the angle
// setpoint and clears the pending flag.
float ctrl_switch_resume_pid(ctrl_switch_t *sw, pid_t *angle_pid, pid_t *vel_pid,
                             float speed_ref_sps, float wheel_rate_sps, float pitch_deg,
                             float gyro_dps, float dt, float vel_dt, float last_accel_sps2);

#endif  // BALANCEBOT_CTRL_SWITCH_H
