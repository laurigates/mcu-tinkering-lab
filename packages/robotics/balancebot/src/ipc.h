// Core0 <-> core1 exchange types. Commands and parameter updates travel
// core0 -> core1 through a pico_util queue; state travels core1 -> core0
// through a seqlock'd snapshot (see balance_control.c). Neither direction
// blocks the control loop.
#ifndef BALANCEBOT_IPC_H
#define BALANCEBOT_IPC_H

#include <stdbool.h>
#include <stdint.h>

#include "lqr.h"

// Which law drives the wheels in RUN. Stored in balance_params_t (uint32_t
// so the struct stays all 4-byte fields with no padding in the flash record).
typedef enum {
    CTRL_PID = 0,
    CTRL_LQR = 1,
} ctrl_mode_t;

// Runtime-tunable parameter set. Lives in the param store (flash) and in a
// live copy on each core; this struct is also the flash record payload.
typedef struct {
    float angle_kp;  // (steps/s^2) per degree
    float angle_ki;
    float angle_kd;
    float vel_kp;  // degrees of lean per (steps/s) of velocity error
    float vel_ki;
    float filter_alpha;
    float max_rate_sps;
    // ---- v2 (PARAM_VERSION 2): LQR. The fields above are the v1 record and
    // must stay a prefix (param_record.c asserts it). K is not stored: core 0
    // re-solves it from these at boot and on every change.
    uint32_t ctrl_mode;   // ctrl_mode_t
    float l_eff_m;        // equivalent pendulum length (m)
    float wheel_diam_m;   // wheel diameter (m)
    float steps_per_rev;  // microsteps per wheel revolution
    float q_theta;        // LQR state weights (diagonal Q) ...
    float q_dtheta;
    float q_x;
    float q_dx;
    float r;  // ... and input weight
} balance_params_t;

typedef enum {
    BAL_STATE_IDLE = 0,   // disarmed, waiting for arm gesture or CLI
    BAL_STATE_RUN = 1,    // balancing
    BAL_STATE_FAULT = 2,  // tilt cutoff tripped; clears after upright hold
} balance_state_t;

typedef enum {
    CMD_SET_PARAM = 0,  // param_id + value
    CMD_ARM = 1,
    CMD_DISARM = 2,
    CMD_CALIBRATE = 3,       // gyro bias cal (IDLE only)
    CMD_SAVE = 4,            // persist params to flash (IDLE/FAULT only)
    CMD_OPEN_LOOP_RATE = 5,  // param_id = motor (0/1), value = steps/s (IDLE only)
    CMD_SET_LQR_GAINS = 6,   // lqr_K = all four gains in one item (atomic handoff)
    CMD_SET_CONTROLLER = 7,  // ctrl_mode; LQR is refused while no valid K is held
} command_id_t;

typedef enum {
    PARAM_ANGLE_KP = 0,
    PARAM_ANGLE_KI,
    PARAM_ANGLE_KD,
    PARAM_VEL_KP,
    PARAM_VEL_KI,
    PARAM_FILTER_ALPHA,
    PARAM_MAX_RATE,
    PARAM_L_EFF,
    PARAM_WHEEL_DIAM,
    PARAM_STEPS_PER_REV,
    PARAM_Q_THETA,
    PARAM_Q_DTHETA,
    PARAM_Q_X,
    PARAM_Q_DX,
    PARAM_R,
    PARAM_COUNT,
} param_id_t;

typedef struct {
    uint8_t cmd;       // command_id_t
    uint8_t param_id;  // param_id_t or motor index
    union {
        float value;         // CMD_SET_PARAM, CMD_OPEN_LOOP_RATE
        float lqr_K[LQR_N];  // CMD_SET_LQR_GAINS, u = -(K . s) in m/s^2
        uint8_t ctrl_mode;   // CMD_SET_CONTROLLER (ctrl_mode_t)
    };
} balance_cmd_t;

// Telemetry snapshot, written by core1 each tick under a sequence counter
// (odd = write in progress). Core0 re-reads until it sees a stable even seq.
typedef struct {
    uint32_t seq;
    uint32_t tick;        // control-loop tick counter
    uint8_t state;        // balance_state_t
    bool imu_ok;          // last IMU read succeeded
    bool save_ok;         // result of the last CMD_SAVE
    uint32_t save_count;  // increments on every completed CMD_SAVE
    float pitch_deg;
    float gyro_dps;        // pitch rate
    float rate_cmd_sps;    // commanded wheel rate (signed)
    float angle_setpoint;  // including velocity-loop lean offset
    float loop_us;         // measured loop body duration
    uint8_t ctrl_mode;     // ctrl_mode_t actually driving the loop
    bool lqr_k_valid;      // core 1 holds a K (so LQR may be selected)
    float lqr_K[LQR_N];    // the K core 1 is using / would use
    float pos_m;           // wheel travel since arming (commanded), m
} balance_snapshot_t;

#endif  // BALANCEBOT_IPC_H
