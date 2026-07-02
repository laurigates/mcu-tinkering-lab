// Core0 <-> core1 exchange types. Commands and parameter updates travel
// core0 -> core1 through a pico_util queue; state travels core1 -> core0
// through a seqlock'd snapshot (see balance_control.c). Neither direction
// blocks the control loop.
#ifndef BALANCEBOT_IPC_H
#define BALANCEBOT_IPC_H

#include <stdbool.h>
#include <stdint.h>

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
} command_id_t;

typedef enum {
    PARAM_ANGLE_KP = 0,
    PARAM_ANGLE_KI,
    PARAM_ANGLE_KD,
    PARAM_VEL_KP,
    PARAM_VEL_KI,
    PARAM_FILTER_ALPHA,
    PARAM_MAX_RATE,
    PARAM_COUNT,
} param_id_t;

typedef struct {
    uint8_t cmd;       // command_id_t
    uint8_t param_id;  // param_id_t or motor index
    float value;
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
} balance_snapshot_t;

#endif  // BALANCEBOT_IPC_H
