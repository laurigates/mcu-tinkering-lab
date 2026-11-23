// Core 1's gravity gate on CMD_ARM, and the CMD_SET_IMU_MOUNT acceptance it
// depends on. balance_control.c's command drain and its per-tick read call
// these, so the host tests run the same decision code core 1 runs. Pure C (no
// SDK dependencies).
#ifndef BALANCEBOT_ARM_GATE_H
#define BALANCEBOT_ARM_GATE_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_filter.h"

typedef struct {
    bool sample_ok;             // the last IMU read succeeded
    float accel_g[3];           // last good raw sample, sensor axes (g)
    float gyro_dps[3];          // ... and its gyro (deg/s)
    imu_gravity_check_t check;  // that sample under its tick's mount (published)
    uint32_t refusals;          // CMD_ARMs refused: still and upside down
    float refused_up_g;         // body Z of the last refusal, for core 0's report
} arm_gate_t;

// After each IMU read: keep the raw sample and classify it under the mount in
// effect for this tick (`check`, which core 0 reads from the snapshot). A
// failed read clears `check` and means no refusal.
void arm_gate_observe(arm_gate_t *g, uint32_t mount, bool ok, const float accel_g[3],
                      const float gyro_dps[3]);

// CMD_ARM from IDLE: classify the last good sample under `mount` as it stands
// now, not `check`. A CMD_SET_IMU_MOUNT drained earlier in the same tick
// counts; `check` was computed on the previous tick under the previous mount.
// Returns true when arming may go ahead; a refusal is counted in `refusals`.
bool arm_gate_allows(arm_gate_t *g, uint32_t mount);

// CMD_SET_IMU_MOUNT: accept a valid mount that differs from `current`, and
// never while running (the pitch would swing 180 degrees mid-balance).
bool arm_gate_mount_accepted(uint32_t current, uint32_t requested, bool running);

// Auto-arm hold-off. The hold-upright gesture is ignored while
// arm_gesture_held_off() is true; an explicit CMD_ARM is not affected. Ticks
// wrap, so both compare by signed difference.
#define ARM_HOLDOFF_MAX_S 60.0f

// True while `now_tick` is before `until_tick`.
bool arm_gesture_held_off(uint32_t now_tick, uint32_t until_tick);

// The hold-off end after a request to hold off `seconds` from `now_tick`:
// `seconds` is clamped to ARM_HOLDOFF_MAX_S, and a hold-off already running
// past it is kept, never shortened. A non-positive or NaN `seconds` changes
// nothing.
uint32_t arm_holdoff_extend(uint32_t now_tick, uint32_t until_tick, float seconds,
                            uint32_t ticks_per_s);

#endif  // BALANCEBOT_ARM_GATE_H
