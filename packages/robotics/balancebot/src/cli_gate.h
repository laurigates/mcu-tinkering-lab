// Core 0's checks before `arm`, `mount` and `load` send anything, decided on
// the snapshot cli.c reads, and what the CLI reports once core 1 has drained
// them. Pure C (no SDK dependencies) so the host tests run the same decision
// code the CLI runs (tests/host/test_cli_gate.c).
#ifndef BALANCEBOT_CLI_GATE_H
#define BALANCEBOT_CLI_GATE_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_filter.h"

typedef enum {
    CLI_GATE_SEND = 0,      // send the command
    CLI_GATE_BUSY,          // core 1 has not applied earlier commands: the snapshot is stale
    CLI_GATE_NOT_IDLE,      // arm: core 1 arms only from IDLE and would ignore it
    CLI_GATE_ARMED,         // mount, load: core 1 ignores the mount in RUN
    CLI_GATE_UPSIDE_DOWN,   // arm: a still IMU reads upside down under the mount core 1 applies
    CLI_GATE_FAULT_NO_IMU,  // arm: FAULT with the IMU not responding; only `disarm` clears it
} cli_gate_t;

// `arm`. `synced` is false when core 1 had not applied the commands sent
// before it within the CLI's wait: the snapshot may predate a `mount`, so
// nothing in it is judged. `state` is balance_state_t; `imu_ok` is the
// snapshot's last IMU read (core 1 clears FAULT by a hold only while it
// reads); `gravity` is the snapshot's check under the mount core 1 applies.
cli_gate_t cli_gate_arm(bool synced, uint8_t state, bool imu_ok,
                        const imu_gravity_check_t *gravity);

// `mount up|down`. Refused in RUN: core 1 ignores CMD_SET_IMU_MOUNT there.
cli_gate_t cli_gate_mount(bool synced, uint8_t state);

// `load`. Refused in RUN: core 1 applies the parameters, gains and controller
// live but ignores CMD_SET_IMU_MOUNT, which would leave the CLI's mirror
// holding a mount core 1 is not applying.
cli_gate_t cli_gate_load(bool synced, uint8_t state);

typedef enum {
    CLI_ARM_OK = 0,          // core 1 is in RUN
    CLI_ARM_NO_ANSWER,       // core 1 did not drain the arm within the CLI's wait
    CLI_ARM_REFUSED,         // core 1's gravity gate refused (arm_refusals moved)
    CLI_ARM_FAULTED_TILT,    // armed, then FAULT on the tilt cutoff in the same tick
    CLI_ARM_FAULTED_NO_IMU,  // armed, then FAULT because the IMU read failed
    CLI_ARM_NOT_ARMED,       // still IDLE with no refusal counted
} cli_arm_outcome_t;

// What `arm` reports from the snapshot taken after core 1 drained CMD_ARM.
// `refusals_before` is arm_refusals from the snapshot the gate judged.
cli_arm_outcome_t cli_arm_outcome(bool synced, uint32_t refusals_before, uint32_t refusals_now,
                                  uint8_t state, bool imu_ok);

typedef enum {
    CLI_MOUNT_APPLIED = 0,  // core 1 applies the mount that was sent
    CLI_MOUNT_DROPPED,      // core 1 applies another one: it armed before the command drained
    CLI_MOUNT_NO_ANSWER,    // core 1 did not drain the command within the CLI's wait
} cli_mount_outcome_t;

// After `mount` or `load` sent CMD_SET_IMU_MOUNT: `applied` is the snapshot's
// imu_mount once core 1 drained it. On APPLIED or DROPPED the CLI's mirror
// takes `applied`, so it never holds a mount core 1 is not applying.
cli_mount_outcome_t cli_mount_outcome(bool synced, uint32_t applied, uint32_t requested);

#endif  // BALANCEBOT_CLI_GATE_H
