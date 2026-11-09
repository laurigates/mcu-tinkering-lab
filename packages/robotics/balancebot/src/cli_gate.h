// Core 0's checks before `arm` and `load` send anything, decided on the
// snapshot cli.c reads. Pure C (no SDK dependencies) so the host tests run the
// same decision code the CLI runs (tests/host/test_cli_gate.c).
#ifndef BALANCEBOT_CLI_GATE_H
#define BALANCEBOT_CLI_GATE_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_filter.h"

typedef enum {
    CLI_GATE_SEND = 0,     // send the command
    CLI_GATE_BUSY,         // core 1 has not applied earlier commands: the snapshot is stale
    CLI_GATE_NOT_IDLE,     // arm: core 1 arms only from IDLE and would ignore it
    CLI_GATE_ARMED,        // load: core 1 ignores the mount in RUN, so a load would be partial
    CLI_GATE_UPSIDE_DOWN,  // arm: a still IMU reads upside down under the mount core 1 applies
} cli_gate_t;

// `arm`. `synced` is false when core 1 had not applied the commands sent
// before it within the CLI's wait: the snapshot may predate a `mount`, so
// nothing in it is judged. `state` is balance_state_t; `gravity` is the
// snapshot's check under the mount core 1 applies.
cli_gate_t cli_gate_arm(bool synced, uint8_t state, const imu_gravity_check_t *gravity);

// `load`. Refused in RUN: core 1 applies the parameters, gains and controller
// live but ignores CMD_SET_IMU_MOUNT, which would leave the CLI's mirror
// holding a mount core 1 is not applying.
cli_gate_t cli_gate_load(bool synced, uint8_t state);

#endif  // BALANCEBOT_CLI_GATE_H
