// Core-1 hard-real-time balancing loop: IMU -> complementary filter -> PID
// cascade -> stepper rates, plus the IDLE/RUN/FAULT state machine, tilt
// cutoff and watchdog. Core 0 talks to it exclusively through
// balance_control_send_cmd() and balance_control_get_snapshot().
#ifndef BALANCEBOT_BALANCE_CONTROL_H
#define BALANCEBOT_BALANCE_CONTROL_H

#include <stdbool.h>

#include "ipc.h"

// Call from core 0 before multicore launch. `initial` seeds the live
// parameter copy (normally loaded from the param store).
void balance_control_init(const balance_params_t *initial);

// Core-1 entry point; never returns. Pass to multicore_launch_core1().
void balance_control_core1_main(void);

// Queue a command to the control loop. Returns false if the queue is full.
bool balance_control_send_cmd(const balance_cmd_t *cmd);

// Copy out the latest consistent telemetry snapshot (seqlock read).
void balance_control_get_snapshot(balance_snapshot_t *out);

#endif  // BALANCEBOT_BALANCE_CONTROL_H
