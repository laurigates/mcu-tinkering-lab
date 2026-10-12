// Line-based tuning CLI over USB CDC stdio, driven from core 0.
// Commands: help stat get set ctrl stream cal arm disarm rate save load
#ifndef BALANCEBOT_CLI_H
#define BALANCEBOT_CLI_H

#include "ipc.h"

// `params` is core 0's mirror of the tunable parameters; the CLI edits it
// and forwards every change to the control loop. Also runs the boot LQR
// solve and hands core 1 its gains, so call it after balance_control_init()
// and before multicore_launch_core1(). A saved LQR mode whose solve fails
// falls back to PID (cli_print_boot_notes() and `stat` say so).
void cli_init(balance_params_t *params);

// Print the active controller and any boot fallback note (after the banner).
void cli_print_boot_notes(void);

// Call frequently from the core-0 loop; consumes any pending input bytes.
void cli_poll(void);

#endif  // BALANCEBOT_CLI_H
