// Line-based tuning CLI over USB CDC stdio, driven from core 0.
// Commands: help stat get set stream cal arm disarm rate save load
#ifndef BALANCEBOT_CLI_H
#define BALANCEBOT_CLI_H

#include "ipc.h"

// `params` is core 0's mirror of the tunable parameters; the CLI edits it
// and forwards every change to the control loop.
void cli_init(balance_params_t *params);

// Call frequently from the core-0 loop; consumes any pending input bytes.
void cli_poll(void);

#endif  // BALANCEBOT_CLI_H
