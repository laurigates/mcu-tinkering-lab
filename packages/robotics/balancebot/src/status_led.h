// Onboard user LED (GPIO25, active low) state patterns, driven from the
// core-0 service loop.
#ifndef BALANCEBOT_STATUS_LED_H
#define BALANCEBOT_STATUS_LED_H

#include "ipc.h"

void status_led_init(void);

// Call frequently from the core-0 loop; non-blocking.
// IDLE: slow blink (1 Hz). RUN: solid on. FAULT: fast blink (5 Hz).
void status_led_poll(balance_state_t state);

#endif  // BALANCEBOT_STATUS_LED_H
