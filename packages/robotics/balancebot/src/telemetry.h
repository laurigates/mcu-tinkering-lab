// CSV telemetry streaming over USB CDC stdio, driven from core 0.
#ifndef BALANCEBOT_TELEMETRY_H
#define BALANCEBOT_TELEMETRY_H

#include <stdbool.h>

void telemetry_init(void);
void telemetry_set_enabled(bool enabled, int hz);  // hz <= 0 keeps the current rate
bool telemetry_is_enabled(void);

// Call frequently from the core-0 loop; prints one CSV line when due.
void telemetry_poll(void);

#endif  // BALANCEBOT_TELEMETRY_H
