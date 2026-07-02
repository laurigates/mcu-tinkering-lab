// DRV8825 stepper abstraction: signed step-rate interface over PIO pulse
// generation. Owns the DIR and shared nENABLE GPIOs and the rate/accel
// clamps. All functions are called from the core-1 control loop except
// stepper_init() (called once before the loop starts).
#ifndef BALANCEBOT_STEPPER_H
#define BALANCEBOT_STEPPER_H

#include <stdbool.h>

typedef enum {
    STEPPER_LEFT = 0,
    STEPPER_RIGHT = 1,
    STEPPER_COUNT = 2,
} stepper_id_t;

// Loads the PIO program and claims one state machine per motor. Motors start
// disabled (nENABLE high) at zero rate.
void stepper_init(void);

// Enable/disable both drivers via the shared nENABLE pin. Disabling also
// zeroes the commanded rates.
void stepper_enable(bool enabled);
bool stepper_is_enabled(void);

// Command a signed step rate (steps/s; sign = direction). The request is
// slew-limited by MAX_ACCEL_SPS2 and clamped to +/- max_rate. Below
// MIN_RATE_SPS the state machine is parked (no pulses). Direction reversals
// pass through the parked state. Call at the control-loop rate.
void stepper_set_rate(stepper_id_t id, float steps_per_s, float max_rate, float dt);

// Rate actually being generated after slew/clamp (steps/s, signed).
float stepper_get_rate(stepper_id_t id);

#endif  // BALANCEBOT_STEPPER_H
