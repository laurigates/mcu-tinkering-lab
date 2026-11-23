// What the XIAO's onboard RGB LED shows, decided from the core-1 snapshot.
// Pure C (no SDK dependencies) so the host tests run the same decision
// (tests/host/test_status_color.c); status_led.c only sends the colour.
#ifndef BALANCEBOT_STATUS_COLOR_H
#define BALANCEBOT_STATUS_COLOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

// Brightness ceiling: the LED is a 5 V WS2812-type part next to the user's
// eyes while they hold the robot; full scale is glaring.
#define STATUS_RGB_MAX 40u

// Within this many degrees of the trimmed zero the LED turns yellow, so the
// user can find the narrower auto-arm window (ARM_GESTURE_DEG).
#define STATUS_NEAR_DEG 5.0f

typedef struct {
    uint8_t state;  // balance_state_t
    bool starting;  // core 1 has not run a loop tick yet (tick 0)
    bool imu_ok;
    float pitch_deg;             // trimmed pitch, as the gesture sees it
    uint32_t arm_holdoff_ticks;  // auto-arm hold-off left
    uint32_t gesture_ticks;      // consecutive ticks inside the arm window
    uint32_t gesture_needed;     // ticks the gesture needs (ARM_GESTURE_S * loop Hz)
    float arm_window_deg;        // ARM_GESTURE_DEG
    uint32_t ms;                 // a millisecond clock, for blinking
} status_inputs_t;

// Colours, in priority order:
//   starting                      cyan, slow pulse
//   FAULT                         red, 5 Hz blink
//   IMU not responding            red, solid
//   RUN                           green, solid
//   IDLE, hold-off running        purple
//   IDLE, inside the arm window   green, brightening as the 3 s count fills
//   IDLE, within STATUS_NEAR_DEG  yellow
//   IDLE, further                 dim blue
rgb_t status_color(const status_inputs_t *in);

#endif  // BALANCEBOT_STATUS_COLOR_H
