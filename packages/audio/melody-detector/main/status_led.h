#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

typedef enum {
    LED_STATE_IDLE,        // slow heartbeat (~1 Hz)
    LED_STATE_CAPTURING,   // fast blink (~5 Hz)
    LED_STATE_PROCESSING,  // solid on
    LED_STATE_PLAYING,     // double-pulse heartbeat
    LED_STATE_ERROR,       // SOS-ish rapid blink
} led_state_t;

esp_err_t status_led_init(void);

// Non-blocking: sets the desired state; a background task drives the LED.
void status_led_set(led_state_t state);

#endif  // STATUS_LED_H
