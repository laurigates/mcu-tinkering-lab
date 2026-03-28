#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

typedef enum {
    LED_OFF,
    LED_SLOW_BLINK,   // Idle, scanning for tags
    LED_RAPID_BLINK,  // Fetching TTS from API
    LED_SOLID_ON,     // Playing audio
    LED_PULSE,        // Celebration
} led_mode_t;

/**
 * Initialize the status LED GPIO.
 */
esp_err_t status_led_init(void);

/**
 * Set the LED mode. The LED task handles the pattern.
 */
void status_led_set_mode(led_mode_t mode);

/**
 * Start the LED pattern task.
 */
esp_err_t status_led_start_task(void);

#endif  // STATUS_LED_H
