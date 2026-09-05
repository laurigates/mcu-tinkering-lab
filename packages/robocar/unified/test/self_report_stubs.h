/**
 * @file self_report_stubs.h
 * @brief Injectable subsystem state for the self_report host test.
 *
 * self_report.c reads its snapshot exclusively through live accessors
 * (i2c_bus_is_ready, motor_is_initialized, ...). Backing every one of those
 * with a field here is what lets the test stage a board that a bench cannot:
 * a live I2C bus with exactly one peripheral refusing to come up.
 */

#ifndef ROBOCAR_UNIFIED_TEST_SELF_REPORT_STUBS_H
#define ROBOCAR_UNIFIED_TEST_SELF_REPORT_STUBS_H

#include <stdbool.h>

typedef struct {
    bool i2c_bus_ready;
    bool motors_initialized;
    bool leds_initialized;
    bool servos_initialized;
    bool buzzer_initialized;
    bool expander_available;
    bool audio_ready;
    bool wifi_connected;
    const char *ssid;
    const char *api_key;
    const char *version;
} self_report_stub_state_t;

extern self_report_stub_state_t g_stub;

/** Every subsystem up, no expander fitted, a short SSID and version. */
void stub_reset_healthy(void);

#endif /* ROBOCAR_UNIFIED_TEST_SELF_REPORT_STUBS_H */
