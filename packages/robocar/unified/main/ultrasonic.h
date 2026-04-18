/**
 * @file ultrasonic.h
 * @brief HC-SR04P / RCWL-1601 / US-100 ultrasonic rangefinder driver.
 *
 * Uses the ESP-IDF v5.4 RMT RX peripheral to capture the echo pulse width
 * without busy-waiting.  Falls back to a GPIO + esp_timer polling loop when
 * RMT initialisation fails (e.g., all RMT channels are already claimed).
 *
 * Pin assignment (defined in pin_config.h):
 *   ULTRASONIC_TRIG_PIN  — GPIO3  (output, 10 µs trigger pulse)
 *   ULTRASONIC_ECHO_PIN  — GPIO4  (input,  RMT RX captures pulse width)
 *
 * Threading
 * ---------
 * ultrasonic_measure() is NOT reentrant — the reactive controller is the only
 * caller and invokes it from a single task.  No internal mutex is used; callers
 * must not call from multiple tasks simultaneously.
 *
 * Host-test build
 * ---------------
 * Compile with -DULTRASONIC_HOST_TEST=1 to replace the ESP-IDF hardware calls
 * with thin stubs.  The stub makes ultrasonic_measure() return the value set by
 * ultrasonic_test_set_distance().
 */

#pragma once

#include <stdint.h>

#ifndef ULTRASONIC_HOST_TEST
#include "esp_err.h"
#else
#include <errno.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG (-2)
#define ESP_ERR_INVALID_STATE (-3)
#define ESP_ERR_NOT_FOUND (-4)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Sentinel value written to *distance_cm on timeout or error. */
#define ULTRASONIC_DIST_ERROR UINT16_MAX

/** Maximum reliable sensing range (HC-SR04P: ~4 m; rounded to 400 cm). */
#define ULTRASONIC_MAX_RANGE_CM 400U

/** Timeout waiting for the echo pulse, µs (40 ms covers 6 m round-trip). */
#define ULTRASONIC_ECHO_TIMEOUT_US 40000U

/**
 * @brief Initialise the ultrasonic driver.
 *
 * Configures TRIG as a GPIO output and attempts to claim an RMT RX channel for
 * ECHO.  If RMT is unavailable the driver falls back to GPIO polling and logs a
 * warning.  Call once at boot before ultrasonic_measure().
 *
 * @return ESP_OK on success.
 */
esp_err_t ultrasonic_init(void);

/**
 * @brief Trigger a single ranging measurement.
 *
 * Sends the 10 µs TRIG pulse and waits for the ECHO response.  Blocks for up
 * to ULTRASONIC_ECHO_TIMEOUT_US µs (≈40 ms).
 *
 * @param[out] distance_cm  Distance in centimetres.  Set to
 *                          ULTRASONIC_DIST_ERROR on timeout or hardware fault.
 * @return  ESP_OK on a valid reading, ESP_FAIL on timeout or error.
 */
esp_err_t ultrasonic_measure(uint16_t *distance_cm);

/**
 * @brief Tear down the RMT channel and GPIO config.
 *
 * Optional — mainly useful for unit tests that need a clean reset between
 * cases.
 */
esp_err_t ultrasonic_deinit(void);

/* =========================================================================
 * Host-test hook (compiled on all builds for linker consistency)
 * ========================================================================= */

/**
 * @brief Inject a distance reading for the next ultrasonic_measure() call.
 *
 * On-target: no-op (the real RMT reading always wins).
 * Host-test (ULTRASONIC_HOST_TEST=1): stores value returned by the next
 * ultrasonic_measure() call.  ULTRASONIC_DIST_ERROR simulates a timeout.
 */
void ultrasonic_test_set_distance(uint16_t cm);

#ifdef __cplusplus
}
#endif
