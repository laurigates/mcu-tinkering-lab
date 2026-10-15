/**
 * @file ultrasonic.h
 * @brief Front and rear RCWL-1601 rangefinders over I2C, one per mux channel.
 *
 * Both modules run in I2C mode at RCWL1601_I2C_ADDR, each alone on its own
 * TCA9548A channel (I2C_BUS_CHANNEL_RANGER_FRONT / _REAR in pin_config.h). The
 * read is the non-blocking two-phase cycle in ultrasonic_core.h: trigger and
 * release the bus, then on a later poll, once ULTRASONIC_CONVERSION_MS has
 * passed, read three bytes and release. The bus is never held across a
 * conversion, so PCA9685 writes are never blocked behind one.
 *
 * Threading
 * ---------
 * ultrasonic_poll() is NOT reentrant per sensor — the reactive controller is
 * the only caller in the robot firmware and polls from a single task. The bus
 * itself is serialised by i2c_bus_select_channel().
 *
 * Host-test build
 * ---------------
 * Compile with -DULTRASONIC_HOST_TEST=1 to replace the I2C calls with a stub
 * per sensor: ultrasonic_poll() returns, on every call, the value last set by
 * ultrasonic_test_set_distance() for that sensor. The two-phase sequencing is
 * tested on the real state machine in test_ultrasonic_core.c.
 */

#pragma once

#include <stdint.h>

#include "ultrasonic_core.h"

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

/** Which rangefinder. */
typedef enum {
    ULTRASONIC_FRONT = 0, /**< Fixed, facing forward; drives the obstacle reflex. */
    ULTRASONIC_REAR,      /**< Facing backward; telemetry only.                  */
    ULTRASONIC_COUNT,
} ultrasonic_id_t;

/**
 * @brief Prepare both rangefinders.
 *
 * Needs the I2C bus up (i2c_bus_init()). Does not probe the sensors: an absent
 * one shows up as ULTRASONIC_ERROR from ultrasonic_poll(), paced like a working
 * one, so a robot with one or no rangefinder fitted still boots and drives.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the I2C bus is not up.
 */
esp_err_t ultrasonic_init(void);

/**
 * @brief Advance one rangefinder's read cycle by at most one bus operation.
 *
 * @param id           Which sensor.
 * @param now_ms       Monotonic milliseconds; may wrap.
 * @param[out] distance_cm  Written on READY (centimetres) and ERROR
 *                          (ULTRASONIC_DIST_ERROR); untouched on PENDING.
 * @return ULTRASONIC_READY with a fresh reading, ULTRASONIC_PENDING when there
 *         is nothing new, ULTRASONIC_ERROR on a bus error, a zero or an
 *         out-of-range reading, an unknown id, or before ultrasonic_init().
 */
ultrasonic_result_t ultrasonic_poll(ultrasonic_id_t id, uint32_t now_ms, uint16_t *distance_cm);

/** @brief The TCA9548A channel a sensor sits on (pin_config.h). */
uint8_t ultrasonic_channel(ultrasonic_id_t id);

/** @brief "front" or "rear", for logs and the console. */
const char *ultrasonic_name(ultrasonic_id_t id);

/** @brief Release the I2C device descriptor. Optional; mainly for tests. */
esp_err_t ultrasonic_deinit(void);

/* =========================================================================
 * Host-test hook (compiled on all builds for linker consistency)
 * ========================================================================= */

/**
 * @brief Inject the reading a sensor returns.
 *
 * On-target: no-op. Host-test (ULTRASONIC_HOST_TEST=1): every later
 * ultrasonic_poll() for @p id returns READY with @p cm, or ERROR when @p cm is
 * ULTRASONIC_DIST_ERROR.
 */
void ultrasonic_test_set_distance(ultrasonic_id_t id, uint16_t cm);

#ifdef __cplusplus
}
#endif
