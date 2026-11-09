/**
 * @file cliff_sensor.h
 * @brief Front and rear downward floor sensors on the MCP23017.
 *
 * Interim hardware until the VL53L4CD ToF pair (issue #1010): two salvaged
 * TCRT5000-style reflective sensors (IR LED plus phototransistor, 4-pin
 * through-hole), aimed at the floor, the phototransistor collectors on
 * MCP23017 pins CLIFF_FRONT_EXPANDER_PIN / CLIFF_REAR_EXPANDER_PIN with the
 * expander's internal pull-ups. Floor under the sensor reflects the LED, the
 * phototransistor conducts and pulls the pin low; no floor leaves it high. An
 * unplugged sensor therefore reads as no floor, which is the safe reading.
 *
 * Wiring and the bench checks are in docs/firmware/cliff-sensors.md.
 */

#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdbool.h>

#include "cliff_core.h"

#ifndef CLIFF_SENSOR_HOST_TEST
#include "esp_err.h"
#else
#include "esp_err.h" /* test/include shim */
#endif

/** The pin level that means "floor seen": the phototransistor pulls low. */
#define CLIFF_FLOOR_LEVEL 0

/**
 * @brief Configure both expander pins as inputs with pull-ups.
 *
 * Requires gpio_expander_init() first. Without an MCP23017 the module stays
 * unavailable and the executor runs without the cliff reflex (non-fatal, like
 * every peripheral; the self-report names it).
 *
 * @return ESP_OK, ESP_ERR_NOT_FOUND with no expander, or the bus error.
 */
esp_err_t cliff_sensor_init(void);

/** Whether cliff_sensor_init() configured both pins. */
bool cliff_sensor_available(void);

/**
 * @brief Read both sensors in one I2C transaction.
 *
 * @param[out] out  floor[] per sensor; valid false when the read failed or the
 *                  module is unavailable.
 */
void cliff_sensor_read(cliff_sample_t *out);

#ifdef CLIFF_SENSOR_HOST_TEST
/** Host stub: availability and what every later read returns. */
void cliff_sensor_test_set(bool available, bool valid, bool front_floor, bool rear_floor);
#endif

#endif /* CLIFF_SENSOR_H */
