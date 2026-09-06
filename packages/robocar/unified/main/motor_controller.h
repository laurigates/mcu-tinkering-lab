/**
 * @file motor_controller.h
 * @brief Hardware abstraction layer for TB6612FNG motor control via PCA9685
 *
 * Same public API as robocar-main, but internal implementation uses PCA9685
 * I2C PWM driver instead of GPIO + LEDC. Speed is still 0-255 externally
 * but mapped to 12-bit (0-4095) internally.
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t motor_controller_init(void);
esp_err_t motor_move_forward(uint8_t speed);
esp_err_t motor_move_backward(uint8_t speed);
esp_err_t motor_turn_left(uint8_t speed);
esp_err_t motor_turn_right(uint8_t speed);
esp_err_t motor_rotate_cw(uint8_t speed);
esp_err_t motor_rotate_ccw(uint8_t speed);
esp_err_t motor_stop(void);
esp_err_t motor_set_individual(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction,
                               uint8_t right_direction);
esp_err_t motor_get_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                          uint8_t *right_direction);

/**
 * @brief Whether motor_controller_init() succeeded.
 *
 * Every motor entry point already refuses with ESP_ERR_INVALID_STATE while
 * uninitialised, so this is not a precondition callers have to check — it is
 * how self_report names motors as the missing peripheral on a board that came
 * up without them (issue #500). Mirrors led_is_initialized() /
 * servo_is_initialized().
 */
bool motor_is_initialized(void);

/**
 * @brief How long a cached PCA9685 motor state is trusted before it is written
 *        again even though nothing changed.
 *
 * An unchanged state is not re-written: reactive_controller's 30 Hz loop calls
 * motor_stop() on every iteration it is not driving, so a parked robot used to
 * re-state six identical registers 30 times a second — the firmware's only
 * continuous I2C traffic.
 *
 * The suppression expires because the cache describes a chip that cannot be
 * read back. A PCA9685 that browned out, was re-seated, or dropped a
 * transaction reported as successful no longer matches the cache, and with no
 * expiry nothing would re-assert the true state. Exposed here rather than kept
 * private so test_motor_controller.c pins the shipped value instead of a
 * retyped copy of it.
 */
#define MOTOR_REFRESH_INTERVAL_MS 1000U

#endif  // MOTOR_CONTROLLER_H
