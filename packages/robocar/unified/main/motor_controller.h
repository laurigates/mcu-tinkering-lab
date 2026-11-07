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

/**
 * @brief Short-brake both motors — IN1 = IN2 = high, PWM = 0.
 *
 * Distinct from motor_stop(), which is the TB6612FNG's *Stop (coast)* state
 * (IN1 = IN2 = low). Per the datasheet truth table reproduced in
 * `docs/wiring-card-motors.typ`, only IN1 = IN2 = high shorts the windings and
 * actually resists rotation; low/low leaves the outputs high-impedance and the
 * robot rolls on inertia.
 *
 * Reserved for the obstacle reflex. It is deliberately NOT what the idle path
 * uses: reactive_controller's 30 Hz loop stops on every iteration it is not
 * driving, and a parked robot should not sit with its windings shorted for as
 * long as it is powered.
 */
esp_err_t motor_brake(void);
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

/**
 * @brief Time for a motor's PWM to rise from 0 to full scale.
 *
 * A stalled DC motor draws its stall current, and stepping a standing robot
 * straight to 60 % duty in both directions (`rotate`) pulled the shared 5 V
 * rail low enough to brown out the XIAO: two BROWNOUT_RST resets captured on
 * J15 on 2026-11-07, one 1 ms after the planner's `rotate` goal and one during
 * the boot wheel exercise, while 50 s of speech in between caused none.
 *
 * Only a rise is limited. Lowering duty, stopping (coast) and braking are
 * applied on the same call, so the obstacle reflex is not slowed. A reversal
 * starts again from 0, since a motor driven against its own rotation draws
 * more than a stalled one.
 */
#define MOTOR_RAMP_FULL_SCALE_MS 300U

/**
 * @brief Longest interval the ramp credits between two calls, in ms.
 *
 * The rise allowed on a call is proportional to the time since the previous
 * write, and that time is unbounded after the robot has been parked. Capping it
 * keeps the first step from rest at
 * PCA9685_PWM_MAX * MOTOR_RAMP_DT_CAP_MS / MOTOR_RAMP_FULL_SCALE_MS (about 13 %
 * of full scale) however long the motors were idle. Set just above the
 * executor's 33 ms tick.
 */
#define MOTOR_RAMP_DT_CAP_MS 40U

#endif  // MOTOR_CONTROLLER_H
