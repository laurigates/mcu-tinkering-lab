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

#endif  // MOTOR_CONTROLLER_H
