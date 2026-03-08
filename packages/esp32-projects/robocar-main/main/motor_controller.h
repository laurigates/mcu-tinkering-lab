/**
 * @file motor_controller.h
 * @brief Hardware abstraction layer for TB6612FNG motor control
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Motor controller initialization
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t motor_controller_init(void);

/**
 * @brief Move robot forward at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_move_forward(uint8_t speed);

/**
 * @brief Move robot backward at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_move_backward(uint8_t speed);

/**
 * @brief Turn robot left at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_turn_left(uint8_t speed);

/**
 * @brief Turn robot right at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_turn_right(uint8_t speed);

/**
 * @brief Rotate robot clockwise at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_rotate_cw(uint8_t speed);

/**
 * @brief Rotate robot counter-clockwise at specified speed
 * @param speed Motor speed (0-255)
 * @return ESP_OK on success
 */
esp_err_t motor_rotate_ccw(uint8_t speed);

/**
 * @brief Stop all motors
 * @return ESP_OK on success
 */
esp_err_t motor_stop(void);

/**
 * @brief Set individual motor speed
 * @param left_speed Left motor speed (0-255)
 * @param right_speed Right motor speed (0-255)
 * @param left_direction 1 for forward, 0 for backward
 * @param right_direction 1 for forward, 0 for backward
 * @return ESP_OK on success
 */
esp_err_t motor_set_individual(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction,
                               uint8_t right_direction);

/**
 * @brief Get current motor state
 * @param left_speed Pointer to store left motor speed
 * @param right_speed Pointer to store right motor speed
 * @param left_direction Pointer to store left motor direction
 * @param right_direction Pointer to store right motor direction
 * @return ESP_OK on success
 */
esp_err_t motor_get_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                          uint8_t *right_direction);

#endif  // MOTOR_CONTROLLER_H
