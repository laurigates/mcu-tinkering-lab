/**
 * @file servo_controller.h
 * @brief Hardware abstraction layer for servo motor control (pan/tilt camera mount)
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Servo identifier
 */
typedef enum {
    SERVO_PAN = 0,
    SERVO_TILT = 1
} servo_id_t;

/**
 * @brief Servo position structure
 */
typedef struct {
    int16_t pan_angle;   // Pan angle in degrees (-90 to +90)
    int16_t tilt_angle;  // Tilt angle in degrees (-45 to +45)
} servo_position_t;

/**
 * @brief Servo configuration limits
 */
#define SERVO_PAN_MIN_ANGLE   -90
#define SERVO_PAN_MAX_ANGLE   90
#define SERVO_PAN_CENTER      0

#define SERVO_TILT_MIN_ANGLE  -45
#define SERVO_TILT_MAX_ANGLE  45
#define SERVO_TILT_CENTER     0

/**
 * @brief Initialize servo controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t servo_controller_init(void);

/**
 * @brief Set servo angle
 * @param servo_id Servo to control (PAN or TILT)
 * @param angle Angle in degrees (range depends on servo)
 * @return ESP_OK on success
 */
esp_err_t servo_set_angle(servo_id_t servo_id, int16_t angle);

/**
 * @brief Set pan servo angle
 * @param angle Pan angle in degrees (-90 to +90)
 * @return ESP_OK on success
 */
esp_err_t servo_set_pan(int16_t angle);

/**
 * @brief Set tilt servo angle
 * @param angle Tilt angle in degrees (-45 to +45)
 * @return ESP_OK on success
 */
esp_err_t servo_set_tilt(int16_t angle);

/**
 * @brief Set both servo angles simultaneously
 * @param pan_angle Pan angle in degrees (-90 to +90)
 * @param tilt_angle Tilt angle in degrees (-45 to +45)
 * @return ESP_OK on success
 */
esp_err_t servo_set_position(int16_t pan_angle, int16_t tilt_angle);

/**
 * @brief Move servo to center position
 * @param servo_id Servo to center (PAN or TILT)
 * @return ESP_OK on success
 */
esp_err_t servo_center(servo_id_t servo_id);

/**
 * @brief Move both servos to center position
 * @return ESP_OK on success
 */
esp_err_t servo_center_all(void);

/**
 * @brief Get current servo angle
 * @param servo_id Servo to query (PAN or TILT)
 * @param angle Pointer to store current angle
 * @return ESP_OK on success
 */
esp_err_t servo_get_angle(servo_id_t servo_id, int16_t *angle);

/**
 * @brief Get current servo position
 * @param position Pointer to store current position
 * @return ESP_OK on success
 */
esp_err_t servo_get_position(servo_position_t *position);

/**
 * @brief Disable servo (stop PWM signal)
 * @param servo_id Servo to disable (PAN or TILT)
 * @return ESP_OK on success
 */
esp_err_t servo_disable(servo_id_t servo_id);

/**
 * @brief Disable all servos
 * @return ESP_OK on success
 */
esp_err_t servo_disable_all(void);

/**
 * @brief Enable servo (start PWM signal)
 * @param servo_id Servo to enable (PAN or TILT)
 * @return ESP_OK on success
 */
esp_err_t servo_enable(servo_id_t servo_id);

/**
 * @brief Enable all servos
 * @return ESP_OK on success
 */
esp_err_t servo_enable_all(void);

/**
 * @brief Check if servo controller is initialized
 * @return true if initialized, false otherwise
 */
bool servo_is_initialized(void);

/**
 * @brief Check if servo angle is within valid range
 * @param servo_id Servo to check (PAN or TILT)
 * @param angle Angle to validate
 * @return true if valid, false otherwise
 */
bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle);

/**
 * @brief Smoothly move servo to target angle
 * @param servo_id Servo to move (PAN or TILT)
 * @param target_angle Target angle in degrees
 * @param step_size Step size in degrees per movement
 * @param delay_ms Delay between steps in milliseconds
 * @return ESP_OK on success
 */
esp_err_t servo_move_smooth(servo_id_t servo_id, int16_t target_angle, 
                           uint8_t step_size, uint32_t delay_ms);

/**
 * @brief Perform servo sweep motion
 * @param servo_id Servo to sweep (PAN or TILT)
 * @param start_angle Starting angle
 * @param end_angle Ending angle
 * @param step_size Step size in degrees
 * @param delay_ms Delay between steps
 * @param cycles Number of sweep cycles (0 = infinite)
 * @return ESP_OK on success
 */
esp_err_t servo_sweep(servo_id_t servo_id, int16_t start_angle, int16_t end_angle,
                     uint8_t step_size, uint32_t delay_ms, uint32_t cycles);

/**
 * @brief Stop any ongoing servo motion
 * @return ESP_OK on success
 */
esp_err_t servo_stop_motion(void);

#endif // SERVO_CONTROLLER_H