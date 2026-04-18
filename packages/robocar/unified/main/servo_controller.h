/**
 * @file servo_controller.h
 * @brief Hardware abstraction layer for servo motor control (pan/tilt camera mount)
 *
 * Same public API as robocar-main. Internal implementation uses PCA9685 at
 * 200Hz via TCA9548A I2C bus.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum { SERVO_PAN = 0, SERVO_TILT = 1 } servo_id_t;

typedef struct {
    int16_t pan_angle;
    int16_t tilt_angle;
} servo_position_t;

#define SERVO_PAN_MIN_ANGLE -90
#define SERVO_PAN_MAX_ANGLE 90
#define SERVO_PAN_CENTER 0
#define SERVO_TILT_MIN_ANGLE -45
#define SERVO_TILT_MAX_ANGLE 45
#define SERVO_TILT_CENTER 0

esp_err_t servo_controller_init(void);
esp_err_t servo_set_angle(servo_id_t servo_id, int16_t angle);
esp_err_t servo_set_pan(int16_t angle);
esp_err_t servo_set_tilt(int16_t angle);
esp_err_t servo_set_position(int16_t pan_angle, int16_t tilt_angle);
esp_err_t servo_center(servo_id_t servo_id);
esp_err_t servo_center_all(void);
esp_err_t servo_get_angle(servo_id_t servo_id, int16_t *angle);
esp_err_t servo_get_position(servo_position_t *position);
esp_err_t servo_disable(servo_id_t servo_id);
esp_err_t servo_disable_all(void);
esp_err_t servo_enable(servo_id_t servo_id);
esp_err_t servo_enable_all(void);
bool servo_is_initialized(void);
bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle);
esp_err_t servo_move_smooth(servo_id_t servo_id, int16_t target_angle, uint8_t step_size,
                            uint32_t delay_ms);
esp_err_t servo_sweep(servo_id_t servo_id, int16_t start_angle, int16_t end_angle,
                      uint8_t step_size, uint32_t delay_ms, uint32_t cycles);
esp_err_t servo_stop_motion(void);

#endif  // SERVO_CONTROLLER_H
