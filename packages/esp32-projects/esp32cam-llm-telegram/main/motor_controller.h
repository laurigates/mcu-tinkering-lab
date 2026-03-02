#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Motor commands
typedef enum {
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_FORWARD,
    MOTOR_CMD_BACKWARD,
    MOTOR_CMD_LEFT,
    MOTOR_CMD_RIGHT,
    MOTOR_CMD_ROTATE_LEFT,
    MOTOR_CMD_ROTATE_RIGHT
} motor_command_t;

// Motor status
typedef struct {
    motor_command_t current_command;
    int left_speed;   // -255 to 255
    int right_speed;  // -255 to 255
    uint32_t command_duration_ms;
    uint32_t total_runtime_ms;
    uint32_t distance_traveled_mm;  // Mock distance
    float heading_degrees;          // Mock heading (0-359)
    bool is_running;
} motor_status_t;

// Initialize motor controller
esp_err_t motor_controller_init(void);

// Execute motor command
esp_err_t motor_execute_command(motor_command_t command, int speed, uint32_t duration_ms);

// Stop all motors
esp_err_t motor_stop(void);

// Set motor speeds directly
esp_err_t motor_set_speeds(int left_speed, int right_speed);

// Get current motor status
motor_status_t motor_get_status(void);

// Get human-readable command name
const char *motor_get_command_name(motor_command_t command);

// Simulate movement and update position
esp_err_t motor_simulate_movement(uint32_t delta_time_ms);

// Get telemetry string for reporting
char *motor_get_telemetry_string(void);

// Cleanup motor controller
void motor_controller_cleanup(void);

#endif  // MOTOR_CONTROLLER_H
