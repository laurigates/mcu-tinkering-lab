#include "motor_controller.h"
#include "config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char* TAG = "MOTOR_CONTROLLER";

// Motor state
static motor_status_t motor_status = {
    .current_command = MOTOR_CMD_STOP,
    .left_speed = 0,
    .right_speed = 0,
    .command_duration_ms = 0,
    .total_runtime_ms = 0,
    .distance_traveled_mm = 0,
    .heading_degrees = 0.0,
    .is_running = false
};

static bool is_initialized = false;
static TickType_t last_update_tick = 0;

// Command names
static const char* command_names[] = {
    "STOP",
    "FORWARD",
    "BACKWARD",
    "LEFT",
    "RIGHT",
    "ROTATE_LEFT",
    "ROTATE_RIGHT"
};

// Initialize PWM channel
static esp_err_t init_pwm_channel(ledc_channel_t channel, int gpio, ledc_timer_t timer) {
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = 0,
        .hpoint = 0
    };
    return ledc_channel_config(&channel_config);
}

// Initialize motor controller
esp_err_t motor_controller_init(void) {
    if (is_initialized) {
        ESP_LOGW(TAG, "Motor controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing motor controller (mock mode)");

    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = MOTOR_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize PWM channels for motors (even in mock mode for realism)
    init_pwm_channel(LEDC_CHANNEL_0, MOTOR_LEFT_FORWARD_PIN, LEDC_TIMER_0);
    init_pwm_channel(LEDC_CHANNEL_1, MOTOR_LEFT_BACKWARD_PIN, LEDC_TIMER_0);
    init_pwm_channel(LEDC_CHANNEL_2, MOTOR_RIGHT_FORWARD_PIN, LEDC_TIMER_0);
    init_pwm_channel(LEDC_CHANNEL_3, MOTOR_RIGHT_BACKWARD_PIN, LEDC_TIMER_0);

    is_initialized = true;
    last_update_tick = xTaskGetTickCount();

    ESP_LOGI(TAG, "Motor controller initialized successfully");
    return ESP_OK;
}

// Set PWM duty cycle
static void set_motor_pwm(ledc_channel_t forward_channel, ledc_channel_t backward_channel,
                          int speed) {
    if (speed > 0) {
        // Forward
        ledc_set_duty(LEDC_LOW_SPEED_MODE, forward_channel, speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, backward_channel, 0);
    } else if (speed < 0) {
        // Backward
        ledc_set_duty(LEDC_LOW_SPEED_MODE, forward_channel, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, backward_channel, -speed);
    } else {
        // Stop
        ledc_set_duty(LEDC_LOW_SPEED_MODE, forward_channel, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, backward_channel, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, forward_channel);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, backward_channel);
}

// Execute motor command
esp_err_t motor_execute_command(motor_command_t command, int speed, uint32_t duration_ms) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Motor controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Executing command: %s, speed: %d, duration: %d ms",
             motor_get_command_name(command), speed, duration_ms);

    motor_status.current_command = command;
    motor_status.command_duration_ms = duration_ms;
    motor_status.is_running = (command != MOTOR_CMD_STOP);

    switch (command) {
        case MOTOR_CMD_FORWARD:
            motor_status.left_speed = speed;
            motor_status.right_speed = speed;
            break;

        case MOTOR_CMD_BACKWARD:
            motor_status.left_speed = -speed;
            motor_status.right_speed = -speed;
            break;

        case MOTOR_CMD_LEFT:
            motor_status.left_speed = speed / 2;
            motor_status.right_speed = speed;
            break;

        case MOTOR_CMD_RIGHT:
            motor_status.left_speed = speed;
            motor_status.right_speed = speed / 2;
            break;

        case MOTOR_CMD_ROTATE_LEFT:
            motor_status.left_speed = -speed;
            motor_status.right_speed = speed;
            break;

        case MOTOR_CMD_ROTATE_RIGHT:
            motor_status.left_speed = speed;
            motor_status.right_speed = -speed;
            break;

        case MOTOR_CMD_STOP:
        default:
            motor_status.left_speed = 0;
            motor_status.right_speed = 0;
            motor_status.is_running = false;
            break;
    }

    // Set motor speeds (mock - just sets PWM for demonstration)
    set_motor_pwm(LEDC_CHANNEL_0, LEDC_CHANNEL_1, motor_status.left_speed);
    set_motor_pwm(LEDC_CHANNEL_2, LEDC_CHANNEL_3, motor_status.right_speed);

    // If duration specified, stop after duration
    if (duration_ms > 0 && command != MOTOR_CMD_STOP) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        motor_stop();
    }

    return ESP_OK;
}

// Stop all motors
esp_err_t motor_stop(void) {
    return motor_execute_command(MOTOR_CMD_STOP, 0, 0);
}

// Set motor speeds directly
esp_err_t motor_set_speeds(int left_speed, int right_speed) {
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp speeds to valid range
    left_speed = (left_speed > 255) ? 255 : (left_speed < -255) ? -255 : left_speed;
    right_speed = (right_speed > 255) ? 255 : (right_speed < -255) ? -255 : right_speed;

    motor_status.left_speed = left_speed;
    motor_status.right_speed = right_speed;
    motor_status.is_running = (left_speed != 0 || right_speed != 0);

    // Determine command based on speeds
    if (left_speed == 0 && right_speed == 0) {
        motor_status.current_command = MOTOR_CMD_STOP;
    } else if (left_speed > 0 && right_speed > 0 && abs(left_speed - right_speed) < 10) {
        motor_status.current_command = MOTOR_CMD_FORWARD;
    } else if (left_speed < 0 && right_speed < 0 && abs(left_speed - right_speed) < 10) {
        motor_status.current_command = MOTOR_CMD_BACKWARD;
    } else if (left_speed < right_speed) {
        motor_status.current_command = MOTOR_CMD_LEFT;
    } else if (right_speed < left_speed) {
        motor_status.current_command = MOTOR_CMD_RIGHT;
    }

    set_motor_pwm(LEDC_CHANNEL_0, LEDC_CHANNEL_1, left_speed);
    set_motor_pwm(LEDC_CHANNEL_2, LEDC_CHANNEL_3, right_speed);

    return ESP_OK;
}

// Get current motor status
motor_status_t motor_get_status(void) {
    return motor_status;
}

// Get human-readable command name
const char* motor_get_command_name(motor_command_t command) {
    if (command < 0 || command >= sizeof(command_names) / sizeof(command_names[0])) {
        return "UNKNOWN";
    }
    return command_names[command];
}

// Simulate movement and update position
esp_err_t motor_simulate_movement(uint32_t delta_time_ms) {
    if (!motor_status.is_running) {
        return ESP_OK;
    }

    // Update total runtime
    motor_status.total_runtime_ms += delta_time_ms;

    // Calculate simulated distance based on speed
    // Assume max speed (255) = 300mm/s
    float speed_factor = ((abs(motor_status.left_speed) + abs(motor_status.right_speed)) / 2.0) / 255.0;
    float distance_delta = speed_factor * 300.0 * (delta_time_ms / 1000.0);

    if (motor_status.current_command == MOTOR_CMD_FORWARD) {
        motor_status.distance_traveled_mm += distance_delta;
    } else if (motor_status.current_command == MOTOR_CMD_BACKWARD) {
        motor_status.distance_traveled_mm -= distance_delta;
    }

    // Calculate heading change for turns
    // Assume max rotation speed = 90 degrees/second
    if (motor_status.current_command == MOTOR_CMD_ROTATE_LEFT ||
        motor_status.current_command == MOTOR_CMD_LEFT) {
        float rotation_speed = (motor_status.current_command == MOTOR_CMD_ROTATE_LEFT) ? 90.0 : 45.0;
        motor_status.heading_degrees -= rotation_speed * (delta_time_ms / 1000.0);
    } else if (motor_status.current_command == MOTOR_CMD_ROTATE_RIGHT ||
               motor_status.current_command == MOTOR_CMD_RIGHT) {
        float rotation_speed = (motor_status.current_command == MOTOR_CMD_ROTATE_RIGHT) ? 90.0 : 45.0;
        motor_status.heading_degrees += rotation_speed * (delta_time_ms / 1000.0);
    }

    // Normalize heading to 0-359
    while (motor_status.heading_degrees < 0) {
        motor_status.heading_degrees += 360.0;
    }
    while (motor_status.heading_degrees >= 360.0) {
        motor_status.heading_degrees -= 360.0;
    }

    return ESP_OK;
}

// Get telemetry string for reporting
char* motor_get_telemetry_string(void) {
    static char telemetry[256];

    snprintf(telemetry, sizeof(telemetry),
             "*Motor Telemetry*\n"
             "Command: %s\n"
             "Left Speed: %d\n"
             "Right Speed: %d\n"
             "Distance: %.1f mm\n"
             "Heading: %.1f°\n"
             "Runtime: %lu ms\n"
             "Status: %s",
             motor_get_command_name(motor_status.current_command),
             motor_status.left_speed,
             motor_status.right_speed,
             motor_status.distance_traveled_mm,
             motor_status.heading_degrees,
             motor_status.total_runtime_ms,
             motor_status.is_running ? "Running" : "Stopped");

    return telemetry;
}

// Cleanup motor controller
void motor_controller_cleanup(void) {
    if (!is_initialized) {
        return;
    }

    // Stop all motors
    motor_stop();

    // Stop PWM
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

    is_initialized = false;
    ESP_LOGI(TAG, "Motor controller cleaned up");
}