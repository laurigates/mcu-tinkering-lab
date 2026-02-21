/**
 * @file system_config.h
 * @brief Centralized configuration management for main controller
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "pin_config_idf.h"

// ========================================
// System Configuration
// ========================================

// System timing configuration
#define SYSTEM_COMMAND_TIMEOUT_MS       1000    // Command timeout in milliseconds
#define SYSTEM_STABILIZATION_DELAY_MS   1000    // System stabilization delay
#define SYSTEM_TASK_DELAY_MS            100     // Default task delay
#define COMMAND_TIMEOUT_MS              SYSTEM_COMMAND_TIMEOUT_MS
#define MUTEX_TIMEOUT_MS                5000    // Mutex/semaphore acquisition timeout

// Task stack sizes
#define COMMAND_TASK_STACK_SIZE         8192    // Stack for command_task (UART + string + I2C ops)

// Command buffer configuration
#define MAX_COMMAND_LENGTH              32      // Maximum command string length
#define ACTION_STRING_LENGTH            32      // Action description length
#define COMMAND_SOURCE_LENGTH           16      // Command source description length

// Hardware value limits
#define COLOR_VALUE_MAX                 255     // Maximum 8-bit color/PWM value
#define PCA9685_PWM_MAX                 4095    // Maximum 12-bit PCA9685 PWM value

// State definitions (centralized from scattered defines)
typedef enum {
    STATE_STOPPED = 0,
    STATE_FORWARD = 1,
    STATE_BACKWARD = 2,
    STATE_LEFT = 3,
    STATE_RIGHT = 4,
    STATE_ROTATE_CW = 5,
    STATE_ROTATE_CCW = 6
} system_state_t;

// ========================================
// Command Configuration
// ========================================

// Command string definitions (centralized)
#define CMD_FORWARD         "F"
#define CMD_BACKWARD        "B"
#define CMD_LEFT            "L"
#define CMD_RIGHT           "R"
#define CMD_STOP            "S"
#define CMD_ROTATE_CW       "C"
#define CMD_ROTATE_CCW      "W"
#define CMD_SOUND_BEEP      "SB"
#define CMD_SOUND_MELODY    "SM"
#define CMD_SOUND_ALERT     "SA"
#define CMD_SERVO_PAN       "PAN"
#define CMD_SERVO_TILT      "TILT"
#define CMD_DISPLAY         "DISP"
#define CMD_HELP            "HELP"

// ========================================
// Hardware Configuration
// ========================================

// Motor configuration
#define MOTOR_DEFAULT_SPEED     DEFAULT_SPEED   // From pin_config_idf.h

// LED configuration (using PCA9685 channels)
typedef struct {
    uint8_t red_channel;
    uint8_t green_channel;
    uint8_t blue_channel;
} led_config_t;

#define LEFT_LED_CONFIG  { LED_LEFT_R_CHANNEL, LED_LEFT_G_CHANNEL, LED_LEFT_B_CHANNEL }
#define RIGHT_LED_CONFIG { LED_RIGHT_R_CHANNEL, LED_RIGHT_G_CHANNEL, LED_RIGHT_B_CHANNEL }

// Servo configuration (using values from pin_config_idf.h)
typedef struct {
    uint8_t pan_channel;
    uint8_t tilt_channel;
    int default_angle;
    int min_angle;
    int max_angle;
} servo_config_t;

#define SERVO_SYSTEM_CONFIG { \
    .pan_channel = SERVO_PAN_CHANNEL, \
    .tilt_channel = SERVO_TILT_CHANNEL, \
    .default_angle = SERVO_DEFAULT_ANGLE, \
    .min_angle = SERVO_MIN_ANGLE, \
    .max_angle = SERVO_MAX_ANGLE \
}

// ========================================
// Communication Configuration
// ========================================

// UART configuration
#define CONSOLE_UART_NUM        UART_NUM_0
#define CONSOLE_BAUD_RATE       115200
#define CONSOLE_BUFFER_SIZE     1024

// I2C configuration (using values from pin_config_idf.h)
#define I2C_MASTER_SDA          I2C_SDA_PIN
#define I2C_MASTER_SCL          I2C_SCL_PIN
#define I2C_MASTER_FREQ         100000

// ========================================
// Display Configuration
// ========================================

// OLED display configuration (using values from pin_config_idf.h)
typedef struct {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t rst_pin;
    uint8_t i2c_address;
    uint16_t width;
    uint16_t height;
} display_config_t;

#define OLED_DISPLAY_CONFIG { \
    .sda_pin = OLED_SDA_PIN, \
    .scl_pin = OLED_SCL_PIN, \
    .rst_pin = OLED_RST_PIN, \
    .i2c_address = OLED_I2C_ADDR, \
    .width = OLED_WIDTH, \
    .height = OLED_HEIGHT \
}

// ========================================
// Feature Flags
// ========================================

// System feature enablement (centralized from pin_config_idf.h)
#define FEATURE_PIEZO_ENABLED       PIEZO_ENABLED
#define FEATURE_PCA9685_ENABLED     PCA9685_ENABLED
#define FEATURE_OLED_ENABLED        OLED_ENABLED
#define FEATURE_WIFI_ENABLED        true
#define FEATURE_I2C_SLAVE_ENABLED   true

// Debug and logging configuration
#define DEBUG_VERBOSE_ACTIONS       true
#define DEBUG_SHOW_PIN_CONFIG       true
#define DEBUG_ACTION_COUNTER        true

#endif // SYSTEM_CONFIG_H