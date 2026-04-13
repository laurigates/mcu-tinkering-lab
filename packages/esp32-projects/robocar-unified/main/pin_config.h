/**
 * @file pin_config.h
 * @brief XIAO ESP32-S3 Sense pin definitions for robocar-unified
 *
 * GPIO Budget (11 available on XIAO headers):
 *   D4/D5 (GPIO5/6)  = I2C SDA/SCL to TCA9548A
 *   D0 (GPIO1)       = TB6612FNG STBY (motor enable)
 *   D1 (GPIO2)       = Piezo buzzer
 *   D2-D3 (GPIO3-4)  = Spare (future: ultrasonic sensor)
 *   D6/D7 (GPIO43/44)= USB Serial TX/RX (debug console)
 *   D8-D10 (GPIO7-9) = Spare (future: SPI expansion)
 *
 * Camera uses internal GPIOs on the Sense module — no conflict with D0-D10.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/gpio.h"

// ========================================
// I2C Bus (to TCA9548A multiplexer)
// ========================================
#define I2C_SDA_PIN GPIO_NUM_5     // XIAO D4/SDA
#define I2C_SCL_PIN GPIO_NUM_6     // XIAO D5/SCL
#define I2C_MASTER_FREQ_HZ 400000  // 400kHz for fast I2C

// ========================================
// TCA9548A I2C Multiplexer
// ========================================
#define TCA9548A_ADDR 0x70  // Default address (A0=A1=A2=GND)

// TCA9548A channel assignments
#define I2C_BUS_CHANNEL_PCA9685 0  // Channel 0: PCA9685 PWM driver
#define I2C_BUS_CHANNEL_OLED 1     // Channel 1: SSD1306 OLED display
// Channels 2-7: reserved for future sensors (IMU, ToF, etc.)

// ========================================
// PCA9685 PWM Driver (via TCA9548A ch0)
// ========================================
#define PCA9685_ADDR 0x40    // Default PCA9685 address
#define PCA9685_FREQ_HZ 200  // 200Hz: compromise for servos + motors + LEDs

// LED channels (2x RGB)
#define LED_LEFT_R_CHANNEL 0
#define LED_LEFT_G_CHANNEL 1
#define LED_LEFT_B_CHANNEL 2
#define LED_RIGHT_R_CHANNEL 3
#define LED_RIGHT_G_CHANNEL 4
#define LED_RIGHT_B_CHANNEL 5

// Servo channels
#define SERVO_PAN_CHANNEL 6
#define SERVO_TILT_CHANNEL 7

// Motor channels (TB6612FNG via PCA9685 digital/PWM outputs)
#define MOTOR_RIGHT_IN1_CHANNEL 8   // Direction: digital 0 or 4096
#define MOTOR_RIGHT_IN2_CHANNEL 9   // Direction: digital 0 or 4096
#define MOTOR_RIGHT_PWM_CHANNEL 10  // Speed: PWM 0-4095
#define MOTOR_LEFT_IN1_CHANNEL 11   // Direction: digital 0 or 4096
#define MOTOR_LEFT_IN2_CHANNEL 12   // Direction: digital 0 or 4096
#define MOTOR_LEFT_PWM_CHANNEL 13   // Speed: PWM 0-4095
// Channels 14-15: reserved for future expansion

// PCA9685 digital output values (for direction pins)
#define PCA9685_FULL_ON 4096  // Bit 12 set = full-on
#define PCA9685_FULL_OFF 0    // All bits clear = full-off

// PCA9685 PWM resolution
#define PCA9685_PWM_MAX 4095  // 12-bit resolution

// ========================================
// Motor Control (TB6612FNG)
// ========================================
#define MOTOR_STBY_PIN GPIO_NUM_1  // XIAO D0: HIGH = motors enabled
#define DEFAULT_SPEED 3200         // Default motor speed (~78% of 4095)

// ========================================
// SSD1306 OLED Display (via TCA9548A ch1)
// ========================================
#define OLED_I2C_ADDR 0x3C  // SSD1306 default address
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ========================================
// Piezo Buzzer
// ========================================
#define PIEZO_PIN GPIO_NUM_2  // XIAO D1

// ========================================
// Servo Configuration (SG90 at 200Hz)
// ========================================
// At 200Hz, period = 5000us. Pulse width range: 500-2500us.
// PCA9685 counts = (pulse_us * 4096) / 5000
#define SERVO_PERIOD_US 5000
#define SERVO_MIN_PULSE_US 500      // 0 degrees
#define SERVO_MAX_PULSE_US 2500     // 180 degrees
#define SERVO_CENTER_PULSE_US 1500  // 90 degrees

// Convert pulse width (us) to PCA9685 count at 200Hz
#define SERVO_PULSE_TO_COUNT(pulse_us) ((uint16_t)(((uint32_t)(pulse_us) * 4096) / SERVO_PERIOD_US))

#define SERVO_MIN_COUNT SERVO_PULSE_TO_COUNT(SERVO_MIN_PULSE_US)
#define SERVO_MAX_COUNT SERVO_PULSE_TO_COUNT(SERVO_MAX_PULSE_US)
#define SERVO_CENTER_COUNT SERVO_PULSE_TO_COUNT(SERVO_CENTER_PULSE_US)

// Servo range limits (degrees from center)
#define SERVO_PAN_MIN_DEG -90
#define SERVO_PAN_MAX_DEG 90
#define SERVO_TILT_MIN_DEG -45
#define SERVO_TILT_MAX_DEG 45

// ========================================
// FreeRTOS Task Configuration
// ========================================
#define MOTOR_TASK_STACK_SIZE 4096
#define MOTOR_TASK_PRIORITY 6
#define MOTOR_TASK_CORE 0

#define PERIPHERAL_TASK_STACK_SIZE 4096
#define PERIPHERAL_TASK_PRIORITY 4
#define PERIPHERAL_TASK_CORE 0

#define CAMERA_TASK_STACK_SIZE 16384
#define CAMERA_TASK_PRIORITY 5
#define CAMERA_TASK_CORE 1

#define AI_TASK_STACK_SIZE 16384
#define AI_TASK_PRIORITY 3
#define AI_TASK_CORE 1

#define NETWORK_TASK_STACK_SIZE 8192
#define NETWORK_TASK_PRIORITY 4
#define NETWORK_TASK_CORE 1

#define COMMAND_TASK_STACK_SIZE 8192
#define COMMAND_TASK_PRIORITY 5
#define COMMAND_TASK_CORE 0

// ========================================
// Queue Configuration
// ========================================
#define MOTOR_CMD_QUEUE_DEPTH 8
#define PERIPHERAL_CMD_QUEUE_DEPTH 16

// ========================================
// Timing
// ========================================
#define COMMAND_TIMEOUT_MS 1000   // Stop motors if no command for this long
#define CAPTURE_INTERVAL_MS 5000  // AI capture interval

#endif  // PIN_CONFIG_H
