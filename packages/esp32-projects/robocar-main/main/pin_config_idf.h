#ifndef PIN_CONFIG_IDF_H
#define PIN_CONFIG_IDF_H

/*
 * RoboCar Pin Configuration for ESP-IDF
 * ------------------------------------
 * This file contains all hardware pin definitions and fixed hardware constants
 * for the RoboCar project using ESP-IDF framework.
 */

// Pin definitions for Heltec WiFi LoRa 32 V1 (avoiding all conflicts)
// Avoiding: LoRa SPI (5,14,18,19,26,27,32,33), Serial2 (16,17), I2C (21,22)
#define RIGHT_PWMA_PIN 23  // PWM for right motor
#define RIGHT_IN1_PIN 2    // Direction control 1 for right motor
#define RIGHT_IN2_PIN 4    // Direction control 2 for right motor
#define STBY_PIN 25        // Standby pin for motor driver
#define LEFT_IN1_PIN 0    // Direction control 1 for left motor (moved from 15 due to OLED conflict)
#define LEFT_IN2_PIN 12   // Direction control 2 for left motor
#define LEFT_PWMB_PIN 13  // PWM for left motor
#define PIEZO_PIN 32      // Piezo buzzer (using 32, avoiding LoRa may not be critical)

// I2C Pins for PCA9685 (shared bus)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// SSD1306 OLED Display Pins (Heltec WiFi LoRa 32 V1 integrated display)
// Note: Heltec integrated OLED uses fixed pins: SDA=4, SCL=15, RST=16
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 15
#define OLED_RST_PIN 16
#define OLED_I2C_ADDR 0x3C

// UART Pins for ESP32-CAM communication (moved due to OLED conflict)
#define UART_CAM_TX_PIN 17
#define UART_CAM_RX_PIN 35  // Changed from 16 (conflicts with OLED_RST)

// PWM Configuration
#define PWM_FREQ_HZ 5000  // PWM frequency in Hz
#define PWM_RESOLUTION 8  // PWM resolution in bits (8 = 0-255)
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_HIGH_SPEED_MODE
#define PWM_RIGHT_CHANNEL LEDC_CHANNEL_0
#define PWM_LEFT_CHANNEL LEDC_CHANNEL_1

// Motor speed configuration
#define DEFAULT_SPEED 200  // Default motor speed (0-255)

// PCA9685 Configuration
#define PCA9685_I2C_ADDR 0x40  // Default I2C address for PCA9685
#define PCA9685_FREQ_LED 1000  // PWM frequency for LEDs (Hz)
#define PCA9685_FREQ_SERVO 50  // PWM frequency for servos (Hz)

// PCA9685 Channel Assignments for LEDs
#define LED_LEFT_R_CHANNEL 0   // Left RGB LED - Red channel
#define LED_LEFT_G_CHANNEL 1   // Left RGB LED - Green channel
#define LED_LEFT_B_CHANNEL 2   // Left RGB LED - Blue channel
#define LED_RIGHT_R_CHANNEL 3  // Right RGB LED - Red channel
#define LED_RIGHT_G_CHANNEL 4  // Right RGB LED - Green channel
#define LED_RIGHT_B_CHANNEL 5  // Right RGB LED - Blue channel

// PCA9685 Channel Assignments for Servos
#define SERVO_PAN_CHANNEL 6   // Pan servo channel
#define SERVO_TILT_CHANNEL 7  // Tilt servo channel

// Servo Configuration
#define SERVO_MIN_ANGLE 0       // Minimum servo angle (degrees)
#define SERVO_MAX_ANGLE 180     // Maximum servo angle (degrees)
#define SERVO_DEFAULT_ANGLE 90  // Default servo position (degrees)
#define SERVO_MIN_PULSE 205     // Min pulse length for SG90 (0 degrees)
#define SERVO_MAX_PULSE 410     // Max pulse length for SG90 (180 degrees)

// Display Configuration
#define OLED_WIDTH 128  // OLED display width in pixels
#define OLED_HEIGHT 64  // OLED display height in pixels

// System Configuration Flags
#define PIEZO_ENABLED true    // Enable/disable piezo buzzer
#define PCA9685_ENABLED true  // Enable/disable PCA9685 functionality
#define OLED_ENABLED true     // Enable/disable OLED display

#endif  // PIN_CONFIG_IDF_H
