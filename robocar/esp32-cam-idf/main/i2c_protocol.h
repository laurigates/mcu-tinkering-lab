/**
 * @file i2c_protocol.h
 * @brief I2C communication protocol between ESP32-CAM and main board
 */

#ifndef I2C_PROTOCOL_H
#define I2C_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// I2C Configuration for inter-board communication
// Note: GPIO 33,35,36,39 are input-only on ESP32-CAM, use bidirectional pins
#define I2C_COMM_SCL_IO             14    // SCL pin for ESP32-CAM communication 
#define I2C_COMM_SDA_IO             15    // SDA pin for ESP32-CAM communication
#define I2C_MASTER_NUM              I2C_NUM_0  // Use I2C_NUM_0 (I2C_NUM_1 used by camera SCCB)
#define I2C_MASTER_FREQ_HZ          100000  // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define I2C_SLAVE_ADDRESS           0x42  // Main board slave address
#define I2C_SLAVE_NUM               I2C_NUM_0  // Use I2C_NUM_0 (I2C_NUM_1 used by camera SCCB)
#define I2C_SLAVE_TX_BUF_LEN        128
#define I2C_SLAVE_RX_BUF_LEN        128

// Command Types
typedef enum {
    CMD_TYPE_MOVEMENT = 0x01,
    CMD_TYPE_SOUND    = 0x02,
    CMD_TYPE_SERVO    = 0x03,
    CMD_TYPE_DISPLAY  = 0x04,
    CMD_TYPE_STATUS   = 0x05,
    CMD_TYPE_PING     = 0x06
} i2c_command_type_t;

// Movement Commands
typedef enum {
    MOVE_FORWARD = 0x01,
    MOVE_BACKWARD = 0x02,
    MOVE_LEFT = 0x03,
    MOVE_RIGHT = 0x04,
    MOVE_ROTATE_CW = 0x05,
    MOVE_ROTATE_CCW = 0x06,
    MOVE_STOP = 0x07
} movement_command_t;

// Sound Commands
typedef enum {
    SOUND_BEEP = 0x01,
    SOUND_MELODY = 0x02,
    SOUND_ALERT = 0x03
} sound_command_t;

// Servo Commands
typedef enum {
    SERVO_PAN = 0x01,
    SERVO_TILT = 0x02
} servo_command_t;

// I2C Command Packet Structure
#define I2C_MAX_DATA_LEN 28
typedef struct __attribute__((packed)) {
    uint8_t command_type;           // Command type from i2c_command_type_t
    uint8_t data_length;            // Length of data array (0-28)
    uint8_t data[I2C_MAX_DATA_LEN]; // Command-specific data
    uint8_t checksum;               // Simple XOR checksum
} i2c_command_packet_t;

// Response packet from slave
typedef struct __attribute__((packed)) {
    uint8_t status;     // 0x00 = OK, 0x01 = ERROR, 0x02 = BUSY
    uint8_t data_length;
    uint8_t data[14];   // Response data
    uint8_t checksum;
} i2c_response_packet_t;

// Movement command data
typedef struct __attribute__((packed)) {
    uint8_t movement;   // movement_command_t
    uint8_t speed;      // 0-255
} movement_data_t;

// Sound command data
typedef struct __attribute__((packed)) {
    uint8_t sound_type; // sound_command_t
} sound_data_t;

// Servo command data
typedef struct __attribute__((packed)) {
    uint8_t servo_type; // servo_command_t
    uint8_t angle;      // 0-180 degrees
} servo_data_t;

// Display command data
typedef struct __attribute__((packed)) {
    uint8_t line;       // Display line 0-7
    char message[27];   // Message text (null-terminated)
} display_data_t;

// Status response data
typedef struct __attribute__((packed)) {
    uint8_t current_state;      // Current movement state
    uint8_t pan_angle;          // Current pan angle
    uint8_t tilt_angle;         // Current tilt angle
    uint8_t wifi_connected;     // WiFi connection status
    uint8_t battery_level;      // Battery level 0-100
} status_data_t;

// Protocol helper functions
uint8_t calculate_checksum(const uint8_t* data, size_t length);
bool verify_checksum(const uint8_t* data, size_t length, uint8_t checksum);
void prepare_movement_command(i2c_command_packet_t* packet, movement_command_t movement, uint8_t speed);
void prepare_sound_command(i2c_command_packet_t* packet, sound_command_t sound);
void prepare_servo_command(i2c_command_packet_t* packet, servo_command_t servo, uint8_t angle);
void prepare_display_command(i2c_command_packet_t* packet, uint8_t line, const char* message);
void prepare_ping_command(i2c_command_packet_t* packet);

#endif // I2C_PROTOCOL_H