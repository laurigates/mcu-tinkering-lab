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
#define I2C_COMM_SCL_IO             26    // SCL pin for ESP32-CAM communication (bidirectional)
#define I2C_COMM_SDA_IO             27    // SDA pin for ESP32-CAM communication (bidirectional)
#define I2C_MASTER_NUM              I2C_NUM_1  // Use I2C_NUM_1 to avoid conflict with PCA9685
#define I2C_MASTER_FREQ_HZ          100000  // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define I2C_SLAVE_ADDRESS           0x42  // Main board slave address
#define I2C_SLAVE_NUM               I2C_NUM_1  // Use I2C_NUM_1 to avoid conflict with PCA9685
#define I2C_SLAVE_TX_BUF_LEN        128
#define I2C_SLAVE_RX_BUF_LEN        128

// Command Types
typedef enum {
    CMD_TYPE_MOVEMENT = 0x01,
    CMD_TYPE_SOUND    = 0x02,
    CMD_TYPE_SERVO    = 0x03,
    CMD_TYPE_DISPLAY  = 0x04,
    CMD_TYPE_STATUS   = 0x05,
    CMD_TYPE_PING     = 0x06,
    // OTA Commands
    CMD_TYPE_ENTER_MAINTENANCE_MODE = 0x50,
    CMD_TYPE_BEGIN_OTA              = 0x51,
    CMD_TYPE_GET_OTA_STATUS         = 0x52,
    CMD_TYPE_GET_VERSION            = 0x53,
    CMD_TYPE_REBOOT                 = 0x54
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
#define I2C_MAX_DATA_LEN 26  // Reduced to make room for sequence number
typedef struct __attribute__((packed)) {
    uint8_t command_type;           // Command type from i2c_command_type_t
    uint8_t sequence_number;        // Sequence number for tracking commands
    uint8_t data_length;            // Length of data array (0-26)
    uint8_t data[I2C_MAX_DATA_LEN]; // Command-specific data
    uint8_t checksum;               // Simple XOR checksum
} i2c_command_packet_t;

// Response packet from slave
typedef struct __attribute__((packed)) {
    uint8_t status;          // 0x00 = OK, 0x01 = ERROR, 0x02 = BUSY, 0x03 = INVALID_SEQ
    uint8_t sequence_number; // Echo back the sequence number from request
    uint8_t data_length;
    uint8_t data[13];        // Response data (reduced by 1 for sequence number)
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

// OTA Status
typedef enum {
    OTA_STATUS_IDLE = 0x00,
    OTA_STATUS_IN_PROGRESS = 0x01,
    OTA_STATUS_SUCCESS = 0x02,
    OTA_STATUS_FAILED = 0x03,
    OTA_STATUS_MAINTENANCE_MODE = 0x04
} ota_status_t;

// OTA Begin command data
#define OTA_URL_MAX_LEN 20
#define OTA_HASH_LEN 4  // First 4 bytes of SHA256 hash for verification
typedef struct __attribute__((packed)) {
    char url[OTA_URL_MAX_LEN];  // OTA firmware URL (truncated if needed)
    uint8_t hash[OTA_HASH_LEN]; // Hash prefix for verification
    uint8_t url_length;         // Actual URL length (may be > OTA_URL_MAX_LEN)
} ota_begin_data_t;

// OTA Status response data
typedef struct __attribute__((packed)) {
    uint8_t status;             // ota_status_t
    uint8_t progress;           // Progress percentage 0-100
    uint8_t error_code;         // Error code if status is FAILED
} ota_status_response_t;

// Version response data
#define VERSION_STRING_LEN 12
typedef struct __attribute__((packed)) {
    char version[VERSION_STRING_LEN];  // Version string (e.g., "1.0.0")
} version_response_t;

// Protocol helper functions
uint8_t calculate_checksum(const uint8_t* data, size_t length);
bool verify_checksum(const uint8_t* data, size_t length, uint8_t checksum);
void prepare_movement_command(i2c_command_packet_t* packet, movement_command_t movement, uint8_t speed, uint8_t seq_num);
void prepare_sound_command(i2c_command_packet_t* packet, sound_command_t sound, uint8_t seq_num);
void prepare_servo_command(i2c_command_packet_t* packet, servo_command_t servo, uint8_t angle, uint8_t seq_num);
void prepare_display_command(i2c_command_packet_t* packet, uint8_t line, const char* message, uint8_t seq_num);
void prepare_ping_command(i2c_command_packet_t* packet, uint8_t seq_num);

// OTA command helpers
void prepare_enter_maintenance_command(i2c_command_packet_t* packet, uint8_t seq_num);
void prepare_begin_ota_command(i2c_command_packet_t* packet, const char* url, const uint8_t* hash, uint8_t seq_num);
void prepare_get_ota_status_command(i2c_command_packet_t* packet, uint8_t seq_num);
void prepare_get_version_command(i2c_command_packet_t* packet, uint8_t seq_num);
void prepare_reboot_command(i2c_command_packet_t* packet, uint8_t seq_num);

#endif // I2C_PROTOCOL_H