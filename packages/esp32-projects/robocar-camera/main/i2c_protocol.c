/**
 * @file i2c_protocol.c
 * @brief I2C communication protocol implementation
 */

#include "i2c_protocol.h"
#include "esp_log.h"
#include <string.h>

// Verify that data structs fit within the packet data field at compile time
_Static_assert(sizeof(movement_data_t) <= I2C_MAX_DATA_LEN,
    "movement_data_t exceeds I2C_MAX_DATA_LEN");
_Static_assert(sizeof(sound_data_t) <= I2C_MAX_DATA_LEN,
    "sound_data_t exceeds I2C_MAX_DATA_LEN");
_Static_assert(sizeof(servo_data_t) <= I2C_MAX_DATA_LEN,
    "servo_data_t exceeds I2C_MAX_DATA_LEN");
_Static_assert(sizeof(display_data_t) <= I2C_MAX_DATA_LEN,
    "display_data_t exceeds I2C_MAX_DATA_LEN");
_Static_assert(sizeof(ota_begin_data_t) <= I2C_MAX_DATA_LEN,
    "ota_begin_data_t exceeds I2C_MAX_DATA_LEN");

uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool verify_checksum(const uint8_t* data, size_t length, uint8_t checksum) {
    return calculate_checksum(data, length) == checksum;
}

void prepare_movement_command(i2c_command_packet_t* packet, movement_command_t movement, uint8_t speed, uint8_t seq_num) {
    if (!packet) return;

    movement_data_t* move_data = (movement_data_t*)packet->data;

    packet->command_type = CMD_TYPE_MOVEMENT;
    packet->sequence_number = seq_num;
    packet->data_length = sizeof(movement_data_t);

    move_data->movement = movement;
    move_data->speed = speed;

    // Calculate checksum for entire packet except checksum field
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_sound_command(i2c_command_packet_t* packet, sound_command_t sound, uint8_t seq_num) {
    if (!packet) return;

    sound_data_t* sound_data = (sound_data_t*)packet->data;

    packet->command_type = CMD_TYPE_SOUND;
    packet->sequence_number = seq_num;
    packet->data_length = sizeof(sound_data_t);

    sound_data->sound_type = sound;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_servo_command(i2c_command_packet_t* packet, servo_command_t servo, uint8_t angle, uint8_t seq_num) {
    if (!packet) return;

    servo_data_t* servo_data = (servo_data_t*)packet->data;

    packet->command_type = CMD_TYPE_SERVO;
    packet->sequence_number = seq_num;
    packet->data_length = sizeof(servo_data_t);

    servo_data->servo_type = servo;
    servo_data->angle = (angle > 180) ? 180 : angle;  // Clamp to valid range

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_display_command(i2c_command_packet_t* packet, uint8_t line, const char* message, uint8_t seq_num) {
    if (!packet || !message) return;

    display_data_t* display_data = (display_data_t*)packet->data;

    packet->command_type = CMD_TYPE_DISPLAY;
    packet->sequence_number = seq_num;
    packet->data_length = sizeof(display_data_t);

    display_data->line = (line > 7) ? 7 : line;  // Clamp to valid range
    strncpy(display_data->message, message, sizeof(display_data->message) - 1);
    display_data->message[sizeof(display_data->message) - 1] = '\0';  // Ensure null termination

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_ping_command(i2c_command_packet_t* packet, uint8_t seq_num) {
    if (!packet) return;

    packet->command_type = CMD_TYPE_PING;
    packet->sequence_number = seq_num;
    packet->data_length = 0;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

// OTA command preparation functions
void prepare_enter_maintenance_command(i2c_command_packet_t* packet, uint8_t seq_num) {
    if (!packet) return;

    packet->command_type = CMD_TYPE_ENTER_MAINTENANCE_MODE;
    packet->sequence_number = seq_num;
    packet->data_length = 0;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_begin_ota_command(i2c_command_packet_t* packet, const char* url, const uint8_t* hash, uint8_t seq_num) {
    if (!packet || !url) return;

    ota_begin_data_t* ota_data = (ota_begin_data_t*)packet->data;

    packet->command_type = CMD_TYPE_BEGIN_OTA;
    packet->sequence_number = seq_num;
    packet->data_length = sizeof(ota_begin_data_t);

    // Copy URL (truncate if necessary)
    size_t url_len = strlen(url);
    if (url_len > 255) {
        /* url_length field is uint8_t; values above 255 would silently truncate */
        ESP_LOGE("i2c_protocol", "OTA URL length %zu exceeds uint8_t max (255), clamping", url_len);
        url_len = 255;
    }
    ota_data->url_length = (uint8_t)url_len;
    strncpy(ota_data->url, url, OTA_URL_MAX_LEN);
    if (url_len < OTA_URL_MAX_LEN) {
        ota_data->url[url_len] = '\0';
    }

    // Copy hash if provided
    if (hash) {
        memcpy(ota_data->hash, hash, OTA_HASH_LEN);
    } else {
        memset(ota_data->hash, 0, OTA_HASH_LEN);
    }

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_get_ota_status_command(i2c_command_packet_t* packet, uint8_t seq_num) {
    if (!packet) return;

    packet->command_type = CMD_TYPE_GET_OTA_STATUS;
    packet->sequence_number = seq_num;
    packet->data_length = 0;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_get_version_command(i2c_command_packet_t* packet, uint8_t seq_num) {
    if (!packet) return;

    packet->command_type = CMD_TYPE_GET_VERSION;
    packet->sequence_number = seq_num;
    packet->data_length = 0;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_reboot_command(i2c_command_packet_t* packet, uint8_t seq_num) {
    if (!packet) return;

    packet->command_type = CMD_TYPE_REBOOT;
    packet->sequence_number = seq_num;
    packet->data_length = 0;

    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}
