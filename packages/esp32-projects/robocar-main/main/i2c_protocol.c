/**
 * @file i2c_protocol.c
 * @brief I2C communication protocol implementation
 */

#include "i2c_protocol.h"
#include <string.h>

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

void prepare_movement_command(i2c_command_packet_t* packet, movement_command_t movement, uint8_t speed) {
    if (!packet) return;
    
    movement_data_t* move_data = (movement_data_t*)packet->data;
    
    packet->command_type = CMD_TYPE_MOVEMENT;
    packet->data_length = sizeof(movement_data_t);
    
    move_data->movement = movement;
    move_data->speed = speed;
    
    // Calculate checksum for entire packet except checksum field
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_sound_command(i2c_command_packet_t* packet, sound_command_t sound) {
    if (!packet) return;
    
    sound_data_t* sound_data = (sound_data_t*)packet->data;
    
    packet->command_type = CMD_TYPE_SOUND;
    packet->data_length = sizeof(sound_data_t);
    
    sound_data->sound_type = sound;
    
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_servo_command(i2c_command_packet_t* packet, servo_command_t servo, uint8_t angle) {
    if (!packet) return;
    
    servo_data_t* servo_data = (servo_data_t*)packet->data;
    
    packet->command_type = CMD_TYPE_SERVO;
    packet->data_length = sizeof(servo_data_t);
    
    servo_data->servo_type = servo;
    servo_data->angle = (angle > 180) ? 180 : angle;  // Clamp to valid range
    
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_display_command(i2c_command_packet_t* packet, uint8_t line, const char* message) {
    if (!packet || !message) return;
    
    display_data_t* display_data = (display_data_t*)packet->data;
    
    packet->command_type = CMD_TYPE_DISPLAY;
    packet->data_length = sizeof(display_data_t);
    
    display_data->line = (line > 7) ? 7 : line;  // Clamp to valid range
    strncpy(display_data->message, message, sizeof(display_data->message) - 1);
    display_data->message[sizeof(display_data->message) - 1] = '\0';  // Ensure null termination
    
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}

void prepare_ping_command(i2c_command_packet_t* packet) {
    if (!packet) return;
    
    packet->command_type = CMD_TYPE_PING;
    packet->data_length = 0;
    
    packet->checksum = calculate_checksum((uint8_t*)packet, sizeof(i2c_command_packet_t) - 1);
}