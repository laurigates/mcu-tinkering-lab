/**
 * @file serial_comm.h
 * @brief Serial communication with main controller
 */

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include "esp_err.h"

/**
 * @brief Initialize serial communication
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t serial_init(void);

/**
 * @brief Send command to main controller
 * @param command Command string to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t serial_send_command(const char* command);

/**
 * @brief Send display message to main controller OLED
 * @param line Display line (0-7)
 * @param message Message to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t serial_send_display_message(int line, const char* message);

/**
 * @brief Read response from main controller (if any)
 * @param buffer Buffer to store response
 * @param buffer_size Size of buffer
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes read, or -1 on error
 */
int serial_read_response(char* buffer, size_t buffer_size, int timeout_ms);

/**
 * @brief Deinitialize serial communication
 */
void serial_deinit(void);

#endif // SERIAL_COMM_H