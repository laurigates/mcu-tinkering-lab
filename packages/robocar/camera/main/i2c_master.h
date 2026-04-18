/**
 * @file i2c_master.h
 * @brief I2C master implementation for ESP32-CAM
 */

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include "esp_err.h"
#include "i2c_protocol.h"

/**
 * @brief Initialize I2C master
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Deinitialize I2C master
 */
void i2c_master_deinit(void);

/**
 * @brief Send command packet to slave and optionally read response
 * @param command Command packet to send
 * @param response Response buffer (can be NULL if no response expected)
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_master_send_command(const i2c_command_packet_t *command,
                                  i2c_response_packet_t *response, uint32_t timeout_ms);

/**
 * @brief Send movement command
 * @param movement Movement type
 * @param speed Movement speed (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_movement_command(movement_command_t movement, uint8_t speed);

/**
 * @brief Send sound command
 * @param sound Sound type
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_sound_command(sound_command_t sound);

/**
 * @brief Send servo command
 * @param servo Servo type
 * @param angle Servo angle (0-180)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_servo_command(servo_command_t servo, uint8_t angle);

/**
 * @brief Send display message command
 * @param line Display line (0-7)
 * @param message Message text
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_display_command(uint8_t line, const char *message);

/**
 * @brief Ping the slave device
 * @return ESP_OK if slave responds, error code otherwise
 */
esp_err_t i2c_ping_slave(void);

/**
 * @brief Get status from slave device
 * @param status Status data structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_get_status(status_data_t *status);

/**
 * @brief Send command to enter maintenance mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_enter_maintenance_mode(void);

/**
 * @brief Send command to begin OTA update
 * @param url OTA firmware URL
 * @param hash Hash prefix for verification (can be NULL)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_begin_ota(const char *url, const uint8_t *hash);

/**
 * @brief Get OTA status from slave device
 * @param ota_status OTA status structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_get_ota_status(ota_status_response_t *ota_status);

/**
 * @brief Get firmware version from slave device
 * @param version Buffer to store version string
 * @param version_len Size of version buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_get_version(char *version, size_t version_len);

/**
 * @brief Send reboot command to slave device
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_send_reboot(void);

#endif  // I2C_MASTER_H
