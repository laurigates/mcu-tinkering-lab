/**
 * @file i2c_slave.h
 * @brief I2C slave implementation for main board
 */

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "esp_err.h"
#include "i2c_protocol.h"

/**
 * @brief Initialize I2C slave
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_slave_init(void);

/**
 * @brief Deinitialize I2C slave
 */
void i2c_slave_deinit(void);

/**
 * @brief Start I2C slave task for handling commands
 */
void i2c_slave_start_task(void);

/**
 * @brief Stop I2C slave task
 */
void i2c_slave_stop_task(void);

/**
 * @brief Set current status data for responses
 * @param status Status data to set
 */
void i2c_slave_set_status(const status_data_t *status);

/**
 * @brief Get current status data
 * @param status Buffer to fill with current status
 */
void i2c_slave_get_status(status_data_t *status);

/**
 * @brief Check if I2C slave is ready
 * @return true if ready, false otherwise
 */
bool i2c_slave_is_ready(void);

#endif  // I2C_SLAVE_H
