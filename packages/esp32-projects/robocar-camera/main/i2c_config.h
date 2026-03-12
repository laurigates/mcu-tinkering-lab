/**
 * @file i2c_config.h
 * @brief Board-specific I2C configuration for ESP32-CAM
 */

#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include "driver/i2c.h"

// I2C Configuration for inter-board communication
// Note: GPIO 33,35,36,39 are input-only on ESP32-CAM, use bidirectional pins
#define I2C_COMM_SCL_IO 14         // SCL pin for ESP32-CAM communication
#define I2C_COMM_SDA_IO 15         // SDA pin for ESP32-CAM communication
#define I2C_MASTER_NUM I2C_NUM_0   // Use I2C_NUM_0 (I2C_NUM_1 used by camera SCCB)
#define I2C_MASTER_FREQ_HZ 100000  // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_SLAVE_ADDRESS 0x42   // Main board slave address
#define I2C_SLAVE_NUM I2C_NUM_0  // Use I2C_NUM_0 (I2C_NUM_1 used by camera SCCB)
#define I2C_SLAVE_TX_BUF_LEN 128
#define I2C_SLAVE_RX_BUF_LEN 128

#endif  // I2C_CONFIG_H
