/**
 * @file i2c_config.h
 * @brief Board-specific I2C configuration for main controller (Heltec WiFi LoRa 32 V1)
 */

#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include "driver/i2c.h"

// I2C Configuration for inter-board communication
//
// Bus assignment on this board (ESP32 has two I2C controllers):
//   I2C_NUM_0 — PCA9685 master (pins 21/22), managed dynamically by i2cdev.
//   I2C_NUM_1 — Camera-link slave (pins 26/27), this file.
//
// The integrated Heltec OLED also wants I2C_NUM_1 (master, pins 4/15) but
// is mutually exclusive with the slave — same controller cannot be both
// master and slave, and the pin sets differ. The slave wins because it is
// the inter-controller link without which camera commands never reach the
// main board; init_oled() in main.c skips OLED setup when the slave has
// already claimed the controller.
#define I2C_COMM_SCL_IO 26         // SCL pin for ESP32-CAM communication (bidirectional)
#define I2C_COMM_SDA_IO 27         // SDA pin for ESP32-CAM communication (bidirectional)
#define I2C_MASTER_NUM I2C_NUM_1   // Camera-link controller (see header note)
#define I2C_MASTER_FREQ_HZ 100000  // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_SLAVE_ADDRESS 0x42   // Main board slave address
#define I2C_SLAVE_NUM I2C_NUM_1  // Camera-link controller (see header note)
#define I2C_SLAVE_TX_BUF_LEN 128
#define I2C_SLAVE_RX_BUF_LEN 128

#endif  // I2C_CONFIG_H
