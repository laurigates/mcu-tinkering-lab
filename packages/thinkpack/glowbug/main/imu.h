/**
 * @file imu.h
 * @brief MPU6050 inertial measurement unit driver (inline I2C, no managed component).
 *
 * Uses ESP-IDF v5.4 new I2C master API (i2c_master.h). Provides shake
 * detection and tilt-to-hue mapping for the glowbug animation system.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** I2C SDA pin on ESP32-S3 SuperMini. */
#define IMU_SDA_GPIO 6

/** I2C SCL pin on ESP32-S3 SuperMini. */
#define IMU_SCL_GPIO 7

/** MPU6050 I2C address (AD0 pin = LOW). */
#define IMU_I2C_ADDR 0x68

/** I2C bus clock frequency (Hz). */
#define IMU_I2C_FREQ_HZ 100000

/**
 * @brief Initialise the I2C master bus and wake the MPU6050.
 *
 * Configures I2C master on IMU_SDA_GPIO / IMU_SCL_GPIO, writes 0x00 to
 * PWR_MGMT_1 (reg 0x6B) to wake the device from sleep, and leaves the
 * accelerometer at its default ±2 g full-scale range.
 *
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t imu_init(void);

/**
 * @brief Read the three accelerometer axes.
 *
 * Reads 6 bytes starting at register 0x3B (ACCEL_XOUT_H) and converts
 * to g values (divide by 16384 for ±2 g range).
 *
 * @param ax  Output: X-axis acceleration in g.
 * @param ay  Output: Y-axis acceleration in g.
 * @param az  Output: Z-axis acceleration in g.
 * @return ESP_OK on success.
 */
esp_err_t imu_read_accel(float *ax, float *ay, float *az);

/**
 * @brief Detect a shake gesture.
 *
 * Computes the Euclidean magnitude of the acceleration vector delta
 * since the last call. Returns true if the delta exceeds 1.5 g.
 * Suitable for coarse shake detection without FFT.
 *
 * @return true if a shake was detected, false otherwise.
 */
bool imu_detect_shake(void);

/**
 * @brief Map the current tilt angle to an HSV hue byte.
 *
 * Computes roll = atan2f(ay, az), maps the range −π..π to hue 0..255.
 *
 * @return Hue byte (0-255).
 */
uint8_t imu_tilt_to_hue(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
