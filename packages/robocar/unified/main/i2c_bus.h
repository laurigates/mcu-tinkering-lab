/**
 * @file i2c_bus.h
 * @brief TCA9548A-based I2C bus multiplexer with thread-safe access
 *
 * Provides atomic channel-select-then-operate semantics for all I2C
 * peripherals (PCA9685, SSD1306) connected through the TCA9548A.
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <esp_err.h>
#include <i2cdev.h>
#include <pca9685.h>
#include <stdbool.h>
#include <stdint.h>
#include "pin_config.h"

/**
 * @brief Initialize the I2C bus, TCA9548A multiplexer, and PCA9685
 *
 * Sets up I2C master on the configured SDA/SCL pins, initializes the
 * TCA9548A at its default address, and configures the PCA9685 for
 * 200Hz PWM output.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Whether the I2C bus (TCA9548A + PCA9685) initialised successfully
 *
 * False on a bare board where no I2C hardware responded — the bus degrades
 * gracefully (see init_hardware) and the PCA9685-backed peripherals fail safe.
 *
 * @return true if i2c_bus_init() completed
 */
bool i2c_bus_is_ready(void);

/**
 * @brief Bus activity since boot or the last i2c_bus_stats_reset().
 *
 * One "op" is one bus acquisition through i2c_bus_select_channel(): a mux
 * channel-select write plus whatever device traffic the caller then issues. It
 * is a lower bound on wire transactions, never an overcount.
 */
typedef struct {
    uint32_t ops;            /**< Successful bus acquisitions. */
    uint32_t failed;         /**< Mux-select failures and mutex timeouts. */
    uint32_t per_channel[8]; /**< Ops per TCA9548A channel. */
    uint32_t hz;             /**< Recent rate; 0 when the bus has gone quiet. */
    uint32_t peak_hz;        /**< Highest windowed rate seen. */
    uint32_t elapsed_ms;     /**< Measurement span. */
} i2c_bus_stats_t;

/**
 * @brief Read the bus activity counters.
 *
 * `hz` is deliberately zeroed once its window has gone stale. A rate whose
 * window has not been touched for two window lengths is not a rate, it is a
 * memory of one, and reporting the stale value would show live traffic on a bus
 * that has gone silent — the exact distinction this counter exists to make.
 */
void i2c_bus_stats_get(i2c_bus_stats_t *out);

/** @brief Zero the bus activity counters and restart the measurement span. */
void i2c_bus_stats_reset(void);

/** @brief Print the bus activity line for the `trace` console command. */
void i2c_bus_stats_report(void);

/**
 * @brief Select a TCA9548A channel and acquire the bus mutex
 *
 * Must be paired with i2c_bus_release(). While the mutex is held,
 * no other task can change the TCA9548A channel.
 *
 * @param channel TCA9548A channel number (0-7)
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_select_channel(uint8_t channel);

/**
 * @brief Release the bus mutex after an I2C operation
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_release(void);

/**
 * @brief Get the PCA9685 device handle
 *
 * Only valid after i2c_bus_init(). Caller must hold the bus mutex
 * (via i2c_bus_select_channel) before using this handle.
 *
 * @return Pointer to the PCA9685 device descriptor
 */
i2c_dev_t *i2c_bus_get_pca9685(void);

/**
 * @brief Set a PCA9685 channel value (thread-safe convenience wrapper)
 *
 * Acquires the bus mutex, selects the PCA9685 channel on the TCA9548A,
 * sets the PWM value, and releases the mutex.
 *
 * @param channel PCA9685 channel (0-15)
 * @param value PWM value (0-4095) or 4096 for full-on
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t value);

/**
 * @brief Set multiple PCA9685 channel values (thread-safe convenience wrapper)
 *
 * Acquires the bus mutex, selects the PCA9685 channel on the TCA9548A,
 * sets all PWM values in a single I2C transaction, and releases the mutex.
 *
 * @param first_ch First PCA9685 channel (0-15)
 * @param count Number of channels to set
 * @param values Array of PWM values
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values);

/**
 * @brief Deinitialize the I2C bus and free resources
 */
void i2c_bus_deinit(void);

#endif  // I2C_BUS_H
