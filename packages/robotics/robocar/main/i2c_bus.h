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
 * Consecutive failed channel-selects before the TCA9548A is reset.
 *
 * A device that locks up its segment holding SDA low (Adafruit #4742 says the
 * RCWL-1601 can) makes every following mux write fail, whichever task issues
 * it. One failure is noise; several in a row with no success between them is a
 * stuck bus. At the executor's 30 Hz plus the motor and LED writers, five is
 * well under a second of lost traffic.
 */
#define I2C_BUS_MUX_RESET_AFTER_FAILURES 5u

/**
 * How long TCA9548A_RST_PIN is held low, in µs.
 *
 * TI's minimum is t_W(L) = 6 ns (TCA9548A datasheet SCPS207H §5.7, "Pulse
 * duration, RESET low"; t_REC(STA) = 0 ns, so the bus may be used at once
 * after). 10 µs is three orders of margin, enough that the breakout's RESET
 * pull-up and the wiring capacitance cannot round the pulse away.
 */
#define TCA9548A_RST_PULSE_US 10u

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
    uint32_t mux_resets;     /**< TCA9548A RESET pulses (i2c_bus_reset_mux()). */
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
 * @brief The PCA9685's current PWM frequency, in Hz.
 *
 * The prescaler is chip-wide — one frequency for motors, servos and LEDs — so
 * this is a property of the board, not of a channel. The servo pulse maths
 * needs it: a pulse width only means something relative to the period it sits
 * in, and a period baked in as a constant silently stops being true the moment
 * the frequency is changed.
 */
uint16_t i2c_bus_pca9685_frequency(void);

/**
 * @brief Set the PCA9685's PWM frequency (24-1526 Hz).
 *
 * Affects every channel at once, motors and LEDs included. Intended for bench
 * bring-up: the shipped 200 Hz is a compromise chosen for motor smoothness, and
 * whether an analog servo will track pulses at that frame rate is a question
 * about the specific servos fitted. Does not persist to NVS.
 */
esp_err_t i2c_bus_pca9685_set_frequency(uint16_t hz);

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
 * @brief Pulse the TCA9548A's active-low RESET (TCA9548A_RST_PIN).
 *
 * The mux resets its state machine and deselects every channel, which
 * disconnects a downstream segment stuck low from the upstream bus (TCA9548A
 * datasheet §7.4.1). The next i2c_bus_select_channel() re-selects as usual.
 *
 * Called by i2c_bus_select_channel() after I2C_BUS_MUX_RESET_AFTER_FAILURES
 * consecutive failed selections; callable directly too. Takes the bus mutex, so
 * it never cuts a transaction short.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE before i2c_bus_init(), or
 *         ESP_ERR_TIMEOUT if the bus mutex could not be taken.
 */
esp_err_t i2c_bus_reset_mux(void);

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
