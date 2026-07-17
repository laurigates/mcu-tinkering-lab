/**
 * @file gpio_expander.h
 * @brief MCP23017 16-bit GPIO expander via TCA9548A channel 2
 *
 * Generic GPIO expansion — no pins are pre-assigned to a role. Direction
 * is configurable per pin at runtime. The device is optional: if no
 * MCP23017 responds at init, the module marks itself unavailable and all
 * pin operations return ESP_ERR_INVALID_STATE without touching the bus.
 *
 * Pin numbering follows the mcp23x17 driver: 0-7 = PORTA (A0-A7),
 * 8-15 = PORTB (B0-B7).
 */

#ifndef GPIO_EXPANDER_H
#define GPIO_EXPANDER_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Pin direction
 */
typedef enum {
    GPIO_EXPANDER_OUTPUT = 0,
    GPIO_EXPANDER_INPUT,         //!< High-impedance input
    GPIO_EXPANDER_INPUT_PULLUP,  //!< Input with internal 100k pullup
} gpio_expander_mode_t;

/**
 * @brief Initialize the MCP23017 GPIO expander
 *
 * Probes the device on its TCA9548A channel. All pins default to inputs
 * (the chip's power-on state). A missing device is not an error — the
 * module logs a warning and reports unavailable.
 *
 * Requires i2c_bus_init() to have run first.
 *
 * @return ESP_OK on success or graceful absence; error only on bus faults
 */
esp_err_t gpio_expander_init(void);

/**
 * @brief Whether an MCP23017 was detected at init
 */
bool gpio_expander_available(void);

/**
 * @brief Configure a pin's direction
 *
 * @param pin Pin number (0-15)
 * @param mode Direction (output, input, input with pullup)
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_set_mode(uint8_t pin, gpio_expander_mode_t mode);

/**
 * @brief Set an output pin's level
 *
 * Pin must be configured as output first.
 *
 * @param pin Pin number (0-15)
 * @param level true = high, false = low
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_write(uint8_t pin, bool level);

/**
 * @brief Read a pin's current level
 *
 * @param pin Pin number (0-15)
 * @param[out] level true = high, false = low
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_read(uint8_t pin, bool *level);

/**
 * @brief Read all 16 pins in one transaction
 *
 * @param[out] value Bit 0 = PORTA pin 0 ... bit 15 = PORTB pin 7
 * @return ESP_OK on success
 */
esp_err_t gpio_expander_read_port(uint16_t *value);

#endif  // GPIO_EXPANDER_H
