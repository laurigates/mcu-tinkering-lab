/**
 * @file gpio_expander.h
 * @brief MCP23017 16-bit GPIO expander via TCA9548A channel 2
 *
 * Generic GPIO expansion — no pins are pre-assigned to a role. Direction
 * is configurable per pin at runtime, except GPA7 (pin 7) and GPB7 (pin 15),
 * which are output-only on the MCP23017 Rev D (DS20001952D p. 1, p. 11,
 * § 3.5.1 note): init makes them outputs driven low, and input or pull-up
 * modes on them are refused with ESP_ERR_NOT_SUPPORTED. The device is
 * optional: if no
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

#include "gpio_expander_core.h"  // gpio_expander_mode_t, output-only pin check

/**
 * @brief Initialize the MCP23017 GPIO expander
 *
 * Probes the device on its TCA9548A channel: MCP23017_ADDR first, then the
 * rest of 0x20-0x27, keeping the first that answers and warning when that is
 * not MCP23017_ADDR (a module whose A0-A2 are not tied low, #781). Pins 0-6
 * and 8-14 stay inputs
 * (the chip's power-on state); the output-only pins 7 and 15 are set to
 * outputs driven low. A missing device is not an error — the module logs a
 * warning and reports unavailable.
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
 * @brief The address the MCP23017 answered at, 0x20-0x27; 0 when unavailable
 */
uint8_t gpio_expander_address(void);

/**
 * @brief Configure a pin's direction
 *
 * @param pin Pin number (0-15)
 * @param mode Direction (output, input, input with pullup)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED for input or pull-up on
 *         the output-only pins 7 (GPA7) and 15 (GPB7), ESP_ERR_INVALID_ARG
 *         for a pin past 15 or an unknown mode
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
