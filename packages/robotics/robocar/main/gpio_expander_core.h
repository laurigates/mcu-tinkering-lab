/**
 * @file gpio_expander_core.h
 * @brief MCP23017 pin/mode validation, with no ESP-IDF dependency.
 *
 * Hardware-agnostic: no ESP-IDF header, so the identical check runs in the
 * firmware and in the host tests (the shared-core pattern in
 * .claude/rules/testing.md). gpio_expander.c maps the result to an esp_err_t.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pin direction
 */
typedef enum {
    GPIO_EXPANDER_OUTPUT = 0,
    GPIO_EXPANDER_INPUT,         //!< High-impedance input
    GPIO_EXPANDER_INPUT_PULLUP,  //!< Input with internal 100k pullup
} gpio_expander_mode_t;

/** Pins 0-7 = PORTA (GPA0-GPA7), 8-15 = PORTB (GPB0-GPB7). */
#define GPIO_EXPANDER_PIN_COUNT 16

/** GPA7 (pin 7) and GPB7 (pin 15) are output-only on the MCP23017 Rev D
 *  (DS20001952D p. 1, p. 11, § 3.5.1 note), so input and pull-up modes are
 *  refused on them. */
#define GPIO_EXPANDER_OUTPUT_ONLY_MASK 0x8080u

/** A0-A2 set the address within 0x20-0x27 (DS20001952D p. 11). */
#define GPIO_EXPANDER_ADDR_BASE 0x20u
#define GPIO_EXPANDER_ADDR_COUNT 8u

/**
 * @brief Outcome of validating a pin/mode pair
 */
typedef enum {
    GPIO_EXPANDER_CHECK_OK = 0,
    GPIO_EXPANDER_CHECK_BAD_PIN,      //!< Pin number past 15
    GPIO_EXPANDER_CHECK_BAD_MODE,     //!< Mode outside gpio_expander_mode_t
    GPIO_EXPANDER_CHECK_OUTPUT_ONLY,  //!< Input or pull-up asked of GPA7/GPB7
} gpio_expander_check_t;

/**
 * @brief Whether a pin is one of the output-only pins (7 = GPA7, 15 = GPB7)
 *
 * @param pin Pin number; anything past 15 returns false
 */
bool gpio_expander_pin_output_only(uint8_t pin);

/**
 * @brief Validate a pin/mode pair before it reaches the chip
 *
 * @param pin Pin number (0-15)
 * @param mode Requested direction
 * @return GPIO_EXPANDER_CHECK_OK, or why the pair is refused
 */
gpio_expander_check_t gpio_expander_check_mode(uint8_t pin, gpio_expander_mode_t mode);

/**
 * @brief The addresses to probe for the MCP23017, in order
 *
 * The carriers tie A0-A2 to GND (0x20); a CJMCU-2317 module on a harness
 * biases them through its own resistor array, possibly high (0x27, #781).
 * `preferred` (MCP23017_ADDR) comes first when it is in 0x20-0x27, then the
 * rest of the range in ascending order, each address once.
 *
 * @param preferred The configured address
 * @param[out] out Receives GPIO_EXPANDER_ADDR_COUNT addresses
 * @return Number of addresses written (GPIO_EXPANDER_ADDR_COUNT)
 */
size_t gpio_expander_probe_order(uint8_t preferred, uint8_t out[GPIO_EXPANDER_ADDR_COUNT]);

#ifdef __cplusplus
}
#endif
