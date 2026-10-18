/**
 * @file gpio_expander_core.c
 * @brief MCP23017 pin/mode validation, with no ESP-IDF dependency.
 */

#include "gpio_expander_core.h"

bool gpio_expander_pin_output_only(uint8_t pin)
{
    if (pin >= GPIO_EXPANDER_PIN_COUNT)
        return false;
    return (GPIO_EXPANDER_OUTPUT_ONLY_MASK & (1u << pin)) != 0;
}

gpio_expander_check_t gpio_expander_check_mode(uint8_t pin, gpio_expander_mode_t mode)
{
    if (pin >= GPIO_EXPANDER_PIN_COUNT)
        return GPIO_EXPANDER_CHECK_BAD_PIN;

    switch (mode) {
        case GPIO_EXPANDER_OUTPUT:
            return GPIO_EXPANDER_CHECK_OK;
        case GPIO_EXPANDER_INPUT:
        case GPIO_EXPANDER_INPUT_PULLUP:
            return gpio_expander_pin_output_only(pin) ? GPIO_EXPANDER_CHECK_OUTPUT_ONLY
                                                      : GPIO_EXPANDER_CHECK_OK;
        default:
            return GPIO_EXPANDER_CHECK_BAD_MODE;
    }
}
