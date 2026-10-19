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

size_t gpio_expander_probe_order(uint8_t preferred, uint8_t out[GPIO_EXPANDER_ADDR_COUNT])
{
    size_t n = 0;
    bool in_range = preferred >= GPIO_EXPANDER_ADDR_BASE &&
                    preferred < GPIO_EXPANDER_ADDR_BASE + GPIO_EXPANDER_ADDR_COUNT;
    if (in_range)
        out[n++] = preferred;
    for (uint8_t i = 0; i < GPIO_EXPANDER_ADDR_COUNT; i++) {
        uint8_t addr = (uint8_t)(GPIO_EXPANDER_ADDR_BASE + i);
        if (!in_range || addr != preferred)
            out[n++] = addr;
    }
    return n;
}
