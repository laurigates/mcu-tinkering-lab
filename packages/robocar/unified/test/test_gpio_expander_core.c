/**
 * @file test_gpio_expander_core.c
 * @brief Pins the MCP23017 pin/mode validation in gpio_expander_core.c.
 *
 * On the MCP23017 Rev D, GPA7 (pin 7) and GPB7 (pin 15) are output-only
 * (DS20001952D p. 1, p. 11, § 3.5.1 note). What is pinned:
 *
 *  - Pins 7 and 15 reject input and input-with-pullup, accept output.
 *  - Every other pin accepts all three modes.
 *  - Pins past 15 are rejected for every mode.
 *  - A mode value outside the enum is rejected rather than treated as input.
 *  - The output-only mask names exactly pins 7 and 15.
 */

#include "gpio_expander_core.h"

#include <stdio.h>

static int s_failures;

#define CHECK(cond, ...)                                           \
    do {                                                           \
        if (!(cond)) {                                             \
            s_failures++;                                          \
            fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                          \
            fputc('\n', stderr);                                   \
        }                                                          \
    } while (0)

static void test_output_only_pins_reject_inputs(void)
{
    const uint8_t pins[] = {7, 15};
    for (unsigned i = 0; i < sizeof(pins); i++) {
        uint8_t p = pins[i];
        CHECK(gpio_expander_pin_output_only(p), "pin %u should be output-only", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT) == GPIO_EXPANDER_CHECK_OUTPUT_ONLY,
              "pin %u in", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT_PULLUP) ==
                  GPIO_EXPANDER_CHECK_OUTPUT_ONLY,
              "pin %u up", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_OUTPUT) == GPIO_EXPANDER_CHECK_OK,
              "pin %u out", p);
    }
}

static void test_other_pins_accept_all_modes(void)
{
    for (uint8_t p = 0; p < GPIO_EXPANDER_PIN_COUNT; p++) {
        if (p == 7 || p == 15)
            continue;
        CHECK(!gpio_expander_pin_output_only(p), "pin %u should not be output-only", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT) == GPIO_EXPANDER_CHECK_OK,
              "pin %u in", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT_PULLUP) == GPIO_EXPANDER_CHECK_OK,
              "pin %u up", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_OUTPUT) == GPIO_EXPANDER_CHECK_OK,
              "pin %u out", p);
    }
}

static void test_out_of_range_pins_rejected(void)
{
    const uint8_t pins[] = {16, 23, 255};
    for (unsigned i = 0; i < sizeof(pins); i++) {
        uint8_t p = pins[i];
        CHECK(!gpio_expander_pin_output_only(p), "pin %u is not a pin", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_OUTPUT) == GPIO_EXPANDER_CHECK_BAD_PIN,
              "pin %u out", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT) == GPIO_EXPANDER_CHECK_BAD_PIN,
              "pin %u in", p);
        CHECK(gpio_expander_check_mode(p, GPIO_EXPANDER_INPUT_PULLUP) ==
                  GPIO_EXPANDER_CHECK_BAD_PIN,
              "pin %u up", p);
    }
}

static void test_unknown_mode_rejected(void)
{
    CHECK(gpio_expander_check_mode(0, (gpio_expander_mode_t)42) == GPIO_EXPANDER_CHECK_BAD_MODE,
          "unknown mode on pin 0");
    CHECK(gpio_expander_check_mode(7, (gpio_expander_mode_t)42) == GPIO_EXPANDER_CHECK_BAD_MODE,
          "unknown mode on pin 7");
}

static void test_output_only_mask(void)
{
    CHECK(GPIO_EXPANDER_OUTPUT_ONLY_MASK == 0x8080u, "mask = 0x%04X",
          (unsigned)GPIO_EXPANDER_OUTPUT_ONLY_MASK);
}

int main(void)
{
    test_output_only_pins_reject_inputs();
    test_other_pins_accept_all_modes();
    test_out_of_range_pins_rejected();
    test_unknown_mode_rejected();
    test_output_only_mask();

    if (s_failures) {
        fprintf(stderr, "test_gpio_expander_core: %d failure(s)\n", s_failures);
        return 1;
    }
    printf("test_gpio_expander_core: all passed\n");
    return 0;
}
