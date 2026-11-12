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
 *
 * And the address probe order (issue #781): the MCP23017's A0-A2 set its
 * address in 0x20-0x27 (DS20001952D p. 11), and a CJMCU-2317 on a harness
 * may pull them high (0x27). The configured address is tried first, then the
 * rest of the range in ascending order, each address once.
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

static void test_probe_order_preferred_first(void)
{
    uint8_t order[GPIO_EXPANDER_ADDR_COUNT];
    size_t n = gpio_expander_probe_order(0x20, order);
    CHECK(n == 8, "count %u", (unsigned)n);
    for (unsigned i = 0; i < 8; i++)
        CHECK(order[i] == 0x20 + i, "0x20 first: order[%u] = 0x%02X", i, order[i]);

    n = gpio_expander_probe_order(0x27, order);
    CHECK(n == 8, "count %u", (unsigned)n);
    const uint8_t want[8] = {0x27, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26};
    for (unsigned i = 0; i < 8; i++)
        CHECK(order[i] == want[i], "0x27 first: order[%u] = 0x%02X, want 0x%02X", i, order[i],
              want[i]);
}

static void test_probe_order_covers_range_once(void)
{
    for (uint8_t pref = 0x20; pref <= 0x27; pref++) {
        uint8_t order[GPIO_EXPANDER_ADDR_COUNT];
        size_t n = gpio_expander_probe_order(pref, order);
        unsigned seen = 0;
        for (size_t i = 0; i < n; i++) {
            CHECK(order[i] >= 0x20 && order[i] <= 0x27, "pref 0x%02X: 0x%02X out of range", pref,
                  order[i]);
            seen |= 1u << (order[i] - 0x20);
        }
        CHECK(n == 8 && seen == 0xFFu, "pref 0x%02X: n %u, seen 0x%02X", pref, (unsigned)n, seen);
        CHECK(order[0] == pref, "pref 0x%02X not first", pref);
    }
}

static void test_probe_order_ignores_out_of_range_preference(void)
{
    uint8_t order[GPIO_EXPANDER_ADDR_COUNT];
    size_t n = gpio_expander_probe_order(0x40, order);
    CHECK(n == 8, "count %u", (unsigned)n);
    for (unsigned i = 0; i < 8; i++)
        CHECK(order[i] == 0x20 + i, "0x40: order[%u] = 0x%02X", i, order[i]);
}

int main(void)
{
    test_output_only_pins_reject_inputs();
    test_other_pins_accept_all_modes();
    test_out_of_range_pins_rejected();
    test_unknown_mode_rejected();
    test_output_only_mask();
    test_probe_order_preferred_first();
    test_probe_order_covers_range_once();
    test_probe_order_ignores_out_of_range_preference();

    if (s_failures) {
        fprintf(stderr, "test_gpio_expander_core: %d failure(s)\n", s_failures);
        return 1;
    }
    printf("test_gpio_expander_core: all passed\n");
    return 0;
}
