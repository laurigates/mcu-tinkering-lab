/**
 * @file test_led_controller.c
 *
 * Pins which PCA9685 channel each colour component lands on.
 *
 * led_set_hardware() writes one LED as a run of three consecutive channels in
 * a single i2c_bus_pca9685_set_multi() transaction. The value array used to be
 * positional — {red, green, blue} — which silently encoded an assumption about
 * the order of LED_*_R/G/B_CHANNEL in pin_config.h that nothing verified. The
 * failure mode is not a crash: reorder those #defines and the module keeps
 * compiling, keeps writing three consecutive channels, and lights the wrong
 * colour. On a status indicator that means a red "capture failed" hold showing
 * up as blue, which reads as a firmware bug in whatever the LED was reporting
 * on rather than in the LED driver.
 *
 * The bench cannot stage the interesting case either, because verifying it
 * there means wiring an LED and trusting your eyes about which leg is which.
 *
 * led_controller.c is compiled unmodified; the i2c_bus it writes through and
 * the FreeRTOS timer it creates are the stubs below and in test/include.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_bus.h"
#include "led_controller.h"
#include "pin_config.h"

/* ---- recording i2c_bus stub --------------------------------------------- */

#define MAX_WRITES 8

static struct {
    uint8_t first_ch;
    uint8_t count;
    uint16_t values[4];
} s_writes[MAX_WRITES];
static int s_write_count;
static esp_err_t s_next_result = ESP_OK;

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    if (s_write_count < MAX_WRITES) {
        s_writes[s_write_count].first_ch = first_ch;
        s_writes[s_write_count].count = count;
        for (uint8_t i = 0; i < count && i < 4; i++)
            s_writes[s_write_count].values[i] = values[i];
    }
    s_write_count++;
    return s_next_result;
}

esp_err_t i2c_bus_pca9685_set_pwm(uint8_t channel, uint16_t value)
{
    (void)channel;
    (void)value;
    return s_next_result;
}

/* ---- harness ------------------------------------------------------------ */

static int s_failures;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
            s_failures++;                                 \
        }                                                 \
    } while (0)

static void reset_bus(void)
{
    s_write_count = 0;
    s_next_result = ESP_OK;
}

/** Mirrors color_to_pwm() in the module under test. Duplicated rather than
 *  exposed, so a change to the scaling shows up here as a failure to explain
 *  instead of being silently tracked. */
static uint16_t expect_pwm(uint8_t component)
{
    return (uint16_t)((uint32_t)component * PCA9685_PWM_MAX / 255);
}

#define SLOT(ch, first) ((ch) - (first))

/* Three components that cannot be confused with one another, and none of them
 * 0 or 255 — a colour with a repeated component would pass even with two slots
 * swapped, which is exactly the bug being pinned. */
static const rgb_color_t k_probe = {.red = 30, .green = 120, .blue = 210};

static void test_left_components_land_on_their_own_channels(void)
{
    reset_bus();
    CHECK(led_controller_init() == ESP_OK, "init should succeed");

    reset_bus();
    CHECK(led_set_left(&k_probe) == ESP_OK, "setting the left LED should succeed");
    CHECK(s_write_count == 1, "one LED is one transaction, got %d", s_write_count);
    CHECK(s_writes[0].first_ch == LED_LEFT_R_CHANNEL && s_writes[0].count == 3,
          "left LED should write 3 channels from %d, got %d from %d", LED_LEFT_R_CHANNEL,
          s_writes[0].count, s_writes[0].first_ch);

    const uint16_t *v = s_writes[0].values;
    CHECK(v[SLOT(LED_LEFT_R_CHANNEL, LED_LEFT_R_CHANNEL)] == expect_pwm(k_probe.red),
          "red belongs on ch%d: expected %u, got %u", LED_LEFT_R_CHANNEL, expect_pwm(k_probe.red),
          v[SLOT(LED_LEFT_R_CHANNEL, LED_LEFT_R_CHANNEL)]);
    CHECK(v[SLOT(LED_LEFT_G_CHANNEL, LED_LEFT_R_CHANNEL)] == expect_pwm(k_probe.green),
          "green belongs on ch%d: expected %u, got %u", LED_LEFT_G_CHANNEL,
          expect_pwm(k_probe.green), v[SLOT(LED_LEFT_G_CHANNEL, LED_LEFT_R_CHANNEL)]);
    CHECK(v[SLOT(LED_LEFT_B_CHANNEL, LED_LEFT_R_CHANNEL)] == expect_pwm(k_probe.blue),
          "blue belongs on ch%d: expected %u, got %u", LED_LEFT_B_CHANNEL, expect_pwm(k_probe.blue),
          v[SLOT(LED_LEFT_B_CHANNEL, LED_LEFT_R_CHANNEL)]);
}

static void test_right_components_land_on_their_own_channels(void)
{
    reset_bus();
    CHECK(led_set_right(&k_probe) == ESP_OK, "setting the right LED should succeed");
    CHECK(s_write_count == 1, "one LED is one transaction, got %d", s_write_count);
    CHECK(s_writes[0].first_ch == LED_RIGHT_R_CHANNEL, "right LED should write from %d, got %d",
          LED_RIGHT_R_CHANNEL, s_writes[0].first_ch);

    const uint16_t *v = s_writes[0].values;
    CHECK(v[SLOT(LED_RIGHT_R_CHANNEL, LED_RIGHT_R_CHANNEL)] == expect_pwm(k_probe.red),
          "red belongs on ch%d", LED_RIGHT_R_CHANNEL);
    CHECK(v[SLOT(LED_RIGHT_G_CHANNEL, LED_RIGHT_R_CHANNEL)] == expect_pwm(k_probe.green),
          "green belongs on ch%d", LED_RIGHT_G_CHANNEL);
    CHECK(v[SLOT(LED_RIGHT_B_CHANNEL, LED_RIGHT_R_CHANNEL)] == expect_pwm(k_probe.blue),
          "blue belongs on ch%d", LED_RIGHT_B_CHANNEL);
}

/** LED_BOTH is two separate runs, not one six-channel write: the two LEDs are
 *  only adjacent by today's numbering, and a merged write would bake that in as
 *  a third undeclared assumption. */
static void test_both_writes_each_led_from_its_own_base(void)
{
    reset_bus();
    CHECK(led_set_both(&k_probe) == ESP_OK, "setting both should succeed");
    CHECK(s_write_count == 2, "LED_BOTH should be two transactions, got %d", s_write_count);
    CHECK(s_writes[0].first_ch == LED_LEFT_R_CHANNEL && s_writes[1].first_ch == LED_RIGHT_R_CHANNEL,
          "expected bases %d then %d, got %d then %d", LED_LEFT_R_CHANNEL, LED_RIGHT_R_CHANNEL,
          s_writes[0].first_ch, s_writes[1].first_ch);
}

int main(void)
{
    printf("test_led_controller\n");

    test_left_components_land_on_their_own_channels();
    test_right_components_land_on_their_own_channels();
    test_both_writes_each_led_from_its_own_base();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
