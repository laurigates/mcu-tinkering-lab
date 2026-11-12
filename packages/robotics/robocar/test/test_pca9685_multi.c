/**
 * @file test_pca9685_multi.c
 *
 * Pins pca9685_set_pwm_values() for a block that does not start at channel 0.
 *
 * The vendored driver used to index both its input array and its stack buffer
 * by the ABSOLUTE channel number while sizing them for the RELATIVE count. With
 * motor_stop()'s call (first_ch = 8, channels = 6) that read six words past a
 * 6-entry array and wrote 24 bytes up to 32 bytes past a 24-byte VLA — straight
 * over the saved registers of the calling frames. On the device it surfaced as
 * a double exception at the first boot with the PCA9685 fitted, with the
 * driver's LED_FULL_ON_OFF flag (0x10) sitting in the high byte of the
 * corrupted return register.
 *
 * The driver is compiled UNMODIFIED against test/include/pca9685_host; the
 * transport below records the last register write. The target builds with
 * AddressSanitizer so the out-of-bounds write itself fails the run, not just
 * the resulting garbage — a coincidentally-plausible buffer must not pass.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pca9685.h"

#define REG_LEDX 0x06
#define LED_FULL_ON_OFF (1 << 4)

/* ---- recording transport ------------------------------------------------ */

static uint8_t s_last_reg;
static uint8_t s_last_data[64];
static size_t s_last_size;
static int s_writes;

esp_err_t i2c_dev_create_mutex(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}
esp_err_t i2c_dev_delete_mutex(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}
esp_err_t i2c_dev_take_mutex(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}
esp_err_t i2c_dev_give_mutex(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
    (void)dev;
    (void)reg;
    memset(in_data, 0, in_size);
    return ESP_OK;
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data,
                            size_t out_size)
{
    (void)dev;
    s_last_reg = reg;
    s_last_size = out_size;
    memset(s_last_data, 0xA5, sizeof(s_last_data));
    if (out_size > sizeof(s_last_data))
        return ESP_ERR_INVALID_ARG;
    memcpy(s_last_data, out_data, out_size);
    s_writes++;
    return ESP_OK;
}

/* ---- harness ------------------------------------------------------------ */

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

static void expect_channel(size_t i, uint16_t value)
{
    const uint8_t *q = &s_last_data[i * 4];
    if (value >= PCA9685_MAX_PWM_VALUE) {
        CHECK(q[0] == 0 && q[1] == LED_FULL_ON_OFF && q[2] == 0xFF && q[3] == 0x0F,
              "slot %zu full-on: %02x %02x %02x %02x", i, q[0], q[1], q[2], q[3]);
    } else if (value == 0) {
        CHECK(q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == LED_FULL_ON_OFF,
              "slot %zu full-off: %02x %02x %02x %02x", i, q[0], q[1], q[2], q[3]);
    } else {
        CHECK(q[0] == 0 && q[1] == 0 && q[2] == (value & 0xFF) && q[3] == (value >> 8),
              "slot %zu pwm %u: %02x %02x %02x %02x", i, value, q[0], q[1], q[2], q[3]);
    }
}

static void test_motor_block_starting_at_channel_8(void)
{
    /* Exactly motor_stop()'s shape: right IN1/IN2/PWM, left IN1/IN2/PWM. */
    i2c_dev_t dev = {0};
    const uint16_t values[6] = {PCA9685_MAX_PWM_VALUE, 0, 1234, 0, PCA9685_MAX_PWM_VALUE, 4095};
    s_writes = 0;

    esp_err_t err = pca9685_set_pwm_values(&dev, 8, 6, values);

    CHECK(err == ESP_OK, "returned %d", err);
    CHECK(s_writes == 1, "expected one register write, got %d", s_writes);
    CHECK(s_last_reg == REG_LEDX + 8 * 4, "register 0x%02x, expected 0x%02x", s_last_reg,
          REG_LEDX + 8 * 4);
    CHECK(s_last_size == 24, "wrote %zu bytes, expected 24", s_last_size);
    for (size_t i = 0; i < 6; i++)
        expect_channel(i, values[i]);
}

static void test_led_block_starting_at_channel_3(void)
{
    /* led_controller's right-LED write, the other non-zero caller. */
    i2c_dev_t dev = {0};
    const uint16_t values[3] = {0, 2048, PCA9685_MAX_PWM_VALUE};
    s_writes = 0;

    CHECK(pca9685_set_pwm_values(&dev, 3, 3, values) == ESP_OK, "returned error");
    CHECK(s_last_reg == REG_LEDX + 3 * 4, "register 0x%02x", s_last_reg);
    CHECK(s_last_size == 12, "wrote %zu bytes", s_last_size);
    for (size_t i = 0; i < 3; i++)
        expect_channel(i, values[i]);
}

static void test_block_starting_at_channel_0_still_works(void)
{
    /* Control: the one shape that never overflowed. */
    i2c_dev_t dev = {0};
    const uint16_t values[3] = {100, 0, PCA9685_MAX_PWM_VALUE};
    s_writes = 0;

    CHECK(pca9685_set_pwm_values(&dev, 0, 3, values) == ESP_OK, "returned error");
    CHECK(s_last_reg == REG_LEDX, "register 0x%02x", s_last_reg);
    CHECK(s_last_size == 12, "wrote %zu bytes", s_last_size);
    for (size_t i = 0; i < 3; i++)
        expect_channel(i, values[i]);
}

static void test_last_channel_alone(void)
{
    /* The far edge: channel 15, the largest absolute index the bug multiplied. */
    i2c_dev_t dev = {0};
    const uint16_t values[1] = {7};
    s_writes = 0;

    CHECK(pca9685_set_pwm_values(&dev, 15, 1, values) == ESP_OK, "returned error");
    CHECK(s_last_reg == REG_LEDX + 15 * 4, "register 0x%02x", s_last_reg);
    CHECK(s_last_size == 4, "wrote %zu bytes", s_last_size);
    expect_channel(0, 7);
}

static void test_out_of_range_block_is_rejected(void)
{
    i2c_dev_t dev = {0};
    const uint16_t values[6] = {0};
    s_writes = 0;

    CHECK(pca9685_set_pwm_values(&dev, 12, 6, values) == ESP_ERR_INVALID_ARG,
          "12+6 channels must be rejected");
    CHECK(s_writes == 0, "rejected call must not touch the bus");
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"motor_block_starting_at_channel_8", test_motor_block_starting_at_channel_8},
        {"led_block_starting_at_channel_3", test_led_block_starting_at_channel_3},
        {"block_starting_at_channel_0_still_works", test_block_starting_at_channel_0_still_works},
        {"last_channel_alone", test_last_channel_alone},
        {"out_of_range_block_is_rejected", test_out_of_range_block_is_rejected},
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
