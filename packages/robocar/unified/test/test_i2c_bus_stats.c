/**
 * @file test_i2c_bus_stats.c
 *
 * Pins the I2C bus activity counters in i2c_bus.c.
 *
 * The counter exists to answer one question from the console — is anything
 * writing this bus on a timer? — so the way it can fail is by answering
 * confidently and wrongly. Three ways it could:
 *
 *  - Reporting a remembered rate after the traffic stopped, which reads as a
 *    busy bus when the bus is silent. That is the reading the instrument exists
 *    to distinguish, so it is the one worth pinning hardest.
 *  - Counting ops per window rather than dividing by the window's real length,
 *    which reports the LED refresh's one op per five seconds as 1/s.
 *  - Losing the uint32 millisecond wrap at day 49 and pinning the rate at 0 or
 *    at nonsense for the rest of the boot.
 *
 * i2c_bus.c is compiled unmodified against the shims in test/include plus the
 * driver stubs below, so the counters are exercised through the real
 * i2c_bus_select_channel() gate rather than by poking their statics.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c_bus.h"
#include "pin_config.h"

/* ---- injectable clock --------------------------------------------------- */

static int64_t s_now_us;

int64_t esp_timer_get_time(void)
{
    return s_now_us;
}

static void advance_ms(uint32_t ms)
{
    s_now_us += (int64_t)ms * 1000;
}

/* ---- i2cdev / driver stubs ---------------------------------------------- */

static esp_err_t s_mux_result = ESP_OK;

esp_err_t i2cdev_init(void)
{
    return ESP_OK;
}

esp_err_t tca9548_init_desc(i2c_dev_t *dev, uint8_t addr, int port, int sda, int scl)
{
    (void)dev;
    (void)addr;
    (void)port;
    (void)sda;
    (void)scl;
    return ESP_OK;
}

esp_err_t tca9548_free_desc(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}

esp_err_t tca9548_set_channels(i2c_dev_t *dev, uint8_t channels)
{
    (void)dev;
    (void)channels;
    return s_mux_result;
}

esp_err_t pca9685_init_desc(i2c_dev_t *dev, uint8_t addr, int port, int sda, int scl)
{
    (void)dev;
    (void)addr;
    (void)port;
    (void)sda;
    (void)scl;
    return ESP_OK;
}

esp_err_t pca9685_free_desc(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}

esp_err_t pca9685_init(i2c_dev_t *dev)
{
    (void)dev;
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_frequency(i2c_dev_t *dev, uint16_t freq)
{
    (void)dev;
    (void)freq;
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_value(i2c_dev_t *dev, uint8_t channel, uint16_t value)
{
    (void)dev;
    (void)channel;
    (void)value;
    return ESP_OK;
}

esp_err_t pca9685_set_pwm_values(i2c_dev_t *dev, uint8_t first, uint8_t count,
                                 const uint16_t *values)
{
    (void)dev;
    (void)first;
    (void)count;
    (void)values;
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

/** One bus acquisition through the real gate, at the current clock. */
static void do_op(uint8_t channel)
{
    if (i2c_bus_select_channel(channel) == ESP_OK) {
        i2c_bus_release();
    }
}

/** `count` ops spread evenly over `span_ms`. */
static void ops_over(uint8_t channel, uint32_t count, uint32_t span_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        do_op(channel);
        advance_ms(span_ms / count);
    }
}

static void test_ops_are_counted_per_channel(void)
{
    CHECK(i2c_bus_init() == ESP_OK, "bus init should succeed");
    i2c_bus_stats_reset();

    do_op(I2C_BUS_CHANNEL_PCA9685);
    do_op(I2C_BUS_CHANNEL_PCA9685);
    do_op(I2C_BUS_CHANNEL_OLED);
    advance_ms(10);

    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);
    CHECK(st.ops == 3u, "expected 3 ops, got %u", (unsigned)st.ops);
    CHECK(st.per_channel[I2C_BUS_CHANNEL_PCA9685] == 2u, "expected 2 on ch0, got %u",
          (unsigned)st.per_channel[I2C_BUS_CHANNEL_PCA9685]);
    CHECK(st.per_channel[I2C_BUS_CHANNEL_OLED] == 1u, "expected 1 on ch1, got %u",
          (unsigned)st.per_channel[I2C_BUS_CHANNEL_OLED]);
    CHECK(st.failed == 0u, "no failures expected, got %u", (unsigned)st.failed);
}

static void test_a_failed_selection_counts_as_a_failure_not_an_op(void)
{
    i2c_bus_stats_reset();
    s_mux_result = ESP_FAIL;

    do_op(I2C_BUS_CHANNEL_PCA9685);
    advance_ms(10);
    s_mux_result = ESP_OK;

    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);
    CHECK(st.ops == 0u, "a failed selection is not an op, got %u", (unsigned)st.ops);
    CHECK(st.failed == 1u, "expected 1 failure, got %u", (unsigned)st.failed);
}

static void test_the_rate_reflects_the_windows_real_length(void)
{
    i2c_bus_stats_reset();

    /* The shape the motor loop used to have: 30 ops a second, sustained. */
    ops_over(I2C_BUS_CHANNEL_PCA9685, 90u, 3000u);

    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);
    CHECK(st.hz >= 28u && st.hz <= 32u, "30 ops/s should read as ~30/s, got %u", (unsigned)st.hz);

    /* And traffic slower than the window: a counter that reported ops-per-window
     * rather than dividing by the window's real length would call this 1/s.
     *
     * The read has to happen while the window is still fresh, or the staleness
     * rule below zeroes the rate and the assertion passes without ever
     * exercising the division. That is how an earlier version of this test
     * silently lost its teeth. */
    i2c_bus_stats_reset();
    for (int i = 0; i < 6; i++) {
        do_op(I2C_BUS_CHANNEL_PCA9685);
        advance_ms(2000u);
    }
    /* 500 ms after the last op: well inside the staleness bound, so `hz` is a
     * live reading of one op per two seconds. */
    s_now_us -= (int64_t)1500u * 1000;
    i2c_bus_stats_get(&st);
    CHECK(st.hz == 0u, "one op per 2 s must not read as 1/s, got %u", (unsigned)st.hz);
    CHECK(st.ops == 6u, "the total must still be exact, got %u", (unsigned)st.ops);
}

static void test_a_stale_rate_reads_as_zero(void)
{
    i2c_bus_stats_reset();
    ops_over(I2C_BUS_CHANNEL_PCA9685, 90u, 3000u);

    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);
    CHECK(st.hz > 0u, "precondition: the rate should be live, got %u", (unsigned)st.hz);
    const uint32_t peak = st.peak_hz;
    CHECK(peak > 0u, "precondition: a peak should have been recorded");

    /* Traffic stops. The window is advanced by ops, so nothing will ever clear
     * the last rate on its own — the read has to notice. */
    advance_ms(60000u);
    i2c_bus_stats_get(&st);
    CHECK(st.hz == 0u, "a bus that went quiet must read 0/s, got %u", (unsigned)st.hz);
    CHECK(st.peak_hz == peak, "the peak is a high-water mark and must survive, got %u",
          (unsigned)st.peak_hz);
    CHECK(st.ops == 90u, "the total is cumulative and must survive, got %u", (unsigned)st.ops);
}

static void test_counters_survive_the_uint32_millisecond_wrap(void)
{
    i2c_bus_stats_reset();

    /* Park just under the wrap and sustain traffic across it. */
    s_now_us = ((int64_t)0xFFFFF000u) * 1000;
    i2c_bus_stats_reset();
    ops_over(I2C_BUS_CHANNEL_PCA9685, 120u, 4000u);

    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);
    CHECK(st.ops == 120u, "every op must be counted across the wrap, got %u", (unsigned)st.ops);
    CHECK(st.hz >= 28u && st.hz <= 32u, "the rate must survive the wrap, got %u", (unsigned)st.hz);
    CHECK(st.elapsed_ms >= 3900u && st.elapsed_ms <= 4100u,
          "the span must survive the wrap, got %u ms", (unsigned)st.elapsed_ms);

    /* The rate must keep TRACKING past the wrap, not merely read plausibly once.
     * A comparison that goes wrong across the wrap can leave the window unable
     * to roll ever again, freezing the last rate for the rest of the boot — and
     * a frozen 30/s looks exactly like a live 30/s. Changing the traffic is the
     * only thing that tells them apart. */
    ops_over(I2C_BUS_CHANNEL_PCA9685, 25u, 5000u);
    i2c_bus_stats_get(&st);
    CHECK(st.hz >= 4u && st.hz <= 6u, "the rate must follow the traffic past the wrap, got %u",
          (unsigned)st.hz);

    /* And the staleness check must still fire on the far side of it. */
    advance_ms(60000u);
    i2c_bus_stats_get(&st);
    CHECK(st.hz == 0u, "staleness must still be detected across the wrap, got %u", (unsigned)st.hz);
}

int main(void)
{
    printf("test_i2c_bus_stats\n");

    test_ops_are_counted_per_channel();
    test_a_failed_selection_counts_as_a_failure_not_an_op();
    test_the_rate_reflects_the_windows_real_length();
    test_a_stale_rate_reads_as_zero();
    test_counters_survive_the_uint32_millisecond_wrap();

    if (s_failures != 0) {
        printf("FAILED (%d)\n", s_failures);
        return EXIT_FAILURE;
    }
    printf("PASSED\n");
    return EXIT_SUCCESS;
}
