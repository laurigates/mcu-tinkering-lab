/**
 * @file i2c_bus.c
 * @brief TCA9548A-based I2C bus multiplexer with thread-safe access
 */

#include "i2c_bus.h"
#include "pin_config.h"

#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <i2cdev.h>
#include <pca9685.h>
#include <tca9548.h>

static const char *TAG = "i2c_bus";

static i2c_dev_t s_tca9548;
static i2c_dev_t s_pca9685;
static SemaphoreHandle_t s_bus_mutex;
static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/* Bus activity counters                                                        */
/*                                                                              */
/* Counted here because i2c_bus_select_channel() is the single gate every        */
/* downstream transaction passes through — PCA9685 writes, the expander, and     */
/* anything added later. Instrumenting the callers instead would mean a new      */
/* writer goes uncounted, which is precisely the case worth catching: an         */
/* unguarded periodic write is invisible from the console and only shows up as   */
/* a symptom somewhere else on the board.                                        */
/*                                                                              */
/* One "op" is one bus acquisition: a mux channel-select write plus whatever     */
/* device traffic the caller then issues. It is a lower bound on wire            */
/* transactions, never an overcount.                                             */
/*                                                                              */
/* No lock. The success path already holds the bus mutex, so those increments    */
/* are serialised; the two failure paths are not, so a simultaneous failure on   */
/* both cores can lose a count. That is the same trade activity_trace makes —    */
/* an instrument that blocks the path it measures changes the number it is       */
/* reporting.                                                                    */
/* -------------------------------------------------------------------------- */

/** Length of the rate window. */
#define I2C_STATS_WINDOW_MS 1000u

static uint32_t s_ops;
static uint32_t s_failed;
static uint32_t s_per_channel[8];
static uint32_t s_window_start_ms;
static uint32_t s_window_ops;
static uint32_t s_last_hz;
static uint32_t s_peak_hz;
static uint32_t s_since_ms;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/** Fold a completed op into the rate window.
 *
 *  The window is advanced by traffic, not by a timer, so its length is however
 *  long the ops actually took — hence the division rather than a bare count. A
 *  bare count would report the LED refresh's one op per five seconds as 1 Hz. */
static void note_op(uint8_t channel)
{
    const uint32_t now = now_ms();

    s_ops++;
    if (channel < 8u) {
        s_per_channel[channel]++;
    }

    if (s_window_start_ms == 0u) {
        s_window_start_ms = (now == 0u) ? 1u : now;
    }

    const uint32_t elapsed = now - s_window_start_ms;
    if (elapsed >= I2C_STATS_WINDOW_MS) {
        s_last_hz = (uint32_t)(((uint64_t)s_window_ops * 1000u) / elapsed);
        if (s_last_hz > s_peak_hz) {
            s_peak_hz = s_last_hz;
        }
        s_window_ops = 0u;
        s_window_start_ms = (now == 0u) ? 1u : now;
    }
    s_window_ops++;
}

void i2c_bus_stats_get(i2c_bus_stats_t *out)
{
    if (out == NULL) {
        return;
    }

    const uint32_t now = now_ms();

    out->ops = s_ops;
    out->failed = s_failed;
    memcpy(out->per_channel, s_per_channel, sizeof(out->per_channel));
    out->peak_hz = s_peak_hz;
    out->elapsed_ms = now - s_since_ms;

    /* A rate whose window has not been touched for a while is not a rate, it is
     * a memory of one. Reporting the stale value would read as live traffic on
     * a bus that has gone quiet — the exact reading this counter exists to
     * distinguish. */
    out->hz = (s_window_start_ms != 0u && (now - s_window_start_ms) < (2u * I2C_STATS_WINDOW_MS))
                  ? s_last_hz
                  : 0u;
}

void i2c_bus_stats_reset(void)
{
    s_ops = 0u;
    s_failed = 0u;
    memset(s_per_channel, 0, sizeof(s_per_channel));
    s_window_start_ms = 0u;
    s_window_ops = 0u;
    s_last_hz = 0u;
    s_peak_hz = 0u;
    s_since_ms = now_ms();
}

void i2c_bus_stats_report(void)
{
    i2c_bus_stats_t st;
    i2c_bus_stats_get(&st);

    const uint32_t secs = st.elapsed_ms / 1000u;
    /* Tenths, by integer math: at these rates a whole number rounds the idle
     * bus to 0 and hides exactly the difference being measured. */
    const uint32_t mean_tenths =
        (st.elapsed_ms > 0u) ? (uint32_t)(((uint64_t)st.ops * 10000u) / st.elapsed_ms) : 0u;

    printf("  i2c: %u ops in %us (mean %u.%u/s, now %u/s, peak %u/s), %u failed\n",
           (unsigned)st.ops, (unsigned)secs, (unsigned)(mean_tenths / 10u),
           (unsigned)(mean_tenths % 10u), (unsigned)st.hz, (unsigned)st.peak_hz,
           (unsigned)st.failed);

    printf("       per channel:");
    bool any = false;
    for (uint8_t ch = 0; ch < 8u; ++ch) {
        if (st.per_channel[ch] > 0u) {
            printf(" ch%u=%u", (unsigned)ch, (unsigned)st.per_channel[ch]);
            any = true;
        }
    }
    printf("%s\n", any ? "" : " none");
}

/* The PCA9685's prescaler is chip-wide: one frequency for motors, servos and
 * LEDs alike. Tracked here because the servo pulse maths needs it — a pulse
 * width is only a pulse width once you know the period it sits in — and
 * because the bench needs to change it without a reflash to find out whether
 * the servos are simply being clocked out of spec. */
static uint16_t s_pwm_freq_hz = PCA9685_FREQ_HZ;

esp_err_t i2c_bus_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Initialize i2cdev library
    esp_err_t ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create bus mutex
    s_bus_mutex = xSemaphoreCreateRecursiveMutex();
    if (!s_bus_mutex) {
        ESP_LOGE(TAG, "Failed to create bus mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize TCA9548A multiplexer
    ret = tca9548_init_desc(&s_tca9548, TCA9548A_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548A init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Disable all channels initially
    ret = tca9548_set_channels(&s_tca9548, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548A set_channels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "TCA9548A initialized at 0x%02X", TCA9548A_ADDR);

    // Initialize PCA9685 via TCA9548A channel 0
    ret = tca9548_set_channels(&s_tca9548, TCA9548_CHANNEL0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select PCA9685 channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_init_desc(&s_pca9685, PCA9685_ADDR, I2C_NUM_0, I2C_SDA_PIN, I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_init(&s_pca9685);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pca9685_set_pwm_frequency(&s_pca9685, PCA9685_FREQ_HZ);
    s_pwm_freq_hz = PCA9685_FREQ_HZ;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 set_pwm_frequency failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // All channels off initially
    ret = pca9685_set_pwm_value(&s_pca9685, PCA9685_CHANNEL_ALL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 all-off failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Disable all TCA9548A channels after init
    tca9548_set_channels(&s_tca9548, 0x00);

    s_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized: TCA9548A(0x%02X) + PCA9685(0x%02X) @ %dHz", TCA9548A_ADDR,
             PCA9685_ADDR, PCA9685_FREQ_HZ);
    return ESP_OK;
}

bool i2c_bus_is_ready(void)
{
    return s_initialized;
}

esp_err_t i2c_bus_select_channel(uint8_t channel)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTakeRecursive(s_bus_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Bus mutex timeout");
        s_failed++;
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = tca9548_set_channels(&s_tca9548, (1 << channel));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Channel select failed: %s", esp_err_to_name(ret));
        s_failed++;
        xSemaphoreGiveRecursive(s_bus_mutex);
        return ret;
    }

    note_op(channel);
    return ESP_OK;
}

esp_err_t i2c_bus_release(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGiveRecursive(s_bus_mutex);
    return ESP_OK;
}

i2c_dev_t *i2c_bus_get_pca9685(void)
{
    return &s_pca9685;
}

uint16_t i2c_bus_pca9685_frequency(void)
{
    return s_pwm_freq_hz;
}

esp_err_t i2c_bus_pca9685_set_frequency(uint16_t hz)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* The driver's own range. Below 24 Hz the prescaler saturates; above 1526
     * it underflows. Rejecting here keeps a typo from silently landing on a
     * clamped value that then reads back as if it had been accepted. */
    if (hz < 24u || hz > 1526u) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pca9685_set_pwm_frequency(&s_pca9685, hz);
    i2c_bus_release();

    if (ret == ESP_OK) {
        s_pwm_freq_hz = hz;
    }
    return ret;
}

esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t value)
{
    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK)
        return ret;

    ret = pca9685_set_pwm_value(&s_pca9685, channel, value);

    i2c_bus_release();
    return ret;
}

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK)
        return ret;

    ret = pca9685_set_pwm_values(&s_pca9685, first_ch, count, values);

    i2c_bus_release();
    return ret;
}

void i2c_bus_deinit(void)
{
    if (!s_initialized)
        return;

    // All PCA9685 channels off
    tca9548_set_channels(&s_tca9548, TCA9548_CHANNEL0);
    pca9685_set_pwm_value(&s_pca9685, PCA9685_CHANNEL_ALL, 0);
    tca9548_set_channels(&s_tca9548, 0x00);

    pca9685_free_desc(&s_pca9685);
    tca9548_free_desc(&s_tca9548);

    if (s_bus_mutex) {
        vSemaphoreDelete(s_bus_mutex);
        s_bus_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "I2C bus deinitialized");
}
