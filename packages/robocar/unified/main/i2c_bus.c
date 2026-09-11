/**
 * @file i2c_bus.c
 * @brief TCA9548A-based I2C bus multiplexer with thread-safe access
 */

#include "i2c_bus.h"
#include "pca9685_phase.h"
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

/* Force every device on this port onto one bus clock.
 *
 * Each esp-idf-lib driver picks its own rate in init_desc() — the TCA9548A
 * 100 kHz, the PCA9685 and MCP23017 1 MHz — so without this the bus runs at
 * three different speeds depending on which device was addressed last.
 * I2C_MASTER_FREQ_HZ (400 kHz) is the fastest rate every part here is rated
 * for: the TCA9548A is a Fast-mode part, the other two are Fm+. It had sat in
 * pin_config.h unread since the first commit.
 *
 * Under i2cdev 1.x this also fixed a real cost. cfg_equal() compared
 * clk_speed, so consecutive transactions at different rates made
 * i2c_setup_port() delete and reinstall the entire I2C driver first — and
 * since a mux channel-select always precedes a device write, that fired twice
 * per motor update, ~30 times a second while driving. i2cdev 2.0.0 moved to
 * ESP-IDF's bus/device API, which caches a per-device handle with its own
 * scl_speed_hz on a shared bus, so the reinstall is gone by construction. What
 * remains is the first paragraph: one deliberate bus speed instead of three
 * accidental ones, and 400 kHz rather than 1 MHz over jumper wiring.
 *
 * Call this after the driver's own init_desc(), which sets clk_speed itself. */
static void pin_bus_clock(i2c_dev_t *dev)
{
    dev->cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
}

/* -------------------------------------------------------------------------- */
/* Phase-staggered PCA9685 writes                                              */
/*                                                                             */
/* The vendored driver hardcodes each channel's ON count to 0, so every output */
/* rises on the same tick of the ~197 Hz period: two motor PWMs, two servos    */
/* and six LED channels all switch together, ~197 times a second. That is the  */
/* worst case for peak current on the shared 5 V rail, and the PCA9685 has a   */
/* per-channel ON register precisely so the edges can be spread instead.       */
/* Upstream esp-idf-lib exposes no API for it (checked against                 */
/* UncleRus/esp-idf-lib at the time of writing), so the register bytes are     */
/* composed here rather than by patching the vendored copy — which is a        */
/* symlink into robocar/main and would change that firmware too, and would be  */
/* silently reverted by any future refresh of the library.                     */
/*                                                                             */
/* Note this is headroom, not a fix for an observed fault: the audio           */
/* distortion that prompted the investigation was resolved by adding bulk      */
/* capacitance.                                                                */
/* -------------------------------------------------------------------------- */

/** Write `count` consecutive channels with phase staggering applied.
 *
 * Relies on the MODE1 auto-increment that pca9685_init() sets, exactly as the
 * vendored multi-write does. Caller must already hold the bus. */
static esp_err_t pca9685_write_staggered(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    if (values == NULL || count == 0u || (uint16_t)first_ch + count > PCA9685_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[PCA9685_CHANNELS * 4u];
    for (uint8_t i = 0; i < count; i++) {
        pca9685_encode((uint8_t)(first_ch + i), values[i], &buf[i * 4]);
    }

    /* The I2C_DEV_* macros return on failure and release the mutex themselves,
     * exactly as the vendored driver's own writers do. */
    I2C_DEV_TAKE_MUTEX(&s_pca9685);
    I2C_DEV_CHECK(&s_pca9685,
                  i2c_dev_write_reg(&s_pca9685, (uint8_t)(PCA9685_REG_LED0 + first_ch * 4), buf,
                                    (size_t)count * 4u));
    I2C_DEV_GIVE_MUTEX(&s_pca9685);

    return ESP_OK;
}

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
    pin_bus_clock(&s_tca9548);

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
    pin_bus_clock(&s_pca9685);

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
    ESP_LOGI(TAG, "I2C bus initialized: TCA9548A(0x%02X) + PCA9685(0x%02X), bus %d kHz, PWM %d Hz",
             TCA9548A_ADDR, PCA9685_ADDR, I2C_MASTER_FREQ_HZ / 1000, PCA9685_FREQ_HZ);
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

    ret = pca9685_write_staggered(channel, 1, &value);

    i2c_bus_release();
    return ret;
}

esp_err_t i2c_bus_pca9685_set_multi(uint8_t first_ch, uint8_t count, const uint16_t *values)
{
    esp_err_t ret = i2c_bus_select_channel(I2C_BUS_CHANNEL_PCA9685);
    if (ret != ESP_OK)
        return ret;

    ret = pca9685_write_staggered(first_ch, count, values);

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
