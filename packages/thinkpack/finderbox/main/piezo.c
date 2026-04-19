/**
 * @file piezo.c
 * @brief LEDC-PWM piezo tone generator.
 *
 * Uses LEDC low-speed timer 0, channel 0.  Duty cycle is fixed at 50%.
 * Duration cut-off is implemented with an esp_timer one-shot so callers
 * do not block.
 */

#include "piezo.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "piezo";

#define PIEZO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define PIEZO_LEDC_TIMER LEDC_TIMER_0
#define PIEZO_LEDC_CHANNEL LEDC_CHANNEL_0
#define PIEZO_DUTY_RES LEDC_TIMER_10_BIT
#define PIEZO_DUTY_HALF (1 << (10 - 1)) /* 50% of 10-bit range */

static esp_timer_handle_t s_stop_timer;
static bool s_initialized = false;

static void stop_timer_cb(void *arg)
{
    (void)arg;
    piezo_stop();
}

esp_err_t piezo_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    /* Timer starts at 2 kHz; frequency is reconfigured on each play call. */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = PIEZO_LEDC_MODE,
        .timer_num = PIEZO_LEDC_TIMER,
        .duty_resolution = PIEZO_DUTY_RES,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ch_cfg = {
        .gpio_num = PIEZO_GPIO,
        .speed_mode = PIEZO_LEDC_MODE,
        .channel = PIEZO_LEDC_CHANNEL,
        .timer_sel = PIEZO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0, /* silent until play */
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_timer_create_args_t t_args = {
        .callback = stop_timer_cb,
        .name = "piezo_stop",
    };
    ret = esp_timer_create(&t_args, &s_stop_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Piezo initialised on GPIO%d", PIEZO_GPIO);
    return ESP_OK;
}

void piezo_play_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!s_initialized || freq_hz == 0) {
        return;
    }

    /* Cancel any in-flight stop timer before restarting. */
    esp_timer_stop(s_stop_timer);

    esp_err_t ret = ledc_set_freq(PIEZO_LEDC_MODE, PIEZO_LEDC_TIMER, freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ledc_set_freq(%u) failed: %s", (unsigned)freq_hz, esp_err_to_name(ret));
        return;
    }
    ledc_set_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL, PIEZO_DUTY_HALF);
    ledc_update_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL);

    if (duration_ms > 0) {
        esp_timer_start_once(s_stop_timer, (uint64_t)duration_ms * 1000ULL);
    }
}

void piezo_stop(void)
{
    if (!s_initialized) {
        return;
    }
    ledc_set_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL, 0);
    ledc_update_duty(PIEZO_LEDC_MODE, PIEZO_LEDC_CHANNEL);
}
