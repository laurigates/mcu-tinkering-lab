/**
 * @file piezo_voice.c
 * @brief LEDC-backed square-wave driver for piezo accent voices.
 */

#include "piezo_voice.h"

#include <stdbool.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "piezo_voice";

#define PIEZO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define PIEZO_LEDC_RES LEDC_TIMER_8_BIT
#define PIEZO_LEDC_DUTY_ON 128 /* 50% of 2^8 */
#define PIEZO_LEDC_DUTY_OFF 0
#define PIEZO_INIT_FREQ_HZ 1000
#define PIEZO_MIN_FREQ_HZ 30.0f
#define PIEZO_MAX_FREQ_HZ 20000.0f

typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
    ledc_timer_t timer;
    bool initialized;
    bool on;
} piezo_voice_t;

static piezo_voice_t s_voices[PIEZO_COUNT];

void piezo_voice_init(piezo_id_t id, gpio_num_t gpio)
{
    if (id >= PIEZO_COUNT) {
        ESP_LOGE(TAG, "invalid piezo id %d", (int)id);
        return;
    }

    ledc_channel_t channel = (id == PIEZO_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
    ledc_timer_t timer = (id == PIEZO_A) ? LEDC_TIMER_0 : LEDC_TIMER_1;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = PIEZO_LEDC_MODE,
        .timer_num = timer,
        .duty_resolution = PIEZO_LEDC_RES,
        .freq_hz = PIEZO_INIT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num = gpio,
        .speed_mode = PIEZO_LEDC_MODE,
        .channel = channel,
        .timer_sel = timer,
        .duty = PIEZO_LEDC_DUTY_OFF,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s_voices[id] = (piezo_voice_t){
        .gpio = gpio,
        .channel = channel,
        .timer = timer,
        .initialized = true,
        .on = false,
    };

    ESP_LOGI(TAG, "piezo %d on GPIO%d (timer %d, channel %d)", (int)id, (int)gpio, (int)timer,
             (int)channel);
}

void piezo_voice_note_on(piezo_id_t id, float freq_hz)
{
    if (id >= PIEZO_COUNT || !s_voices[id].initialized)
        return;

    if (freq_hz < PIEZO_MIN_FREQ_HZ)
        freq_hz = PIEZO_MIN_FREQ_HZ;
    if (freq_hz > PIEZO_MAX_FREQ_HZ)
        freq_hz = PIEZO_MAX_FREQ_HZ;

    esp_err_t err = ledc_set_freq(PIEZO_LEDC_MODE, s_voices[id].timer, (uint32_t)freq_hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "piezo %d: ledc_set_freq(%u) failed: %s", (int)id, (unsigned)freq_hz,
                 esp_err_to_name(err));
        return;
    }

    if (!s_voices[id].on) {
        ESP_ERROR_CHECK(ledc_set_duty(PIEZO_LEDC_MODE, s_voices[id].channel, PIEZO_LEDC_DUTY_ON));
        ESP_ERROR_CHECK(ledc_update_duty(PIEZO_LEDC_MODE, s_voices[id].channel));
        s_voices[id].on = true;
    }
}

void piezo_voice_note_off(piezo_id_t id)
{
    if (id >= PIEZO_COUNT || !s_voices[id].initialized || !s_voices[id].on)
        return;

    ESP_ERROR_CHECK(ledc_set_duty(PIEZO_LEDC_MODE, s_voices[id].channel, PIEZO_LEDC_DUTY_OFF));
    ESP_ERROR_CHECK(ledc_update_duty(PIEZO_LEDC_MODE, s_voices[id].channel));
    s_voices[id].on = false;
}
