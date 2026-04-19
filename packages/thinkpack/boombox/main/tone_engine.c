/**
 * @file tone_engine.c
 * @brief LEDC PWM tone engine — piezo buzzer on GPIO 5.
 *
 * 13-bit resolution (8192 steps). Safety cap: duty never exceeds 1<<12 (50%).
 * Frequency is clamped to [100, 3500] Hz per FR-T21 toddler safety constraints.
 */

#include "tone_engine.h"

#include <math.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "tone_engine";

/* ------------------------------------------------------------------ */
/* Hardware constants                                                  */
/* ------------------------------------------------------------------ */

#define PIEZO_GPIO 5
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT /**< 13-bit: 8192 steps              */
#define LEDC_DUTY_ON (1u << 12)         /**< 50% duty — safety cap (FR-T21)  */
#define LEDC_INITIAL_FREQ 440u

/* Safety frequency bounds (FR-T21) */
#define TONE_FREQ_MIN_HZ 100u
#define TONE_FREQ_MAX_HZ 3500u

/* ------------------------------------------------------------------ */
/* Note frequency table (equal temperament, A4 = 440 Hz)              */
/* ------------------------------------------------------------------ */

static const uint32_t s_note_freq_hz[NOTE_COUNT] = {
    0,    /* NOTE_REST  */
    262,  /* NOTE_C4    */
    277,  /* NOTE_Cs4   */
    294,  /* NOTE_D4    */
    311,  /* NOTE_Ds4   */
    330,  /* NOTE_E4    */
    349,  /* NOTE_F4    */
    370,  /* NOTE_Fs4   */
    392,  /* NOTE_G4    */
    415,  /* NOTE_Gs4   */
    440,  /* NOTE_A4    */
    466,  /* NOTE_As4   */
    494,  /* NOTE_B4    */
    523,  /* NOTE_C5    */
    554,  /* NOTE_Cs5   */
    587,  /* NOTE_D5    */
    622,  /* NOTE_Ds5   */
    659,  /* NOTE_E5    */
    698,  /* NOTE_F5    */
    740,  /* NOTE_Fs5   */
    784,  /* NOTE_G5    */
    831,  /* NOTE_Gs5   */
    880,  /* NOTE_A5    */
    932,  /* NOTE_As5   */
    988,  /* NOTE_B5    */
    1047, /* NOTE_C6    */
};

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int tone_engine_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_INITIAL_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t chan_cfg = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIEZO_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Tone engine initialized on GPIO%d (13-bit, duty cap %u)", PIEZO_GPIO,
             LEDC_DUTY_ON);
    return ESP_OK;
}

void tone_engine_play(note_t note, int8_t semitone_shift)
{
    if (note == NOTE_REST || note >= NOTE_COUNT) {
        tone_engine_stop();
        return;
    }

    /* Clamp semitone shift */
    if (semitone_shift < -12)
        semitone_shift = -12;
    if (semitone_shift > 12)
        semitone_shift = 12;

    uint32_t base_freq = s_note_freq_hz[note];
    uint32_t freq = base_freq;

    if (semitone_shift != 0) {
        float shifted = (float)base_freq * powf(2.0f, (float)semitone_shift / 12.0f);
        freq = (uint32_t)(shifted + 0.5f);
    }

    /* Clamp to safe frequency range (FR-T21) */
    if (freq < TONE_FREQ_MIN_HZ)
        freq = TONE_FREQ_MIN_HZ;
    if (freq > TONE_FREQ_MAX_HZ)
        freq = TONE_FREQ_MAX_HZ;

    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void tone_engine_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
