/**
 * @file pot_reader.c
 * @brief ADC oneshot potentiometer reader — ESP-IDF v5.4 oneshot API.
 *
 * Samples every 50 ms; applies exponential smoothing with alpha=0.2.
 */

#include "pot_reader.h"

#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "pot_reader";

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define ADC_UNIT ADC_UNIT_1
#define POT_TEMPO_CHANNEL ADC_CHANNEL_0 /**< GPIO 1 on ESP32-S3 */
#define POT_PITCH_CHANNEL ADC_CHANNEL_1 /**< GPIO 2 on ESP32-S3 */
#define ADC_BITWIDTH ADC_BITWIDTH_12
#define ADC_ATTEN ADC_ATTEN_DB_12 /**< Full 0-3.3 V range */

#define SAMPLE_INTERVAL_MS 50u
#define SMOOTH_ALPHA_NUM 2u /**< alpha = 0.2 = 2/10 */
#define SMOOTH_ALPHA_DEN 10u

#define ADC_MAX_RAW 4095u
#define BPM_MIN 60u
#define BPM_MAX 200u
#define SEMITONE_MIN (-12)
#define SEMITONE_MAX 12

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

static adc_oneshot_unit_handle_t s_adc_handle;
static uint16_t s_smoothed[POT_COUNT];
static uint32_t s_last_sample_ms;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int pot_reader_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };

    ret = adc_oneshot_config_channel(s_adc_handle, POT_TEMPO_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC tempo channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_oneshot_config_channel(s_adc_handle, POT_PITCH_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC pitch channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Seed smoothing state with a first sample */
    int raw_tempo = 0, raw_pitch = 0;
    adc_oneshot_read(s_adc_handle, POT_TEMPO_CHANNEL, &raw_tempo);
    adc_oneshot_read(s_adc_handle, POT_PITCH_CHANNEL, &raw_pitch);
    s_smoothed[POT_TEMPO] = (uint16_t)raw_tempo;
    s_smoothed[POT_PITCH] = (uint16_t)raw_pitch;
    s_last_sample_ms = 0;

    ESP_LOGI(TAG, "Pot reader initialized (tempo ch%d, pitch ch%d)", POT_TEMPO_CHANNEL,
             POT_PITCH_CHANNEL);
    return ESP_OK;
}

void pot_reader_tick(uint32_t now_ms)
{
    if ((now_ms - s_last_sample_ms) < SAMPLE_INTERVAL_MS) {
        return;
    }
    s_last_sample_ms = now_ms;

    int raw_tempo = 0, raw_pitch = 0;
    if (adc_oneshot_read(s_adc_handle, POT_TEMPO_CHANNEL, &raw_tempo) != ESP_OK) {
        raw_tempo = (int)s_smoothed[POT_TEMPO];
    }
    if (adc_oneshot_read(s_adc_handle, POT_PITCH_CHANNEL, &raw_pitch) != ESP_OK) {
        raw_pitch = (int)s_smoothed[POT_PITCH];
    }

    /* Exponential smoothing: new = old + alpha*(sample - old), alpha = 0.2 */
    s_smoothed[POT_TEMPO] =
        (uint16_t)(s_smoothed[POT_TEMPO] + SMOOTH_ALPHA_NUM *
                                               ((int)raw_tempo - (int)s_smoothed[POT_TEMPO]) /
                                               (int)SMOOTH_ALPHA_DEN);
    s_smoothed[POT_PITCH] =
        (uint16_t)(s_smoothed[POT_PITCH] + SMOOTH_ALPHA_NUM *
                                               ((int)raw_pitch - (int)s_smoothed[POT_PITCH]) /
                                               (int)SMOOTH_ALPHA_DEN);
}

uint16_t pot_reader_get_raw(pot_id_t pot)
{
    if (pot >= POT_COUNT)
        return 0;
    return s_smoothed[pot];
}

uint16_t pot_tempo_to_bpm(uint16_t raw)
{
    if (raw > ADC_MAX_RAW)
        raw = ADC_MAX_RAW;
    return (uint16_t)(BPM_MIN + (uint32_t)(BPM_MAX - BPM_MIN) * raw / ADC_MAX_RAW);
}

int8_t pot_pitch_to_semitones(uint16_t raw)
{
    if (raw > ADC_MAX_RAW)
        raw = ADC_MAX_RAW;
    /* Map [0..4095] -> [-12..+12] = 24 semitone range */
    int32_t shift = SEMITONE_MIN + (int32_t)24 * raw / (int32_t)ADC_MAX_RAW;
    if (shift < SEMITONE_MIN)
        shift = SEMITONE_MIN;
    if (shift > SEMITONE_MAX)
        shift = SEMITONE_MAX;
    return (int8_t)shift;
}
