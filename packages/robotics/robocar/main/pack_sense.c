/**
 * @file pack_sense.c
 * @brief ADC1 reader for the pack divider (see pack_sense.h).
 */

#include "pack_sense.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "pin_config.h"
#include "power_core.h"

static const char *TAG = "pack_sense";

/** Conversions averaged per reading. */
#define PACK_SENSE_SAMPLES 16

/** 6 dB: the ESP32-S3 datasheet calibrates ATTEN2 over 0-1600 mV (Table 5-6),
 *  for the 1.51 V a full 2S pack puts on the pad. A pack above about 8.87 V
 *  leaves the calibrated range; 12 dB covers more and resolves less of the
 *  6.6 V cutoff (1.19 V). */
#define PACK_SENSE_ATTEN ADC_ATTEN_DB_6

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;

esp_err_t pack_sense_init(void)
{
    if (s_adc != NULL) {
        return ESP_OK;
    }

    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel((int)PACK_SENSE_PIN, &unit, &s_channel);
    if (err != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC1 pad (%s)", (int)PACK_SENSE_PIN, esp_err_to_name(err));
        return err != ESP_OK ? err : ESP_ERR_INVALID_ARG;
    }

    const adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 unit: %s", esp_err_to_name(err));
        s_adc = NULL;
        return err;
    }

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = PACK_SENSE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg);
    if (err == ESP_OK) {
        const adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = s_channel,
            .atten = PACK_SENSE_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel or calibration: %s", esp_err_to_name(err));
        (void)adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        s_cali = NULL;
        return err;
    }
    ESP_LOGI(TAG, "ADC1 channel %d on GPIO%d, calibrated", (int)s_channel, (int)PACK_SENSE_PIN);
    return ESP_OK;
}

esp_err_t pack_sense_read_mv(uint32_t *pack_mv)
{
    if (s_adc == NULL || s_cali == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pack_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int sum = 0;
    for (int i = 0; i < PACK_SENSE_SAMPLES; i++) {
        int raw = 0;
        const esp_err_t err = adc_oneshot_read(s_adc, s_channel, &raw);
        if (err != ESP_OK) {
            return err;
        }
        sum += raw;
    }

    int pin_mv = 0;
    const esp_err_t err = adc_cali_raw_to_voltage(s_cali, sum / PACK_SENSE_SAMPLES, &pin_mv);
    if (err != ESP_OK) {
        return err;
    }
    *pack_mv = power_pack_mv_from_pin_mv(pin_mv > 0 ? (uint32_t)pin_mv : 0);
    return ESP_OK;
}
