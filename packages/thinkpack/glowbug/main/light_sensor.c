/**
 * @file light_sensor.c
 * @brief Ambient light sensor driver using ADC oneshot (ESP-IDF v5.4).
 *
 * LDR is wired as: 3.3V → LDR → GPIO1 → 10kΩ → GND.
 * Higher ADC reading = brighter ambient light.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include "light_sensor.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "light_sensor";

static adc_oneshot_unit_handle_t s_adc;

esp_err_t light_sensor_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12, /* 0-3.3V range */
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_oneshot_config_channel(s_adc, LIGHT_SENSOR_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Light sensor initialized on GPIO%d (ADC1_CH%d)", LIGHT_SENSOR_GPIO,
             LIGHT_SENSOR_ADC_CHANNEL);
    return ESP_OK;
}

esp_err_t light_sensor_read(int *out_raw)
{
    return adc_oneshot_read(s_adc, LIGHT_SENSOR_ADC_CHANNEL, out_raw);
}
