/**
 * @file light_sensor.h
 * @brief Ambient light sensor driver — LDR on GPIO1 (ADC1_CH0).
 *
 * Uses ESP-IDF v5.4 ADC oneshot driver.  Returns a raw 12-bit reading
 * (0 = dark, 4095 = saturated bright).
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ADC GPIO for the LDR voltage-divider output. */
#define LIGHT_SENSOR_GPIO 1

/** ADC channel corresponding to GPIO1 on ESP32-S3. */
#define LIGHT_SENSOR_ADC_CHANNEL ADC_CHANNEL_0

/**
 * @brief Initialise the ADC oneshot unit for the light sensor.
 *
 * Must be called once before light_sensor_read().
 *
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t light_sensor_init(void);

/**
 * @brief Read the current ambient light level.
 *
 * @param out_raw  Output: 12-bit ADC reading (0 = dark, 4095 = bright).
 * @return ESP_OK on success.
 */
esp_err_t light_sensor_read(int *out_raw);

#ifdef __cplusplus
}
#endif

#endif /* LIGHT_SENSOR_H */
