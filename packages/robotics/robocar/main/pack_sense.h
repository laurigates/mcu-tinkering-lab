/**
 * @file pack_sense.h
 * @brief The pack divider on ADC1, read through the calibrated oneshot driver
 *        (ADR-038, #1271).
 *
 * PACK_SENSE_PIN (GPIO4, ADC1_CH3) sits on R17 100 kOhm / R18 22 kOhm from the
 * switched pack rail, so the pad is 0.1803 of the pack voltage
 * (power_core.h's POWER_DIVIDER_*). ADC1 only: ADC2 is unusable while Wi-Fi
 * runs. The conversion to pack millivolts is power_core.c's.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set up the ADC1 unit, the pad's channel and its calibration.
 *
 * @return ESP_OK, or the driver's error. Calibration is required: without it
 *         the reading is not millivolts, so a failure here leaves the monitor
 *         off rather than guessing.
 */
esp_err_t pack_sense_init(void);

/**
 * @brief Read the pack voltage.
 *
 * Averages PACK_SENSE_SAMPLES conversions back to back, which also spreads the
 * reading across the 100 nF at the pad.
 *
 * @param[out] pack_mv Pack voltage in millivolts.
 * @return ESP_OK, ESP_ERR_INVALID_STATE before init, or the driver's error.
 */
esp_err_t pack_sense_read_mv(uint32_t *pack_mv);

#ifdef __cplusplus
}
#endif
