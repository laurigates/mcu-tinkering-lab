/**
 * @file power_manager.h
 * @brief Pack monitor, power button and the shutdown sequence (ADR-038,
 *        #1271).
 *
 * One task on Core 1 polls the pack divider and the button's expander pin
 * every POWER_POLL_MS and feeds power_core.c. A tripped low-pack cutoff or a
 * long press runs the same shutdown: coast the wheels, centre the head, say
 * so, release HOLD. The pack voltage is published for the OLED, MQTT
 * (POWER_MQTT_TOPIC) and GlitchTip events (a `pack_mv` extra).
 *
 * Everything here is inert unless FEATURE_POWER_LATCH_ENABLED is set.
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the monitor task.
 *
 * Call after init_hardware() (the button is an expander pin) and after the
 * executor is up (the shutdown holds it). Non-fatal: without the ADC or the
 * expander the matching half stays off and the log says which.
 *
 * @return ESP_OK, ESP_ERR_NOT_SUPPORTED when the feature is off, or the
 *         reason the task could not be created.
 */
esp_err_t power_manager_start(void);

/** The filtered pack voltage in millivolts; 0 when there is no reading (the
 *  feature is off, no carrier, no pack, or the ADC failed). Any task. */
uint32_t power_manager_pack_mv(void);

#ifdef __cplusplus
}
#endif
