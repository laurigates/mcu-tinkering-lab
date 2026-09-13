/**
 * @file ota_manager.h
 * @brief OTA update manager for the single-board unified robocar
 *
 * Manages firmware updates by polling the web-flasher's published manifest
 * (see ota_manager.c and issue #539) with a periodic timer and MQTT
 * check-now nudges.
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initialize OTA manager
 *
 * Starts the periodic manifest-poll task (see ota_manager.c), subscribes to
 * the MQTT OTA notification topic, and starts the rollback stability timer.
 *
 * Requires WiFi and MQTT to be initialized first.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_manager_init(void);

/**
 * @brief Manually trigger an update check
 *
 * Wakes the OTA task immediately instead of waiting for the periodic
 * timeout. Called from the MQTT check-now nudge.
 *
 * @return ESP_OK if the check was queued, error code otherwise
 */
esp_err_t ota_manager_check_update(void);

/**
 * @brief Get current firmware version string
 *
 * Returns the version from esp_app_get_description(), which is embedded
 * at compile time from the project version.
 *
 * @return Null-terminated version string (e.g., "0.1.0")
 */
const char *ota_manager_get_version(void);

/**
 * @brief Mark current firmware as valid after stable boot
 *
 * Should be called after the device has been running stably for
 * OTA_STABILITY_TIMEOUT_MS. Cancels automatic rollback to the
 * previous firmware.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_manager_confirm_valid(void);

#endif  // OTA_MANAGER_H
