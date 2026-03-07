/**
 * @file ota_manager.h
 * @brief OTA update manager for ESP32-CAM robocar
 *
 * Manages firmware updates via esp_ghota (GitHub OTA) with periodic polling
 * and MQTT push notifications. Orchestrates updates for both camera and
 * main controller boards.
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initialize OTA manager
 *
 * Sets up esp_ghota for periodic GitHub release checking, subscribes to
 * MQTT OTA notification topic, and registers event handlers for update
 * progress tracking.
 *
 * Requires WiFi and MQTT to be initialized first.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_manager_init(void);

/**
 * @brief Manually trigger an update check
 *
 * Forces an immediate check against GitHub Releases, bypassing the
 * periodic timer. Useful when an MQTT notification is received.
 *
 * @return ESP_OK if check started, error code otherwise
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

#endif // OTA_MANAGER_H
