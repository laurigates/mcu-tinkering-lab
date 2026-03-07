/**
 * @file ota_handler.h
 * @brief OTA update handler for robocar main controller
 *
 * Handles OTA firmware updates triggered by I2C commands from the camera
 * module. Manages WiFi-on-demand for downloading firmware from GitHub
 * Releases via esp_https_ota.
 */

#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include "esp_err.h"
#include "i2c_protocol.h"

/**
 * @brief Initialize OTA handler
 *
 * Sets up rollback confirmation timer. I2C OTA command handling is
 * already integrated into i2c_slave.c via handle_ota_commands().
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_handler_init(void);

/**
 * @brief Execute OTA firmware update
 *
 * Called from i2c_slave.c when CMD_TYPE_BEGIN_OTA is received.
 * Initializes WiFi if needed, constructs the download URL from the
 * release tag, and performs the OTA update via esp_https_ota.
 *
 * @param tag Release tag (e.g., "v0.2.0") received via I2C
 * @param hash First 4 bytes of SHA256 for verification
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_handler_begin_update(const char *tag, const uint8_t *hash);

/**
 * @brief Get current OTA progress
 *
 * @return Progress percentage 0-100, or 0 if not updating
 */
uint8_t ota_handler_get_progress(void);

/**
 * @brief Get current OTA status
 *
 * @return Current ota_status_t value
 */
ota_status_t ota_handler_get_status(void);

/**
 * @brief Get last OTA error code
 *
 * @return Error code from the most recent OTA attempt, 0 if no error
 */
uint8_t ota_handler_get_error_code(void);

/**
 * @brief Mark current firmware as valid after stable boot
 *
 * Cancels automatic rollback to previous firmware version.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_handler_confirm_valid(void);

#endif // OTA_HANDLER_H
