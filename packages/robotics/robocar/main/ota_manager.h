/**
 * @file ota_manager.h
 * @brief OTA update manager for the single-board robocar
 *
 * Manages firmware updates by polling the web-flasher's published manifest
 * (see ota_manager.c and issue #539) with a periodic timer and MQTT
 * check-now nudges.
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <stddef.h>

#include "esp_err.h"

/**
 * Where the pre-install sequence is. A newer version is not installed at
 * once: the robot waits for a quiet moment, says what the update changes and
 * OTA_ANNOUNCE_PUN, keeps a postpone window open, parks on open floor
 * (ota_park.h) and holds still (reactive_controller_set_hold()) for the
 * download and the reboot.
 */
typedef enum {
    OTA_PHASE_IDLE = 0,
    OTA_PHASE_WAITING_QUIET,
    OTA_PHASE_ANNOUNCING,
    OTA_PHASE_WINDOW,
    OTA_PHASE_PARKING,
    OTA_PHASE_INSTALLING,
} ota_phase_t;

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
 * @brief Get the git commit SHA this firmware was built from
 *
 * Resolved at CMake configure time (see the project CMakeLists.txt) and
 * compared against the OTA manifest's "buildSha" (issue #627).
 *
 * @return 40 lowercase hex digits, the same with "-dirty" appended for a tree
 *         with uncommitted changes, or "unknown" when git was unavailable
 */
const char *ota_manager_get_build_sha(void);

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

/**
 * @brief Postpone the announced update to the next check.
 *
 * Accepted only while the postpone window is open (OTA_PHASE_WINDOW). The
 * console's `ota postpone`, also reachable over MQTT.
 *
 * @return ESP_OK if postponed, ESP_ERR_INVALID_STATE when no window is open
 */
esp_err_t ota_manager_postpone(void);

/** The pre-install sequence's current phase. Safe from any task. */
ota_phase_t ota_manager_phase(void);

/** Human-readable phase, for the console. */
const char *ota_manager_phase_name(ota_phase_t phase);

/** Copy the version the sequence is (or was last) for into @p out. */
void ota_manager_pending_version(char *out, size_t out_len);

#endif  // OTA_MANAGER_H
