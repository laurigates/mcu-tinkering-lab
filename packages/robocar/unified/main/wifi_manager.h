/**
 * @file wifi_manager.h
 * @brief WiFi connection management
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_wifi.h"

/**
 * @brief Initialize WiFi
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_init(void);

/**
 * @brief Start joining a network, without waiting for it (issue #846).
 *
 * Returns as soon as the attempt is under way. The link is reported by
 * wifi_is_connected(); wifi_connect_failed() says when the fast retries for
 * this attempt are spent (reconnection continues in the background with
 * backoff). Replaces any attempt or association in progress.
 *
 * @return ESP_OK when the attempt started; the driver's error otherwise, which
 *         is logged and never aborts the boot.
 */
esp_err_t wifi_connect(const char *ssid, const char *password);

/** True once the fast retries of the current wifi_connect() attempt are spent
 *  without a link. Cleared by the next wifi_connect() and by a link-up. */
bool wifi_connect_failed(void);

/**
 * @brief Check if WiFi is connected
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Disconnect from WiFi
 */
void wifi_disconnect(void);

/**
 * @brief Deinitialize WiFi
 */
void wifi_manager_deinit(void);

#endif  // WIFI_MANAGER_H
