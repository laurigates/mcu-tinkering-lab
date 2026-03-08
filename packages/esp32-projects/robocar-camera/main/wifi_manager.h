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
 * @brief Connect to WiFi network
 * @param ssid WiFi network name
 * @param password WiFi password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_connect(const char *ssid, const char *password);

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
