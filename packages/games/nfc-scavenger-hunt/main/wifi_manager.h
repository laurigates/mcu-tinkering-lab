#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

/**
 * Initialize WiFi in STA mode and connect to the configured network.
 * Credentials are read from secrets.h at compile time.
 * Blocks until connected or max retries exceeded.
 */
esp_err_t wifi_manager_init(void);

/**
 * Check if WiFi is currently connected.
 */
bool wifi_manager_is_connected(void);

/**
 * Wait for WiFi connection with timeout.
 *
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return ESP_OK if connected, ESP_ERR_TIMEOUT if timed out.
 */
esp_err_t wifi_manager_wait_connected(uint32_t timeout_ms);

#endif  // WIFI_MANAGER_H
