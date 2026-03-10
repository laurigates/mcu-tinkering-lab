/**
 * @file wifi_manager.h
 * @brief WiFi connection manager for IT Troubleshooter (Phase 2).
 *
 * Connects to the pre-configured hotspot from credentials.h with exponential
 * backoff retry. LED feedback is driven by the caller via status_led.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize WiFi and begin connecting to the given SSID.
 *
 * Non-blocking: returns immediately after starting the connection attempt.
 * Poll wifi_manager_is_connected() to wait for completion.
 *
 * @param ssid     WiFi network name (null-terminated, max 32 bytes).
 * @param password WiFi password (null-terminated, max 64 bytes).
 * @return ESP_OK on successful init, error code on failure.
 */
esp_err_t wifi_manager_init(const char *ssid, const char *password);

/**
 * @brief Return true if WiFi is currently connected and has an IP address.
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Deinitialize WiFi and release resources.
 */
void wifi_manager_deinit(void);
