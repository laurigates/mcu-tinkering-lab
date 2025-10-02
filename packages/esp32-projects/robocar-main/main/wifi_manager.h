#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi manager for main controller OTA functionality
 * 
 * This module provides WiFi connectivity management for the main controller,
 * enabling OTA updates while maintaining robotics-specific power management.
 */

typedef struct {
    char ssid[32];
    char password[64];
    bool auto_connect;
    uint8_t max_retry_attempts;
    uint32_t retry_delay_ms;
} wifi_manager_config_t;

typedef enum {
    WIFI_STATE_DISCONNECTED,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR
} wifi_state_t;

/**
 * @brief Initialize WiFi manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Start WiFi connection with provided credentials
 * 
 * @param ssid WiFi network SSID
 * @param password WiFi network password  
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_connect(const char* ssid, const char* password);

/**
 * @brief Disconnect from WiFi network
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_disconnect(void);

/**
 * @brief Get current WiFi connection state
 * 
 * @return Current WiFi state
 */
wifi_state_t wifi_manager_get_state(void);

/**
 * @brief Get WiFi connection status information
 * 
 * @param ip_addr Output buffer for IP address (16 bytes minimum)
 * @param rssi Output for RSSI value
 * @return true if connected and info retrieved successfully
 */
bool wifi_manager_get_connection_info(char* ip_addr, int* rssi);

/**
 * @brief Enable/disable automatic reconnection
 * 
 * @param enable true to enable auto-reconnect, false to disable
 */
void wifi_manager_set_auto_reconnect(bool enable);

/**
 * @brief Set WiFi power save mode for robotics use
 * 
 * Configures WiFi power management suitable for robotics applications
 * (balance between power consumption and responsiveness)
 */
esp_err_t wifi_manager_set_power_save_mode(void);

/**
 * @brief Check if WiFi is ready for OTA operations
 * 
 * @return true if WiFi is connected and stable for OTA
 */
bool wifi_manager_is_ota_ready(void);

#ifdef __cplusplus
}
#endif