/**
 * @file wifi_manager.h
 * @brief WiFi connectivity manager for Heltec WiFi LoRa 32 V1 main controller
 *
 * Provides robust WiFi connectivity with credential management, connection
 * retry logic, network monitoring, and OTA update support.
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

// WiFi configuration constants
#define WIFI_MAXIMUM_RETRY 10               // Maximum number of connection attempts
#define WIFI_RETRY_BASE_DELAY_MS 1000       // Initial retry delay (exponential backoff)
#define WIFI_MAX_RETRY_DELAY_MS 30000       // Maximum retry delay
#define WIFI_SSID_MAX_LEN 32                // Maximum SSID length
#define WIFI_PASSWORD_MAX_LEN 64            // Maximum password length
#define WIFI_HOSTNAME "RoboCar-Controller"  // Device hostname

// WiFi power management modes for robotics
typedef enum {
    WIFI_POWER_MAX_PERFORMANCE = 0,  // No power saving, lowest latency
    WIFI_POWER_BALANCED = 1,         // Balanced power/performance
    WIFI_POWER_MIN_MODEM = 2,        // Minimum modem power saving
    WIFI_POWER_MAX_MODEM = 3         // Maximum modem power saving
} wifi_power_mode_t;

// WiFi connection state
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_CONNECTION_FAILED,
    WIFI_STATE_RECONNECTING
} wifi_state_t;

// WiFi connection information
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char ip_address[16];
    int8_t rssi;
    uint8_t channel;
    bool is_connected;
    wifi_state_t state;
    uint32_t connection_attempts;
    uint32_t successful_connections;
    uint32_t total_uptime_seconds;
    uint32_t last_disconnect_reason;
} wifi_info_t;

// WiFi credentials structure
typedef struct {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
} wifi_credentials_t;

// WiFi event callback type
typedef void (*wifi_event_callback_t)(wifi_state_t state, void *data);

/**
 * @brief Initialize WiFi manager
 *
 * Initializes NVS, event loop, and WiFi subsystem.
 * Must be called before any other WiFi functions.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Deinitialize WiFi manager
 *
 * Stops WiFi and releases all resources.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_deinit(void);

/**
 * @brief Connect to WiFi network
 *
 * Attempts to connect using provided credentials. If empty strings
 * are provided, uses stored credentials from NVS.
 *
 * @param ssid Network SSID (empty string to use stored)
 * @param password Network password (empty string to use stored)
 * @return ESP_OK on successful connection start, error code otherwise
 */
esp_err_t wifi_manager_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from current WiFi network
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_disconnect(void);

/**
 * @brief Check if connected to WiFi
 *
 * @return true if connected, false otherwise
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Get current WiFi connection information
 *
 * @param info Pointer to structure to fill with connection info
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if info is NULL
 */
esp_err_t wifi_manager_get_info(wifi_info_t *info);

/**
 * @brief Save WiFi credentials to NVS
 *
 * Stores credentials for automatic reconnection.
 *
 * @param credentials Pointer to credentials structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_save_credentials(const wifi_credentials_t *credentials);

/**
 * @brief Load WiFi credentials from NVS
 *
 * @param credentials Pointer to structure to fill with loaded credentials
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if no stored credentials
 */
esp_err_t wifi_manager_load_credentials(wifi_credentials_t *credentials);

/**
 * @brief Clear stored WiFi credentials
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_clear_credentials(void);

/**
 * @brief Set WiFi power management mode
 *
 * For robotics applications, WIFI_POWER_MAX_PERFORMANCE is recommended
 * for lowest latency, while WIFI_POWER_BALANCED provides good compromise.
 *
 * @param mode Power management mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_set_power_mode(wifi_power_mode_t mode);

/**
 * @brief Register callback for WiFi state changes
 *
 * @param callback Function to call on state changes
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_register_callback(wifi_event_callback_t callback, void *user_data);

/**
 * @brief Start WiFi scan for available networks
 *
 * @return ESP_OK on scan start, error code otherwise
 */
esp_err_t wifi_manager_start_scan(void);

/**
 * @brief Get scan results
 *
 * @param ap_records Array to store AP records
 * @param max_records Maximum number of records to retrieve
 * @param actual_count Pointer to store actual number of APs found
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_get_scan_results(wifi_ap_record_t *ap_records, uint16_t max_records,
                                        uint16_t *actual_count);

/**
 * @brief Enable automatic reconnection
 *
 * When enabled, automatically attempts to reconnect on disconnection.
 *
 * @param enable true to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_set_auto_reconnect(bool enable);

/**
 * @brief Get reconnection status
 *
 * @return true if auto-reconnect is enabled
 */
bool wifi_manager_get_auto_reconnect(void);

/**
 * @brief Force immediate reconnection attempt
 *
 * Useful for manual retry after connection failure.
 *
 * @return ESP_OK if reconnection started, error code otherwise
 */
esp_err_t wifi_manager_force_reconnect(void);

/**
 * @brief Get WiFi MAC address
 *
 * @param mac Buffer to store MAC address (6 bytes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_get_mac_address(uint8_t *mac);

/**
 * @brief Get current WiFi state
 *
 * @return Current WiFi state
 */
wifi_state_t wifi_manager_get_state(void);

/**
 * @brief Get WiFi state as string
 *
 * @param state WiFi state
 * @return String representation of state
 */
const char *wifi_manager_state_to_string(wifi_state_t state);

#endif  // WIFI_MANAGER_H
