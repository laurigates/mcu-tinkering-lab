/**
 * @file wifi_manager.c
 * @brief WiFi connectivity manager implementation for Heltec WiFi LoRa 32 V1
 *
 * Implements robust WiFi connectivity with automatic reconnection,
 * credential management, and power optimization for robotics applications.
 */

#include "wifi_manager.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// Module configuration
#define TAG "WiFi_Manager"
#define NVS_NAMESPACE "wifi_config"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASSWORD "password"

// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_SCANNING_BIT BIT2

// Internal state management
typedef struct {
    EventGroupHandle_t event_group;
    SemaphoreHandle_t mutex;
    esp_timer_handle_t reconnect_timer;
    wifi_info_t info;
    wifi_credentials_t current_credentials;
    wifi_event_callback_t user_callback;
    void* user_callback_data;
    bool initialized;
    bool auto_reconnect;
    uint32_t retry_count;
    uint32_t retry_delay_ms;
    uint32_t connect_start_time;
    wifi_power_mode_t power_mode;
} wifi_manager_context_t;

// Static instance
static wifi_manager_context_t g_wifi_context = {0};

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data);
static void reconnect_timer_callback(void* arg);
static void update_connection_info(void);
static void notify_state_change(wifi_state_t new_state);
static uint32_t calculate_backoff_delay(uint32_t retry_count);

/**
 * Initialize WiFi manager
 */
esp_err_t wifi_manager_init(void) {
    esp_err_t ret;

    if (g_wifi_context.initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi manager");

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create mutex for thread safety
    g_wifi_context.mutex = xSemaphoreCreateMutex();
    if (g_wifi_context.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Create event group
    g_wifi_context.event_group = xEventGroupCreate();
    if (g_wifi_context.event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        vSemaphoreDelete(g_wifi_context.mutex);
        return ESP_ERR_NO_MEM;
    }

    // Initialize default netif
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Create default WiFi station
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA");
        ret = ESP_FAIL;
        goto cleanup;
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Register event handlers
    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              NULL,
                                              NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler");
        goto cleanup;
    }

    ret = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &ip_event_handler,
                                              NULL,
                                              NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler");
        goto cleanup;
    }

    // Create reconnection timer
    const esp_timer_create_args_t timer_args = {
        .callback = &reconnect_timer_callback,
        .name = "wifi_reconnect"
    };
    ret = esp_timer_create(&timer_args, &g_wifi_context.reconnect_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create reconnect timer");
        goto cleanup;
    }

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Set hostname for easier identification
    esp_netif_set_hostname(sta_netif, WIFI_HOSTNAME);

    // Initialize context
    g_wifi_context.initialized = true;
    g_wifi_context.auto_reconnect = true;
    g_wifi_context.info.state = WIFI_STATE_DISCONNECTED;
    g_wifi_context.power_mode = WIFI_POWER_BALANCED;

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi manager initialized successfully");
    return ESP_OK;

cleanup:
    if (g_wifi_context.event_group) {
        vEventGroupDelete(g_wifi_context.event_group);
        g_wifi_context.event_group = NULL;
    }
    if (g_wifi_context.mutex) {
        vSemaphoreDelete(g_wifi_context.mutex);
        g_wifi_context.mutex = NULL;
    }
    return ret;
}

/**
 * Deinitialize WiFi manager
 */
esp_err_t wifi_manager_deinit(void) {
    if (!g_wifi_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi manager");

    // Stop reconnection timer
    if (g_wifi_context.reconnect_timer) {
        esp_timer_stop(g_wifi_context.reconnect_timer);
        esp_timer_delete(g_wifi_context.reconnect_timer);
        g_wifi_context.reconnect_timer = NULL;
    }

    // Stop and deinit WiFi
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    // Cleanup resources
    if (g_wifi_context.event_group) {
        vEventGroupDelete(g_wifi_context.event_group);
        g_wifi_context.event_group = NULL;
    }

    if (g_wifi_context.mutex) {
        vSemaphoreDelete(g_wifi_context.mutex);
        g_wifi_context.mutex = NULL;
    }

    g_wifi_context.initialized = false;

    ESP_LOGI(TAG, "WiFi manager deinitialized");
    return ESP_OK;
}

/**
 * Connect to WiFi network
 */
esp_err_t wifi_manager_connect(const char* ssid, const char* password) {
    if (!g_wifi_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);

    // If empty credentials, try to load from NVS
    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGI(TAG, "No credentials provided, loading from NVS");
        wifi_credentials_t stored_creds;
        if (wifi_manager_load_credentials(&stored_creds) == ESP_OK) {
            ssid = stored_creds.ssid;
            password = stored_creds.password;
            memcpy(&g_wifi_context.current_credentials, &stored_creds, sizeof(wifi_credentials_t));
        } else {
            ESP_LOGW(TAG, "No stored credentials found");
            xSemaphoreGive(g_wifi_context.mutex);
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        // Validate SSID and password lengths
        if (strlen(ssid) > WIFI_SSID_MAX_LEN) {
            ESP_LOGE(TAG, "SSID too long (max %d characters)", WIFI_SSID_MAX_LEN);
            xSemaphoreGive(g_wifi_context.mutex);
            return ESP_ERR_INVALID_ARG;
        }
        if (password && strlen(password) > WIFI_PASSWORD_MAX_LEN) {
            ESP_LOGE(TAG, "Password too long (max %d characters)", WIFI_PASSWORD_MAX_LEN);
            xSemaphoreGive(g_wifi_context.mutex);
            return ESP_ERR_INVALID_ARG;
        }

        // Store provided credentials
        strncpy(g_wifi_context.current_credentials.ssid, ssid, WIFI_SSID_MAX_LEN);
        g_wifi_context.current_credentials.ssid[WIFI_SSID_MAX_LEN] = '\0';
        if (password) {
            strncpy(g_wifi_context.current_credentials.password, password, WIFI_PASSWORD_MAX_LEN);
            g_wifi_context.current_credentials.password[WIFI_PASSWORD_MAX_LEN] = '\0';
        } else {
            g_wifi_context.current_credentials.password[0] = '\0';
        }
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", g_wifi_context.current_credentials.ssid);

    // Configure WiFi
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, g_wifi_context.current_credentials.ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, g_wifi_context.current_credentials.password,
            sizeof(wifi_config.sta.password) - 1);

    // Security settings
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    // Set configuration
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Reset retry counter
    g_wifi_context.retry_count = 0;
    g_wifi_context.retry_delay_ms = WIFI_RETRY_BASE_DELAY_MS;
    g_wifi_context.connect_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Update state
    g_wifi_context.info.state = WIFI_STATE_CONNECTING;
    notify_state_change(WIFI_STATE_CONNECTING);

    xSemaphoreGive(g_wifi_context.mutex);

    // Start connection
    return esp_wifi_connect();
}

/**
 * Disconnect from WiFi
 */
esp_err_t wifi_manager_disconnect(void) {
    if (!g_wifi_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disconnecting from WiFi");

    // Stop auto-reconnect temporarily
    bool prev_auto_reconnect = g_wifi_context.auto_reconnect;
    g_wifi_context.auto_reconnect = false;

    esp_err_t ret = esp_wifi_disconnect();

    // Restore auto-reconnect setting
    g_wifi_context.auto_reconnect = prev_auto_reconnect;

    return ret;
}

/**
 * Check if connected
 */
bool wifi_manager_is_connected(void) {
    return g_wifi_context.info.is_connected;
}

/**
 * Get connection info
 */
esp_err_t wifi_manager_get_info(wifi_info_t* info) {
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);
    memcpy(info, &g_wifi_context.info, sizeof(wifi_info_t));
    xSemaphoreGive(g_wifi_context.mutex);

    return ESP_OK;
}

/**
 * Save credentials to NVS
 */
esp_err_t wifi_manager_save_credentials(const wifi_credentials_t* credentials) {
    if (!credentials) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate SSID and password lengths
    if (strlen(credentials->ssid) == 0 || strlen(credentials->ssid) > WIFI_SSID_MAX_LEN) {
        ESP_LOGE(TAG, "Invalid SSID length (must be 1-%d characters)", WIFI_SSID_MAX_LEN);
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(credentials->password) > WIFI_PASSWORD_MAX_LEN) {
        ESP_LOGE(TAG, "Password too long (max %d characters)", WIFI_PASSWORD_MAX_LEN);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save SSID
    ret = nvs_set_str(nvs_handle, NVS_KEY_SSID, credentials->ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Save password
    ret = nvs_set_str(nvs_handle, NVS_KEY_PASSWORD, credentials->password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Credentials saved successfully");
    }

    nvs_close(nvs_handle);
    return ret;
}

/**
 * Load credentials from NVS
 */
esp_err_t wifi_manager_load_credentials(wifi_credentials_t* credentials) {
    if (!credentials) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load SSID
    size_t ssid_len = sizeof(credentials->ssid);
    ret = nvs_get_str(nvs_handle, NVS_KEY_SSID, credentials->ssid, &ssid_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load SSID: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Load password
    size_t pass_len = sizeof(credentials->password);
    ret = nvs_get_str(nvs_handle, NVS_KEY_PASSWORD, credentials->password, &pass_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load password: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ESP_LOGI(TAG, "Credentials loaded successfully for SSID: %s", credentials->ssid);

    nvs_close(nvs_handle);
    return ESP_OK;
}

/**
 * Clear stored credentials
 */
esp_err_t wifi_manager_clear_credentials(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Erase all keys
    ret = nvs_erase_all(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Credentials cleared successfully");
    }

    nvs_close(nvs_handle);
    return ret;
}

/**
 * Set WiFi power mode
 */
esp_err_t wifi_manager_set_power_mode(wifi_power_mode_t mode) {
    wifi_ps_type_t ps_type;

    switch (mode) {
        case WIFI_POWER_MAX_PERFORMANCE:
            ps_type = WIFI_PS_NONE;
            break;
        case WIFI_POWER_BALANCED:
            ps_type = WIFI_PS_MIN_MODEM;
            break;
        case WIFI_POWER_MIN_MODEM:
            ps_type = WIFI_PS_MIN_MODEM;
            break;
        case WIFI_POWER_MAX_MODEM:
            ps_type = WIFI_PS_MAX_MODEM;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_wifi_set_ps(ps_type);
    if (ret == ESP_OK) {
        g_wifi_context.power_mode = mode;
        ESP_LOGI(TAG, "Power mode set to: %d", mode);
    } else {
        ESP_LOGE(TAG, "Failed to set power mode: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Register event callback
 */
esp_err_t wifi_manager_register_callback(wifi_event_callback_t callback, void* user_data) {
    xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);
    g_wifi_context.user_callback = callback;
    g_wifi_context.user_callback_data = user_data;
    xSemaphoreGive(g_wifi_context.mutex);

    ESP_LOGI(TAG, "Callback registered");
    return ESP_OK;
}

/**
 * Start WiFi scan
 */
esp_err_t wifi_manager_start_scan(void) {
    if (!g_wifi_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting WiFi scan");

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300
    };

    return esp_wifi_scan_start(&scan_config, false);
}

/**
 * Get scan results
 */
esp_err_t wifi_manager_get_scan_results(wifi_ap_record_t* ap_records,
                                        uint16_t max_records,
                                        uint16_t* actual_count) {
    if (!ap_records || !actual_count) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t ap_count = max_records;
    esp_err_t ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (ret == ESP_OK) {
        *actual_count = ap_count;
        ESP_LOGI(TAG, "Found %d access points", ap_count);
    }

    return ret;
}

/**
 * Set auto-reconnect
 */
esp_err_t wifi_manager_set_auto_reconnect(bool enable) {
    g_wifi_context.auto_reconnect = enable;
    ESP_LOGI(TAG, "Auto-reconnect %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * Get auto-reconnect status
 */
bool wifi_manager_get_auto_reconnect(void) {
    return g_wifi_context.auto_reconnect;
}

/**
 * Force reconnect
 */
esp_err_t wifi_manager_force_reconnect(void) {
    if (!g_wifi_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Forcing reconnection");

    // Reset retry count for immediate attempt
    g_wifi_context.retry_count = 0;
    g_wifi_context.retry_delay_ms = WIFI_RETRY_BASE_DELAY_MS;

    return wifi_manager_connect(NULL, NULL);
}

/**
 * Get MAC address
 */
esp_err_t wifi_manager_get_mac_address(uint8_t* mac) {
    if (!mac) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_wifi_get_mac(WIFI_IF_STA, mac);
}

/**
 * Get current state
 */
wifi_state_t wifi_manager_get_state(void) {
    return g_wifi_context.info.state;
}

/**
 * Convert state to string
 */
const char* wifi_manager_state_to_string(wifi_state_t state) {
    switch (state) {
        case WIFI_STATE_DISCONNECTED:
            return "DISCONNECTED";
        case WIFI_STATE_CONNECTING:
            return "CONNECTING";
        case WIFI_STATE_CONNECTED:
            return "CONNECTED";
        case WIFI_STATE_CONNECTION_FAILED:
            return "CONNECTION_FAILED";
        case WIFI_STATE_RECONNECTING:
            return "RECONNECTING";
        default:
            return "UNKNOWN";
    }
}

/**
 * WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi started");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to AP");
            g_wifi_context.info.connection_attempts++;
            g_wifi_context.info.successful_connections++;
            update_connection_info();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
            ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);

            g_wifi_context.info.is_connected = false;
            g_wifi_context.info.last_disconnect_reason = event->reason;

            if (g_wifi_context.auto_reconnect && g_wifi_context.retry_count < WIFI_MAXIMUM_RETRY) {
                g_wifi_context.retry_count++;

                xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);
                g_wifi_context.info.state = WIFI_STATE_RECONNECTING;
                xSemaphoreGive(g_wifi_context.mutex);

                notify_state_change(WIFI_STATE_RECONNECTING);

                uint32_t delay = calculate_backoff_delay(g_wifi_context.retry_count);
                ESP_LOGI(TAG, "Reconnection attempt %d/%d in %d ms",
                        g_wifi_context.retry_count, WIFI_MAXIMUM_RETRY, delay);

                esp_timer_stop(g_wifi_context.reconnect_timer);
                esp_timer_start_once(g_wifi_context.reconnect_timer, delay * 1000);
            } else {
                xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);
                g_wifi_context.info.state = WIFI_STATE_CONNECTION_FAILED;
                xSemaphoreGive(g_wifi_context.mutex);

                notify_state_change(WIFI_STATE_CONNECTION_FAILED);
                xEventGroupSetBits(g_wifi_context.event_group, WIFI_FAIL_BIT);
            }
            break;
        }

        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "WiFi scan completed");
            xEventGroupSetBits(g_wifi_context.event_group, WIFI_SCANNING_BIT);
            break;

        default:
            ESP_LOGD(TAG, "Unhandled WiFi event: %d", event_id);
            break;
    }
}

/**
 * IP event handler
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;

        xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);

        snprintf(g_wifi_context.info.ip_address, sizeof(g_wifi_context.info.ip_address),
                 IPSTR, IP2STR(&event->ip_info.ip));
        g_wifi_context.info.is_connected = true;
        g_wifi_context.info.state = WIFI_STATE_CONNECTED;
        g_wifi_context.retry_count = 0;

        xSemaphoreGive(g_wifi_context.mutex);

        ESP_LOGI(TAG, "Got IP address: %s", g_wifi_context.info.ip_address);

        notify_state_change(WIFI_STATE_CONNECTED);
        xEventGroupSetBits(g_wifi_context.event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * Reconnection timer callback
 */
static void reconnect_timer_callback(void* arg) {
    ESP_LOGI(TAG, "Reconnection timer triggered");
    esp_wifi_connect();
}

/**
 * Calculate exponential backoff delay
 */
static uint32_t calculate_backoff_delay(uint32_t retry_count) {
    uint32_t delay = WIFI_RETRY_BASE_DELAY_MS * (1 << (retry_count - 1));
    if (delay > WIFI_MAX_RETRY_DELAY_MS) {
        delay = WIFI_MAX_RETRY_DELAY_MS;
    }
    return delay;
}

/**
 * Update connection information
 */
static void update_connection_info(void) {
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        xSemaphoreTake(g_wifi_context.mutex, portMAX_DELAY);

        memcpy(g_wifi_context.info.ssid, ap_info.ssid, sizeof(g_wifi_context.info.ssid));
        g_wifi_context.info.rssi = ap_info.rssi;
        g_wifi_context.info.channel = ap_info.primary;

        xSemaphoreGive(g_wifi_context.mutex);
    }
}

/**
 * Notify state change
 */
static void notify_state_change(wifi_state_t new_state) {
    if (g_wifi_context.user_callback) {
        g_wifi_context.user_callback(new_state, g_wifi_context.user_callback_data);
    }
}