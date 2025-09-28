#include "wifi_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_manager";

// WiFi event bits
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define WIFI_DISCONNECTED_BIT BIT2

// Static variables
static EventGroupHandle_t wifi_event_group;
static wifi_state_t current_state = WIFI_STATE_DISCONNECTED;
static int retry_count = 0;
static bool auto_reconnect_enabled = true;
static const int MAX_RETRY_ATTEMPTS = 5;
static const int RETRY_DELAY_MS = 5000;

// Current WiFi configuration
static wifi_manager_config_t current_wifi_config = {0};
static esp_netif_t* sta_netif = NULL;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
        esp_wifi_connect();
        current_state = WIFI_STATE_CONNECTING;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected from AP");
        current_state = WIFI_STATE_DISCONNECTED;
        xEventGroupSetBits(wifi_event_group, WIFI_DISCONNECTED_BIT);
        
        if (auto_reconnect_enabled && retry_count < MAX_RETRY_ATTEMPTS) {
            ESP_LOGI(TAG, "Retrying WiFi connection (%d/%d)", retry_count + 1, MAX_RETRY_ATTEMPTS);
            vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
            esp_wifi_connect();
            retry_count++;
            current_state = WIFI_STATE_CONNECTING;
        } else {
            ESP_LOGE(TAG, "WiFi connection failed after %d attempts", MAX_RETRY_ATTEMPTS);
            current_state = WIFI_STATE_ERROR;
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        current_state = WIFI_STATE_CONNECTED;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi manager");
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase required, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create WiFi event group
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_FAIL;
    }
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create default WiFi station
    sta_netif = esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Configure WiFi power save mode for robotics use
    wifi_manager_set_power_save_mode();
    
    ESP_LOGI(TAG, "WiFi manager initialized successfully");
    return ESP_OK;
}

esp_err_t wifi_manager_connect(const char* ssid, const char* password) {
    if (ssid == NULL || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID provided");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (password == NULL) {
        password = "";  // Open network
    }
    
    ESP_LOGI(TAG, "Connecting to WiFi network: %s", ssid);
    
    // Store configuration
    strncpy(current_wifi_config.ssid, ssid, sizeof(current_wifi_config.ssid) - 1);
    strncpy(current_wifi_config.password, password, sizeof(current_wifi_config.password) - 1);
    current_wifi_config.auto_connect = true;
    current_wifi_config.max_retry_attempts = MAX_RETRY_ATTEMPTS;
    current_wifi_config.retry_delay_ms = RETRY_DELAY_MS;
    
    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    
    // Clear previous event bits
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_DISCONNECTED_BIT);
    
    // Set configuration and start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    retry_count = 0;
    current_state = WIFI_STATE_CONNECTING;
    
    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to WiFi network: %s", ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to WiFi network: %s", ssid);
        return ESP_FAIL;
    }
    
    ESP_LOGE(TAG, "Unexpected WiFi connection result");
    return ESP_FAIL;
}

esp_err_t wifi_manager_disconnect(void) {
    ESP_LOGI(TAG, "Disconnecting from WiFi");
    
    // Disable auto-reconnect temporarily
    auto_reconnect_enabled = false;
    
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Stop WiFi
    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    current_state = WIFI_STATE_DISCONNECTED;
    ESP_LOGI(TAG, "WiFi disconnected successfully");
    
    // Re-enable auto-reconnect
    auto_reconnect_enabled = true;
    
    return ESP_OK;
}

wifi_state_t wifi_manager_get_state(void) {
    return current_state;
}

bool wifi_manager_get_connection_info(char* ip_addr, int* rssi) {
    if (current_state != WIFI_STATE_CONNECTED) {
        return false;
    }
    
    if (ip_addr != NULL) {
        esp_netif_ip_info_t ip_info;
        esp_err_t ret = esp_netif_get_ip_info(sta_netif, &ip_info);
        if (ret == ESP_OK) {
            sprintf(ip_addr, IPSTR, IP2STR(&ip_info.ip));
        } else {
            strcpy(ip_addr, "0.0.0.0");
        }
    }
    
    if (rssi != NULL) {
        wifi_ap_record_t ap_info;
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
        if (ret == ESP_OK) {
            *rssi = ap_info.rssi;
        } else {
            *rssi = 0;
        }
    }
    
    return true;
}

void wifi_manager_set_auto_reconnect(bool enable) {
    auto_reconnect_enabled = enable;
    ESP_LOGI(TAG, "Auto-reconnect %s", enable ? "enabled" : "disabled");
}

esp_err_t wifi_manager_set_power_save_mode(void) {
    // Set power save mode suitable for robotics applications
    // WIFI_PS_MIN_MODEM provides a balance between power consumption and responsiveness
    esp_err_t ret = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi power save mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WiFi power save mode configured for robotics use");
    return ESP_OK;
}

bool wifi_manager_is_ota_ready(void) {
    if (current_state != WIFI_STATE_CONNECTED) {
        return false;
    }
    
    // Check signal strength (should be better than -70 dBm for reliable OTA)
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Cannot get AP info for OTA readiness check");
        return false;
    }
    
    if (ap_info.rssi < -70) {
        ESP_LOGW(TAG, "WiFi signal too weak for OTA (RSSI: %d dBm)", ap_info.rssi);
        return false;
    }
    
    ESP_LOGI(TAG, "WiFi ready for OTA (RSSI: %d dBm)", ap_info.rssi);
    return true;
}