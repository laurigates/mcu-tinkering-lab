#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "wifitest";

#define WIFI_SSID "esp32s3-test"
#define WIFI_PASS "testpass1234"
#define WIFI_CHANNEL 6
#define MAX_CONNECTIONS 4

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station connected: AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station disconnected: AID=%d", event->aid);
    }
}

void app_main(void)
{
    // Redirect stdout/stderr to USB-Serial-JTAG so `just monitor` works
    usb_serial_jtag_driver_config_t usj_cfg = {.tx_buffer_size = 4096, .rx_buffer_size = 256};
    usb_serial_jtag_driver_install(&usj_cfg);
    usb_serial_jtag_vfs_use_driver();

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "=== ESP32-S3 WiFi AP Test === v%s (%s %s)", app->version, app->date, app->time);

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Network stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // WiFi init
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    // Configure AP
    wifi_config_t wifi_config = {
        .ap =
            {
                .ssid = WIFI_SSID,
                .ssid_len = strlen(WIFI_SSID),
                .channel = WIFI_CHANNEL,
                .password = WIFI_PASS,
                .max_connection = MAX_CONNECTIONS,
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // Set country code for proper regional TX power
    wifi_country_t country = {
        .cc = "FI", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_ERROR_CHECK(esp_wifi_start());

    // Force HT20 bandwidth (some clients miss HT40 APs)
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));

    // Max TX power: 78 = 19.5 dBm (maximum for ESP32-S3)
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));

    int8_t tx_power = 0;
    esp_wifi_get_max_tx_power(&tx_power);
    ESP_LOGI(TAG, "AP started: SSID=%s channel=%d tx_power=%.1fdBm bw=HT20 country=FI", WIFI_SSID,
             WIFI_CHANNEL, tx_power * 0.25);
    ESP_LOGI(TAG, "Password: %s", WIFI_PASS);

    // Keep alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
