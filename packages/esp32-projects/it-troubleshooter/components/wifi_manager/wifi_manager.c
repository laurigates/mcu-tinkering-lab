/**
 * @file wifi_manager.c
 * @brief WiFi connection manager — connects to hotspot from credentials.h.
 */

#include "wifi_manager.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "wifi_manager";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAX_RETRIES 10

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_count = 0;
static volatile bool s_connected = false;
static esp_timer_handle_t s_retry_timer = NULL;

static void retry_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Retry timer fired — attempting reconnect");
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_retry_count < WIFI_MAX_RETRIES) {
            /* Exponential backoff: 1s, 2s, 4s … capped at 16s — use timer, not vTaskDelay */
            int delay_s = 1 << (s_retry_count < 4 ? s_retry_count : 4);
            ESP_LOGW(TAG, "Disconnected — retry %d/%d in %ds", s_retry_count + 1, WIFI_MAX_RETRIES,
                     delay_s);
            s_retry_count++;
            if (s_retry_timer != NULL) {
                esp_timer_start_once(s_retry_timer, (int64_t)delay_s * 1000000);
            }
        } else {
            ESP_LOGE(TAG, "Max retries reached — giving up");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        s_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_init(const char *ssid, const char *password)
{
    if (s_wifi_event_group != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = retry_timer_cb,
        .name = "wifi_retry",
    };
    esp_err_t timer_ret = esp_timer_create(&timer_args, &s_retry_timer);
    if (timer_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create retry timer (%s) — retries will be immediate",
                 esp_err_to_name(timer_ret));
        s_retry_timer = NULL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);
    return ESP_OK;
}

bool wifi_manager_is_connected(void)
{
    return s_connected;
}

void wifi_manager_deinit(void)
{
    if (s_wifi_event_group == NULL) {
        return;
    }
    if (s_retry_timer != NULL) {
        esp_timer_stop(s_retry_timer);
        esp_timer_delete(s_retry_timer);
        s_retry_timer = NULL;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;
    s_connected = false;
    s_retry_count = 0;
}
