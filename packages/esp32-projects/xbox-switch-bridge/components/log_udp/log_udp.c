/**
 * @file log_udp.c
 * @brief WiFi AP + UDP broadcast log sink.
 *
 * Starts a SoftAP so any device can connect directly and receive logs.
 * No external router or credentials needed.
 */

#include "log_udp.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "status_led.h"

static const char *TAG = "log_udp";

#define LOG_AP_SSID "xbox-bridge-log"
#define LOG_AP_PASS "xboxbridge"
#define LOG_AP_CHANNEL 6
#define LOG_AP_MAX_CONN 2
#define LOG_BUF_SIZE 256

/* Broadcast to the AP subnet (192.168.4.255) */
#define LOG_AP_BROADCAST "192.168.4.255"

static int s_sock = -1;
static struct sockaddr_in s_dest;
static bool s_client_connected = false;

/* ── WiFi event handler ─────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Client connected (MAC: " MACSTR ")", MAC2STR(event->mac));
        s_client_connected = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "Client disconnected (MAC: " MACSTR ")", MAC2STR(event->mac));
        s_client_connected = false;
    }
}

/* ── Log hook ────────────────────────────────────────────────────────────── */

static int udp_log_vprintf(const char *fmt, va_list args)
{
    va_list args_copy;
    va_copy(args_copy, args);
    int ret = vprintf(fmt, args);

    /* Best-effort UDP broadcast; skip if no client has connected yet */
    if (s_sock >= 0 && s_client_connected) {
        char buf[LOG_BUF_SIZE];
        int len = vsnprintf(buf, sizeof(buf), fmt, args_copy);
        if (len > 0) {
            sendto(s_sock, buf, (size_t)len, 0, (struct sockaddr *)&s_dest, sizeof(s_dest));
        }
    }

    va_end(args_copy);
    return ret;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t log_udp_init(uint16_t port)
{
    /* netif / event loop init — idempotent if already called */
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap =
            {
                .ssid = LOG_AP_SSID,
                .ssid_len = sizeof(LOG_AP_SSID) - 1,
                .password = LOG_AP_PASS,
                .channel = LOG_AP_CHANNEL,
                .authmode = WIFI_AUTH_WPA2_PSK,
                .max_connection = LOG_AP_MAX_CONN,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));

    // Set country code for proper regional TX power
    wifi_country_t country = {
        .cc = "FI", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_ERROR_CHECK(esp_wifi_start());

    // Force HT20 bandwidth and max TX power for reliable AP visibility
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));  // 19.5 dBm

    ESP_LOGI(TAG, "AP started: SSID=\"%s\" ch=%d bw=HT20 country=FI", LOG_AP_SSID, LOG_AP_CHANNEL);

    /* Open UDP socket for broadcast */
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return ESP_FAIL;
    }

    /* Enable broadcast on the socket */
    int broadcast = 1;
    setsockopt(s_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family = AF_INET;
    s_dest.sin_port = htons(port);
    inet_aton(LOG_AP_BROADCAST, &s_dest.sin_addr);

    /* Hook ESP_LOG */
    esp_log_set_vprintf(udp_log_vprintf);
    ESP_LOGI(TAG,
             "UDP log broadcast on port %u — connect to '%s' and run: "
             "socat UDP-RECV:%u STDOUT",
             port, LOG_AP_SSID, port);

    /* Cyan flash = AP ready, waiting for client */
    status_led_flash(0, 32, 32, 500);

    return ESP_OK;
}
