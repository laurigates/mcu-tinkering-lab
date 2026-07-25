/**
 * @file wifi_manager.c
 * @brief WiFi connection management implementation
 */

#include "wifi_manager.h"
#include <inttypes.h>
#include <string.h>
#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "wifi_manager";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAXIMUM_RETRY 5

/* Delay between the first WIFI_MAXIMUM_RETRY attempts. Retrying straight from
 * the disconnect event (no delay at all) burned the whole retry budget in under
 * a second, which is useless against the common case — an AP that is still
 * rebooting. */
#define WIFI_FAST_RETRY_DELAY_MS 1000
/* Ceiling for the exponential backoff that runs after the fast retries are
 * spent. Reconnection must never stop: a router outage has to heal by itself,
 * without a power cycle. */
#define WIFI_MAX_BACKOFF_MS 300000

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static uint32_t s_backoff_ms = 0;
static esp_timer_handle_t s_reconnect_timer;

static void reconnect_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Reconnect timer fired — retrying association");
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_connect failed (%s)", esp_err_to_name(err));
    }
}

/** Arm the one-shot reconnect timer. Called from the event-loop task, where
 *  blocking is not an option — hence a timer rather than a delay. */
static void schedule_reconnect(uint32_t delay_ms)
{
    if (!s_reconnect_timer) {
        return;
    }
    esp_timer_stop(s_reconnect_timer);  // no-op if not running
    esp_err_t err = esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not arm reconnect timer (%s)", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Reconnecting in %" PRIu32 " ms", delay_ms);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                          void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
        /* Drop the connected bit first: wifi_is_connected() feeds self_report's
         * health-change detection, and leaving it latched made a dropped link
         * indistinguishable from a healthy one for the rest of the uptime. */
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi disconnected. Reason: %d (%s)", disconnected->reason,
                 disconnected->reason == WIFI_REASON_NO_AP_FOUND         ? "AP not found"
                 : disconnected->reason == WIFI_REASON_AUTH_FAIL         ? "Auth failed"
                 : disconnected->reason == WIFI_REASON_ASSOC_FAIL        ? "Assoc failed"
                 : disconnected->reason == WIFI_REASON_HANDSHAKE_TIMEOUT ? "Handshake timeout"
                                                                         : "Other");

        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            s_retry_num++;
            ESP_LOGI(TAG, "Retry %d/%d to connect to the AP", s_retry_num, WIFI_MAXIMUM_RETRY);
            schedule_reconnect(WIFI_FAST_RETRY_DELAY_MS);
        } else {
            /* Unblock the initial wifi_connect() caller, but keep trying in the
             * background — the fast retries are exhausted, not the device. */
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            s_backoff_ms = (s_backoff_ms == 0)
                               ? WIFI_RETRY_INTERVAL_MS
                               : ((s_backoff_ms > WIFI_MAX_BACKOFF_MS / 2) ? WIFI_MAX_BACKOFF_MS
                                                                           : s_backoff_ms * 2);
            ESP_LOGE(TAG, "Failed to connect to AP after %d retries — backing off %" PRIu32 " ms",
                     WIFI_MAXIMUM_RETRY, s_backoff_ms);
            schedule_reconnect(s_backoff_ms);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR ", mask:" IPSTR ", gw:" IPSTR, IP2STR(&event->ip_info.ip),
                 IP2STR(&event->ip_info.netmask), IP2STR(&event->ip_info.gw));
        /* Log the resolver(s) DHCP installed. Purely diagnostic — the firmware
         * uses whatever DHCP supplies. This exists because a broken resolver
         * presents as a silent getaddrinfo() EAI_FAIL from deep inside esp-tls,
         * with nothing naming the resolver actually in use; printing it at
         * connect time is what distinguishes a link/router fault from a
         * firmware bug. A 0.0.0.0 entry means the AP offered no DNS server. */
        esp_netif_dns_info_t dns = {0};
        if (esp_netif_get_dns_info(event->esp_netif, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
            ESP_LOGI(TAG, "DNS main (DHCP):" IPSTR, IP2STR(&dns.ip.u_addr.ip4));
        }
        if (esp_netif_get_dns_info(event->esp_netif, ESP_NETIF_DNS_BACKUP, &dns) == ESP_OK &&
            dns.ip.u_addr.ip4.addr != 0) {
            ESP_LOGI(TAG, "DNS backup (DHCP):" IPSTR, IP2STR(&dns.ip.u_addr.ip4));
        }
        s_retry_num = 0;
        s_backoff_ms = 0;
        if (s_reconnect_timer) {
            esp_timer_stop(s_reconnect_timer);
        }
        /* Clear the stale failure bit too, so a later wifi_connect() does not
         * see a fail latched by a previous outage. */
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi");

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Could not create WiFi event group");
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t reconnect_args = {
        .callback = reconnect_timer_cb,
        .name = "wifi_reconnect",
    };
    esp_err_t timer_err = esp_timer_create(&reconnect_args, &s_reconnect_timer);
    if (timer_err != ESP_OK) {
        // Non-fatal: association still works, only the automatic retry is lost.
        ESP_LOGE(TAG, "Could not create reconnect timer (%s) — auto-reconnect disabled",
                 esp_err_to_name(timer_err));
        s_reconnect_timer = NULL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set country code for regulatory compliance (Finland)
    wifi_country_t country = {
        .cc = "FI", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_AUTO};
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    // Set power save mode off for better connectivity
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &event_handler, NULL, &instance_got_ip));

    ESP_LOGI(TAG, "WiFi initialized successfully");
    return ESP_OK;
}

esp_err_t wifi_connect(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", ssid);

    wifi_config_t wifi_config = {
        .sta =
            {
                .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
                .pmf_cfg = {.capable = true, .required = false},
                .scan_method = WIFI_FAST_SCAN,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            },
    };

    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    /* Start from a clean slate. Without this a second wifi_connect() — the
     * Improv provisioning path retrying with new credentials — would return
     * instantly on the WIFI_FAIL_BIT latched by the previous attempt, before
     * the new SSID had any chance to associate. */
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry_num = 0;
    s_backoff_ms = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed
     * for the maximum number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see
     * above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which
     * event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID:%s", ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

bool wifi_is_connected(void)
{
    if (!s_wifi_event_group) {
        return false;
    }
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

void wifi_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting from WiFi");
    esp_wifi_disconnect();
}

void wifi_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing WiFi");
    if (s_reconnect_timer) {
        esp_timer_stop(s_reconnect_timer);
        esp_timer_delete(s_reconnect_timer);
        s_reconnect_timer = NULL;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
}
