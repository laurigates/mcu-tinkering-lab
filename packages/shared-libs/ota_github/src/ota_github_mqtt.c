/**
 * @file ota_github_mqtt.c
 * @brief Optional MQTT push-notify integration.
 *
 * When enabled, the component subscribes to the caller's configured
 * `mqtt_notify_topic` and triggers an immediate update check whenever a
 * message arrives. The trigger is rate-limited to at most once every 60
 * seconds to avoid churn if CI fires many notifications in quick succession.
 *
 * Supports both PULL mode (calls @ref ota_github_check_now) and TRIGGERED
 * mode (uses the message payload as the tag and triggers via
 * @ref ota_github_trigger_tag).
 */

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#include "ota_github.h"
#include "ota_github_internal.h"

static const char *TAG = "ota_github.mqtt";

static const int64_t RATE_LIMIT_US = 60LL * 1000LL * 1000LL;  /* 60 seconds */
static int64_t s_last_trigger_us = 0;

static void handle_notify(const char *payload, int payload_len)
{
    int64_t now = esp_timer_get_time();
    if (s_last_trigger_us > 0 && (now - s_last_trigger_us) < RATE_LIMIT_US) {
        ESP_LOGW(TAG, "MQTT trigger rate-limited (min interval 60s)");
        return;
    }
    s_last_trigger_us = now;

    if (g_ota_github.cfg.mode == OTA_GITHUB_MODE_PULL) {
        ESP_LOGI(TAG, "MQTT notify received — triggering update check");
        esp_err_t ret = ota_github_check_now();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ota_github_check_now: %s", esp_err_to_name(ret));
        }
    } else {
        /* TRIGGERED mode — use payload as tag (e.g. "myproject@v1.2.3"). */
        if (payload_len <= 0 || !payload) {
            ESP_LOGW(TAG, "MQTT notify in TRIGGERED mode with empty payload — ignoring");
            return;
        }
        char tag[64];
        size_t n = (size_t)payload_len < sizeof(tag) - 1 ? (size_t)payload_len : sizeof(tag) - 1;
        memcpy(tag, payload, n);
        tag[n] = '\0';

        ESP_LOGI(TAG, "MQTT notify received — triggering download for tag: %s", tag);
        esp_err_t ret = ota_github_trigger_tag(NULL, tag, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ota_github_trigger_tag: %s", esp_err_to_name(ret));
        }
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            if (g_ota_github.cfg.mqtt_notify_topic) {
                int msg_id = esp_mqtt_client_subscribe(g_ota_github.cfg.mqtt_client,
                                                      g_ota_github.cfg.mqtt_notify_topic, 1);
                ESP_LOGI(TAG, "Subscribed to %s (msg_id=%d)",
                         g_ota_github.cfg.mqtt_notify_topic, msg_id);
            }
            break;

        case MQTT_EVENT_DATA: {
            if (!event->topic || !g_ota_github.cfg.mqtt_notify_topic) {
                break;
            }
            /* topic is NOT null-terminated in the event — compare by length. */
            size_t want = strlen(g_ota_github.cfg.mqtt_notify_topic);
            if ((size_t)event->topic_len == want &&
                memcmp(event->topic, g_ota_github.cfg.mqtt_notify_topic, want) == 0) {
                handle_notify(event->data, event->data_len);
            }
            break;
        }

        default:
            break;
    }
}

esp_err_t ota_github_mqtt_init(void)
{
    if (!g_ota_github.cfg.mqtt_client || !g_ota_github.cfg.mqtt_notify_topic) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_mqtt_client_register_event(g_ota_github.cfg.mqtt_client,
                                                   ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_register_event: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MQTT notify enabled on topic: %s", g_ota_github.cfg.mqtt_notify_topic);
    return ESP_OK;
}
