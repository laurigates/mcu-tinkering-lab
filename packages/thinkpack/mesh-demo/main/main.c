/**
 * @file main.c
 * @brief ThinkPack mesh demo — validates Phase 1 foundation on two ESP32-S3 boards.
 *
 * Identity is derived from the last byte of the WiFi-STA MAC address:
 *   even → glowbug-demo  (BOX_GLOWBUG, LED ring + IMU + light sensor)
 *   odd  → boombox-demo  (BOX_BOOMBOX, audio out + pots + rhythm)
 *
 * No peripherals required — all output goes to the USB-Serial-JTAG monitor.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/194
 */

#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "espnow_mesh.h"
#include "thinkpack_protocol.h"

static const char *TAG = "demo";

/* ------------------------------------------------------------------ */
/* Event callback                                                      */
/* ------------------------------------------------------------------ */

static void on_mesh_event(const thinkpack_mesh_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;

    switch (event->type) {
        case THINKPACK_EVENT_PEER_DISCOVERED:
            ESP_LOGI(TAG, "peer discovered: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_PEER_LOST:
            ESP_LOGW(TAG, "peer lost: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_ELECTED:
            ESP_LOGI(TAG, "leader elected: " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_BECAME_LEADER:
            ESP_LOGI(TAG, "I AM THE LEADER");
            break;

        case THINKPACK_EVENT_BECAME_FOLLOWER:
            ESP_LOGI(TAG, "following " MACSTR, MAC2STR(event->peer_mac));
            break;

        case THINKPACK_EVENT_LEADER_LOST:
            ESP_LOGW(TAG, "leader lost");
            break;

        case THINKPACK_EVENT_SYNC_PULSE:
            /* Intentionally silent in this demo — reduces noise. */
            break;

        case THINKPACK_EVENT_COMMAND_RECEIVED:
            ESP_LOGI(TAG, "command received from " MACSTR, MAC2STR(event->peer_mac));
            break;

        default:
            ESP_LOGW(TAG, "unknown event type %d", (int)event->type);
            break;
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* Read the WiFi-STA MAC to choose this board's identity. The MAC is
     * burned into eFuse and is available before WiFi is initialised. */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));

    ESP_LOGI(TAG, "local MAC: " MACSTR, MAC2STR(mac));

    thinkpack_mesh_config_t config;
    memset(&config, 0, sizeof(config));

    if ((mac[5] & 1u) == 0u) {
        /* even last byte → glowbug identity */
        config.box_type = BOX_GLOWBUG;
        config.capabilities = CAP_LED_RING | CAP_IMU | CAP_LIGHT_SENSE;
        strncpy(config.name, "glowbug-demo", THINKPACK_BOX_NAME_LEN - 1);
        ESP_LOGI(TAG, "identity: glowbug-demo (BOX_GLOWBUG)");
    } else {
        /* odd last byte → boombox identity */
        config.box_type = BOX_BOOMBOX;
        config.capabilities = CAP_AUDIO_OUT | CAP_POTS | CAP_RHYTHM;
        strncpy(config.name, "boombox-demo", THINKPACK_BOX_NAME_LEN - 1);
        ESP_LOGI(TAG, "identity: boombox-demo (BOX_BOOMBOX)");
    }

    config.channel = 1;
    config.battery_level = 100;
    config.beacon_interval_ms = 500;

    ESP_ERROR_CHECK(thinkpack_mesh_init(&config));
    thinkpack_mesh_set_event_callback(on_mesh_event, NULL);
    ESP_ERROR_CHECK(thinkpack_mesh_start());

    ESP_LOGI(TAG, "mesh started — waiting for peers");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "peer_count = %u", (unsigned)thinkpack_mesh_peer_count());
    }
}
