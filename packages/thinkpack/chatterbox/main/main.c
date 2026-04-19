/**
 * @file main.c
 * @brief ThinkPack chatterbox — touch-driven audio box on ESP32-S3 SuperMini.
 *
 * Initialises NVS, the netif event loop, WiFi (STA mode for ESP-NOW), the
 * I2S audio engine, touch peripheral, and joins the ThinkPack mesh.  The
 * touch driver drives a thinkpack_audio_sm_t via standalone_mode; group_mode
 * handles MSG_AUDIO_CLIP_BROADCAST.
 *
 * Hardware:
 *   MAX98357A speaker — I2S0 (BCLK 4, LRCLK 5, DOUT 6)
 *   INMP441 mic       — I2S1 (BCLK 10, LRCLK 11, DIN 12)
 *   Touch pad         — GPIO14 (touch channel 14)
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/197
 */

#include <string.h>

#include "audio_engine.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_mode.h"
#include "nvs_flash.h"
#include "standalone_mode.h"
#include "thinkpack_protocol.h"
#include "touch_driver.h"

static const char *TAG = "chatterbox";

/* ------------------------------------------------------------------ */
/* Touch edge callback                                                 */
/* ------------------------------------------------------------------ */

static void on_touch_edge(bool pressed)
{
    ESP_LOGD(TAG, "touch %s", pressed ? "PRESS" : "RELEASE");
    standalone_mode_on_touch(pressed);

    /* When the release transition yields a captured clip and we're in a
     * mesh with peers, broadcast it so followers can echo. */
    if (!pressed) {
        size_t n = audio_engine_clip_sample_count();
        if (n > 0 && thinkpack_mesh_peer_count() > 0) {
            group_mode_broadcast_clip((uint16_t)n);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Audio tick task — services the record loop at 50 Hz                 */
/* ------------------------------------------------------------------ */

static void audio_task(void *arg)
{
    (void)arg;
    for (;;) {
        standalone_mode_tick();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "chatterbox starting — MAC: " MACSTR, MAC2STR(mac));

    /* Peripherals */
    ESP_ERROR_CHECK(audio_engine_init());
    standalone_mode_init();
    group_mode_init();

    ESP_ERROR_CHECK(touch_driver_init(on_touch_edge));

    /* Mesh */
    thinkpack_mesh_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.box_type = BOX_CHATTERBOX;
    cfg.capabilities = CAP_AUDIO_OUT | CAP_AUDIO_IN | CAP_TOUCH;
    cfg.channel = 1;
    cfg.battery_level = 100;
    cfg.beacon_interval_ms = 500;
    strncpy(cfg.name, "chatterbox", THINKPACK_BOX_NAME_LEN - 1);

    ESP_ERROR_CHECK(thinkpack_mesh_init(&cfg));
    thinkpack_mesh_set_event_callback(group_mode_on_event, NULL);
    ESP_ERROR_CHECK(thinkpack_mesh_start());

    ESP_LOGI(TAG, "mesh started — spawning audio task");
    xTaskCreate(audio_task, "chat_audio", 4096, NULL, 5, NULL);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "peers=%u clip=%u samples", (unsigned)thinkpack_mesh_peer_count(),
                 (unsigned)audio_engine_clip_sample_count());
    }
}
