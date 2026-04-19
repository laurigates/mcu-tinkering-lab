/**
 * @file main.c
 * @brief ThinkPack Finderbox — NFC scan-to-chime box for ESP32-S3.
 *
 * Boot sequence:
 *   1. NVS init (for tag registry blob).
 *   2. netif + event loop + WiFi STA init (required by ESP-NOW).
 *   3. Load tag registry from NVS.
 *   4. Init peripherals: RC522, piezo, LED ring.
 *   5. Bring up the ThinkPack ESP-NOW mesh (box_type = BOX_FINDERBOX).
 *   6. Start the standalone scan task.
 *
 * Group-mode behaviour (Hot-Cold, NFC Story Sounds) is handled by the
 * behaviors component + PR E on top of this foundation.
 */

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_mode.h"
#include "led_ring.h"
#include "nvs_flash.h"
#include "piezo.h"
#include "standalone_mode.h"
#include "tag_registry.h"
#include "thinkpack_nfc.h"
#include "thinkpack_protocol.h"
#include "thinkpack_rc522.h"

static const char *TAG = "finderbox";

/* One static registry instance shared between main and the scan task. */
static thinkpack_nfc_registry_t s_registry;

/* ------------------------------------------------------------------ */
/* WiFi STA init (ESP-NOW prerequisite)                                */
/* ------------------------------------------------------------------ */

static void wifi_init_for_espnow(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ------------------------------------------------------------------ */
/* app_main                                                             */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* NVS — required before WiFi start and by the tag registry. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Read MAC before WiFi init — available from eFuse */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "finderbox starting — MAC: " MACSTR, MAC2STR(mac));

    /* WiFi STA must be up before mesh init. */
    wifi_init_for_espnow();

    /* Tag registry: load or start empty. */
    ESP_ERROR_CHECK(tag_registry_load(&s_registry));

    /* Peripherals. */
    ESP_ERROR_CHECK(led_ring_init());
    ESP_ERROR_CHECK(piezo_init());
    ret = thinkpack_rc522_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RC522 init failed (%s) — continuing so mesh still comes up",
                 esp_err_to_name(ret));
    }

    /* Mesh config. */
    thinkpack_mesh_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.box_type = BOX_FINDERBOX;
    cfg.capabilities = CAP_NFC | CAP_LED_RING | CAP_AUDIO_OUT | CAP_STORAGE;
    cfg.channel = 1;
    cfg.battery_level = 100;
    cfg.beacon_interval_ms = 500;
    strncpy(cfg.name, "finderbox", THINKPACK_BOX_NAME_LEN - 1);

    ESP_ERROR_CHECK(thinkpack_mesh_init(&cfg));

    /* Wire the group-mode event handler before starting the mesh so
     * LEADER_ELECTED events fire through us from the very first tick. */
    group_mode_init(&s_registry);
    ESP_ERROR_CHECK(thinkpack_mesh_set_event_callback(group_mode_on_event, NULL));

    ESP_ERROR_CHECK(thinkpack_mesh_start());

    ESP_LOGI(TAG, "Mesh started — starting scan task");

    /* Start the standalone NFC scan task. */
    ESP_ERROR_CHECK(standalone_mode_start(&s_registry));

    /* Main task parks here. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "peers=%u registry=%u", (unsigned)thinkpack_mesh_peer_count(),
                 (unsigned)s_registry.count);
    }
}
