/**
 * @file main.c
 * @brief ThinkPack glowbug — light + motion box for ESP32-S3 SuperMini.
 *
 * Initialises peripherals (WS2812B ring, MPU6050, LDR, button), brings up
 * the ThinkPack ESP-NOW mesh, then delegates display work to an animation
 * task pinned to Core 1 at 30 Hz.
 *
 * Hardware:
 *   WS2812B ring — GPIO4
 *   MPU6050 IMU  — SDA GPIO6, SCL GPIO7
 *   LDR          — GPIO1 (ADC1_CH0, 10kΩ pulldown)
 *   Button        — GPIO9 (active LOW, internal pull-up)
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#include <string.h>

#include "animations.h"
#include "button.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "group_mode.h"
#include "imu.h"
#include "led_ring.h"
#include "light_sensor.h"
#include "standalone_mode.h"
#include "thinkpack_protocol.h"

static const char *TAG = "glowbug";

/* ------------------------------------------------------------------ */
/* Button callback                                                     */
/* ------------------------------------------------------------------ */

static void on_button_press(void)
{
    standalone_mode_cycle_override();
}

/* ------------------------------------------------------------------ */
/* Animation task (Core 1, 30 Hz)                                      */
/* ------------------------------------------------------------------ */

static void animation_task(void *arg)
{
    (void)arg;

    for (;;) {
        uint32_t t = (uint32_t)(esp_timer_get_time() / 1000LL);
        standalone_mode_tick(t);
        animations_render(t);
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* Read MAC before WiFi init — available from eFuse */
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "glowbug starting — MAC: " MACSTR, MAC2STR(mac));

    /* Peripherals */
    ESP_ERROR_CHECK(led_ring_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(light_sensor_init());
    ESP_ERROR_CHECK(button_init(on_button_press));

    /* Animation engine + standalone state machine */
    animations_init();
    standalone_mode_init();

    /* Mesh */
    thinkpack_mesh_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.box_type = BOX_GLOWBUG;
    cfg.capabilities = CAP_LED_RING | CAP_IMU | CAP_LIGHT_SENSE;
    cfg.channel = 1;
    cfg.battery_level = 100;
    cfg.beacon_interval_ms = 500;
    strncpy(cfg.name, "glowbug", THINKPACK_BOX_NAME_LEN - 1);

    ESP_ERROR_CHECK(thinkpack_mesh_init(&cfg));
    thinkpack_mesh_set_event_callback(group_mode_on_event, NULL);
    ESP_ERROR_CHECK(thinkpack_mesh_start());

    ESP_LOGI(TAG, "Mesh started — spawning animation task on Core 1");

    /* Animation task pinned to Core 1 at priority 5, 4 KB stack */
    xTaskCreatePinnedToCore(animation_task, "glow_anim", 4096, NULL, 5, NULL, 1);

    /* Main task parks here; all work happens in animation_task + mesh tasks */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "peers=%u", (unsigned)thinkpack_mesh_peer_count());
    }
}
