/**
 * @file main.c
 * @brief Xbox Controller -> Nintendo Switch Bridge (ESP32-S3)
 *
 * This firmware turns an ESP32-S3 into a protocol bridge:
 *   1. Connects to an Xbox Series controller via Bluetooth LE (Bluepad32)
 *   2. Maps the Xbox inputs to Switch Pro Controller format
 *   3. Presents itself as a wired Switch Pro Controller over USB (TinyUSB)
 *
 * The bridge loop runs at ~125 Hz (8ms interval), matching the USB poll
 * rate expected by the Nintendo Switch.
 *
 * Hardware: ESP32-S3 DevKit (any variant with USB-OTG pins exposed)
 * Connection: USB-C from ESP32-S3 -> Nintendo Switch dock USB port
 */

#include <stdio.h>
#include <string.h>

#include "bluepad32_host.h"
#include "button_mapper.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "switch_pro_usb.h"

static const char *TAG = "xbox_switch_bridge";

/* Bridge loop interval: 8ms = 125 Hz USB poll rate */
#define BRIDGE_LOOP_INTERVAL_MS 8

typedef enum {
    BRIDGE_STATE_INIT,
    BRIDGE_STATE_SCANNING,  /* Waiting for Xbox controller */
    BRIDGE_STATE_CONNECTED, /* Xbox connected, waiting for Switch */
    BRIDGE_STATE_BRIDGING,  /* Both sides active, forwarding inputs */
} bridge_state_t;

static volatile bridge_state_t s_state = BRIDGE_STATE_INIT;

/**
 * @brief Callback when Xbox controller connects/disconnects.
 */
static void on_controller_connection(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "*** Xbox controller CONNECTED ***");
        s_state = BRIDGE_STATE_CONNECTED;
    } else {
        ESP_LOGW(TAG, "*** Xbox controller DISCONNECTED ***");
        s_state = BRIDGE_STATE_SCANNING;
    }
}

/**
 * @brief Initialize NVS (required by Bluepad32/BT stack).
 */
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Main bridge task.
 *
 * Runs the translation loop: read Xbox -> map -> send to Switch.
 * Runs on core 1 while Bluepad32/BTstack runs on core 0.
 */
static void bridge_task(void *arg)
{
    (void)arg;

    xbox_gamepad_state_t xbox_state;
    switch_pro_input_t switch_input;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t report_count = 0;

    ESP_LOGI(TAG, "Bridge task started on core %d", xPortGetCoreID());

    while (1) {
        switch (s_state) {
            case BRIDGE_STATE_SCANNING:
                /* Just wait for Bluepad32 to find a controller */
                break;

            case BRIDGE_STATE_CONNECTED:
                /* Xbox connected, check if Switch is ready too */
                if (switch_pro_usb_is_ready()) {
                    ESP_LOGI(TAG, "*** BRIDGE ACTIVE ***");
                    s_state = BRIDGE_STATE_BRIDGING;
                }
                /* Fall through to send neutral state */
                /* fallthrough */

            case BRIDGE_STATE_BRIDGING:
                if (bp32_host_get_state(&xbox_state)) {
                    button_mapper_convert(&xbox_state, &switch_input);
                } else {
                    /* Controller disconnected mid-bridge */
                    memset(&switch_input, 0, sizeof(switch_input));
                    switch_input.lx = 2048;
                    switch_input.ly = 2048;
                    switch_input.rx = 2048;
                    switch_input.ry = 2048;
                }
                switch_pro_usb_send_report(&switch_input);
                report_count++;

                if (report_count % 1000 == 0) {
                    ESP_LOGI(TAG, "Reports sent: %lu", (unsigned long)report_count);
                }
                break;

            default:
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BRIDGE_LOOP_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Xbox -> Switch Controller Bridge ===");
    ESP_LOGI(TAG, "Firmware built: %s %s", __DATE__, __TIME__);

    /* Initialize NVS (required for BT) */
    ESP_ERROR_CHECK(init_nvs());

    /* Initialize USB HID (Switch Pro Controller emulation) */
    ESP_ERROR_CHECK(switch_pro_usb_init());

    /* Initialize Bluepad32 (BLE gamepad host) */
    ESP_ERROR_CHECK(bp32_host_init(on_controller_connection));
    s_state = BRIDGE_STATE_SCANNING;

    ESP_LOGI(TAG, "Waiting for Xbox controller...");
    ESP_LOGI(TAG, "  1. Turn on your Xbox controller (hold Xbox button)");
    ESP_LOGI(TAG, "  2. Put it in pairing mode (hold Pair button)");
    ESP_LOGI(TAG, "  3. Connect ESP32-S3 USB to Switch dock");

    /* Start the bridge task on core 1 (core 0 is used by BTstack/BT) */
    xTaskCreatePinnedToCore(bridge_task, "bridge", 4096, NULL, 5, NULL, 1);

    /* Start BTstack event loop on core 0 (app_main task).
     * This call does NOT return - BTstack takes over this task. */
    bp32_host_start();
}
