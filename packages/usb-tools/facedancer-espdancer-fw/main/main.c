/**
 * @file main.c
 * @brief facedancer espdancer firmware entry point (Milestone 0 scaffold).
 *
 * Bring-up only — no USB emulation yet. The raw-DCD relay (`components/raw_usb`)
 * is Milestone 1. This firmware:
 *   1. Initializes the WS2812 status LED.
 *   2. Starts NVS (required for WiFi).
 *   3. Brings up the "espdancer-log" WiFi SoftAP + UDP log sink (log_udp).
 *   4. Starts the usb_rpc TCP server on port 4444 (HELLO/GET_VERSION echo).
 *   5. Idles, updating the LED to reflect control-channel state.
 *
 * Architecture (once raw_usb lands):
 *   - Core 0: usb_rpc control-channel task (host RPC framing/dispatch)
 *   - Core 1: raw_usb relay pump (single-writer to TinyUSB DCD endpoints)
 *
 * Hardware: Waveshare ESP32-S3-Zero (same as xbox-switch-bridge).
 *   - USB-C → target host (USB emulation, Full Speed)
 *   - WiFi SoftAP "espdancer-log" → controlling PC (control + log channel)
 *   - (debug-uart) GPIO43/44 → CP2102 USB-UART adapter for logging
 *
 * Adapted from xbox-switch-bridge/main/main.c (BLE/controller logic removed).
 */
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_udp.h"
#include "nvs_flash.h"
#include "raw_usb.h"
#include "sdkconfig.h"
#include "status_led.h"
#include "usb_rpc.h"

static const char *TAG = "espdancer";

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void app_main(void)
{
    status_led_set_mode(STATUS_LED_BOOT);
    ESP_LOGI(TAG, "=== facedancer espdancer firmware ===");
    ESP_LOGI(TAG, "Built: %s %s", __DATE__, __TIME__);

    /* 1. Status LED */
    ESP_ERROR_CHECK(status_led_init());

    /* 2. NVS (WiFi) */
    ESP_ERROR_CHECK(init_nvs());

    /* 3. WiFi SoftAP + UDP log broadcast (also the carrier for usb_rpc) */
    esp_err_t wifi_ret = log_udp_init(4444);
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "log_udp_init failed: %s — control channel unavailable",
                 esp_err_to_name(wifi_ret));
        status_led_set_mode(STATUS_LED_USB_ERROR);
    } else {
        status_led_set_mode(STATUS_LED_WIFI_AP_UP);
    }

    /* 4. Control channel TCP server (needs WiFi AP up) */
    esp_err_t rpc_ret = usb_rpc_start();
    if (rpc_ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_rpc_start failed: %s", esp_err_to_name(rpc_ret));
    }

    /* 5. raw_usb relay (M1). dcd_event_handler installs the USB ISR once the
     *    DCD-only TinyUSB port is vendored (see components/raw_usb/tinyusb_port/).
     *    Until then raw_usb_init() just allocates the event queue; the pump task
     *    is started so we can verify on a wired USB cable (later) that no events
     *    fire pre-vendoring. */
    esp_err_t usb_ret = raw_usb_init();
    if (usb_ret != ESP_OK) {
        ESP_LOGW(TAG, "raw_usb_init failed: %s", esp_err_to_name(usb_ret));
    } else {
        xTaskCreatePinnedToCore(raw_usb_pump_task, "raw_usb_pump", 4096, NULL, 4,
                                NULL, 1);
        ESP_LOGI(TAG, "raw_usb pump started on core 1");
    }

    ESP_LOGI(TAG, "Connect to 'espdancer-log' WiFi, then TCP 192.168.4.1:%d",
             USB_RPC_PORT);
    ESP_LOGI(TAG, "  Send a HELLO frame: %02X %02X 00 03 01 00 01 (CRC)",
             USB_RPC_MAGIC0, USB_RPC_MAGIC1);

    /* 6. Idle — reflect control-channel state on the LED */
    bool was_connected = false;
    while (1) {
        bool connected = usb_rpc_host_connected();
        if (connected != was_connected) {
            status_led_set_mode(connected ? STATUS_LED_HOST_CONNECTED
                                           : STATUS_LED_WIFI_AP_UP);
            ESP_LOGI(TAG, "Host %s", connected ? "connected" : "disconnected");
            was_connected = connected;
        }
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}