/**
 * @file main.c
 * @brief IT Troubleshooter — Phase 2 entry point.
 *
 * Phase 2 adds WiFi connectivity (mobile hotspot) and replaces the CDC echo
 * demo with a command-passthrough loop: bytes received on CDC serial are
 * typed into the target via HID keyboard injection.
 *
 * Flow:
 *   1. NVS init
 *   2. Status LED init (BOOT mode)
 *   3. USB composite init (HID keyboard + CDC serial)
 *   4. Wait for USB mount (30s timeout)
 *   5. WiFi init → connect to hotspot
 *   6. Wait for WiFi connected (30s timeout)
 *   7. Spawn command-passthrough task on core 1
 */

#include <string.h>

#include "credentials.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "status_led.h"
#include "usb_composite.h"
#include "wifi_manager.h"

static const char *TAG = "it-troubleshooter";

#define CDC_BUF_SIZE         256
#define USB_MOUNT_TIMEOUT_MS 30000
#define WIFI_CONNECT_TIMEOUT_MS 30000

/**
 * Command passthrough task: reads bytes from CDC serial and injects them as
 * HID keystrokes into the target computer. Runs on core 1.
 */
static void command_passthrough_task(void *arg)
{
    (void)arg;
    uint8_t buf[CDC_BUF_SIZE];

    ESP_LOGI(TAG, "Command passthrough task started");
    status_led_set_mode(STATUS_LED_DIAGNOSTIC);

    while (1) {
        int n = usb_cdc_read(buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            ESP_LOGI(TAG, "Injecting %d bytes via HID", n);
            usb_keyboard_type_string((const char *)buf);
        }
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    /* 1. Init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialized");

    /* 2. Init status LED */
    ESP_ERROR_CHECK(status_led_init());
    status_led_set_mode(STATUS_LED_BOOT);
    status_led_update();

    /* 3. Init USB composite (HID keyboard + CDC serial) */
    ESP_ERROR_CHECK(usb_composite_init());
    ESP_LOGI(TAG, "Waiting for USB mount...");

    /* 4. Wait for USB mount */
    int wait_ms = 0;
    while (!usb_composite_is_mounted() && wait_ms < USB_MOUNT_TIMEOUT_MS) {
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10;
    }
    if (!usb_composite_is_mounted()) {
        ESP_LOGW(TAG, "USB mount timeout after %d ms", USB_MOUNT_TIMEOUT_MS);
        status_led_set_mode(STATUS_LED_ERROR);
        status_led_update();
        return;
    }
    status_led_set_mode(STATUS_LED_USB_READY);
    status_led_update();
    ESP_LOGI(TAG, "USB mounted");

    /* 5. Init WiFi — connect to hotspot */
    status_led_set_mode(STATUS_LED_WIFI_CONNECTING);
    status_led_update();

    ret = wifi_manager_init(WIFI_SSID, WIFI_PASS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed (%s) — continuing without WiFi", esp_err_to_name(ret));
        status_led_set_mode(STATUS_LED_ERROR);
        status_led_update();
        return;
    }

    /* 6. Wait for WiFi connected */
    wait_ms = 0;
    while (!wifi_manager_is_connected() && wait_ms < WIFI_CONNECT_TIMEOUT_MS) {
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10;
    }
    if (!wifi_manager_is_connected()) {
        ESP_LOGW(TAG, "WiFi connect timeout after %d ms", WIFI_CONNECT_TIMEOUT_MS);
        status_led_set_mode(STATUS_LED_ERROR);
        status_led_update();
        return;
    }
    ESP_LOGI(TAG, "WiFi connected");

    /* 7. Start command passthrough task on core 1 */
    xTaskCreatePinnedToCore(command_passthrough_task, "cmd_passthrough", 4096, NULL, 3, NULL, 1);
}
