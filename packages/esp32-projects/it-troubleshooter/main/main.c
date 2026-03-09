/**
 * @file main.c
 * @brief IT Troubleshooter — Phase 1 entry point.
 *
 * Initializes USB composite device (HID keyboard + CDC serial) and demonstrates
 * keyboard typing and CDC echo functionality.
 */

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "status_led.h"
#include "usb_composite.h"

static const char *TAG = "it-troubleshooter";

#define CDC_BUF_SIZE 256
#define USB_MOUNT_TIMEOUT_MS 30000

/**
 * CDC echo task: reads data from CDC serial and echoes it back.
 * Runs on core 1 to avoid contending with USB stack on core 0.
 */
static void cdc_echo_task(void *arg)
{
    (void)arg;
    uint8_t buf[CDC_BUF_SIZE];

    ESP_LOGI(TAG, "CDC echo task started");

    while (1) {
        int n = usb_cdc_read(buf, sizeof(buf));
        if (n > 0) {
            usb_cdc_write(buf, n);
            ESP_LOGI(TAG, "CDC echo: %d bytes", n);
        }
        /* Update status LED from this loop */
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    /* 1. Init NVS (required before any NVS reads in later phases) */
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

    /* 4. Wait for USB mount (with timeout) */
    int mount_wait_ms = 0;
    while (!usb_composite_is_mounted() && mount_wait_ms < USB_MOUNT_TIMEOUT_MS) {
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(10));
        mount_wait_ms += 10;
    }
    if (!usb_composite_is_mounted()) {
        ESP_LOGW(TAG, "USB mount timeout after %d ms — no host connected?", USB_MOUNT_TIMEOUT_MS);
        status_led_set_mode(STATUS_LED_ERROR);
        status_led_update();
        return;
    }
    status_led_set_mode(STATUS_LED_USB_READY);
    status_led_update();
    ESP_LOGI(TAG, "USB mounted");

    /* 5. Phase 1 demo: type greeting after 3 second delay */
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Typing demo string...");
    usb_keyboard_type_string("Hello from IT Troubleshooter!\n");

    /* 6. Start CDC echo task on core 1 */
    xTaskCreatePinnedToCore(cdc_echo_task, "cdc_echo", 4096, NULL, 3, NULL, 1);
}
