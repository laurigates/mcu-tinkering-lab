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
 * Hardware: Waveshare ESP32-S3-Zero (or any ESP32-S3 with native USB)
 * Connection: USB-C from ESP32-S3 -> Nintendo Switch dock USB port
 */

#include <stdio.h>
#include <string.h>

#if CONFIG_BT_ENABLED
#include "bluepad32_host.h"
#include "button_mapper.h"
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_udp.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "status_led.h"
#if CONFIG_TINYUSB_ENABLED
#include "switch_pro_usb.h"
#endif

/* credentials.h is still used for future STA-mode WiFi features */
#if __has_include("credentials.h")
#include "credentials.h"
#endif

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
#if CONFIG_TINYUSB_ENABLED
static bool s_usb_init_ok = false;
#endif

#if CONFIG_BT_ENABLED
/**
 * @brief Callback when Xbox controller connects/disconnects.
 */
static void on_controller_connection(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "*** Xbox controller CONNECTED ***");
        s_state = BRIDGE_STATE_CONNECTED;
        /* LED substate depends on USB status — bridge_task will update it */
    } else {
        ESP_LOGW(TAG, "*** Xbox controller DISCONNECTED ***");
        s_state = BRIDGE_STATE_SCANNING;
        status_led_set_mode(STATUS_LED_SCANNING);
    }
}
#endif

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

#if CONFIG_BT_ENABLED
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
    switch_pro_input_t switch_input = {
        .lx = 2048,
        .ly = 2048,
        .rx = 2048,
        .ry = 2048,
    };
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t report_count = 0;

    ESP_LOGI(TAG, "Bridge task started on core %d", xPortGetCoreID());

    /* Log stack high-water mark after first few iterations to catch sizing issues early */
    uint32_t hwm_check_at = 100; /* report count threshold for next HWM log */

    while (1) {
        switch (s_state) {
            case BRIDGE_STATE_SCANNING:
                /* Just wait for Bluepad32 to find a controller */
                break;

            case BRIDGE_STATE_CONNECTED:
#if CONFIG_TINYUSB_ENABLED
                /* Xbox connected — update LED to reflect USB progress */
                if (!s_usb_init_ok) {
                    status_led_set_mode(STATUS_LED_USB_ERROR);
                } else if (!switch_pro_usb_is_mounted()) {
                    status_led_set_mode(STATUS_LED_CONNECTED_NO_USB);
                } else if (!switch_pro_usb_is_ready()) {
                    status_led_set_mode(STATUS_LED_CONNECTED_USB);
                } else {
                    ESP_LOGI(TAG, "*** BRIDGE ACTIVE ***");
                    s_state = BRIDGE_STATE_BRIDGING;
                    status_led_set_mode(STATUS_LED_BRIDGING);
                }
                /* Don't send 0x30 reports until handshake completes.
                 * Real Pro Controllers only start 0x30 after FORCE_USB. */
#else
            {
                static bool logged_once = false;
                if (!logged_once) {
                    ESP_LOGI(TAG, "Xbox connected (USB disabled in debug build)");
                    logged_once = true;
                }
                status_led_set_mode(STATUS_LED_CONNECTED_NO_USB);
            }
#endif
                break;

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
#if CONFIG_TINYUSB_ENABLED
                switch_pro_usb_send_report(&switch_input);
#endif
                report_count++;

                if (report_count % 1000 == 0) {
                    ESP_LOGI(TAG, "Reports sent: %lu", (unsigned long)report_count);
                }
                break;

            default:
                break;
        }

        /* Periodic stack high-water mark check (exponential backoff) */
        if (report_count == hwm_check_at) {
            ESP_LOGI(TAG, "Stack HWM — bridge: %u bytes free",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
            hwm_check_at *= 10; /* 100 → 1000 → 10000 → ... */
        }

        status_led_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BRIDGE_LOOP_INTERVAL_MS));
    }
}
#endif /* CONFIG_BT_ENABLED */

void app_main(void)
{
    /* USB-Serial-JTAG logging is handled by CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
     * in the sdkconfig overlay (debug/wifi-test builds). No manual driver init needed. */

    ESP_LOGI(TAG, "=== Xbox -> Switch Controller Bridge ===");
    ESP_LOGI(TAG, "Firmware built: %s %s", __DATE__, __TIME__);

    /* Initialize status LED */
    ESP_ERROR_CHECK(status_led_init());

    /* Initialize NVS (required for BT and WiFi) */
    ESP_ERROR_CHECK(init_nvs());

    /* Start WiFi AP for UDP log broadcasting.
     * Connect to "xbox-bridge-log" network, then: socat UDP-RECV:4444 STDOUT */
    log_udp_init(4444);

#if CONFIG_TINYUSB_ENABLED
    /* Initialize USB HID (Switch Pro Controller emulation).
     * TinyUSB may fail when not connected to a Switch dock (e.g. on a
     * computer USB port). Log the error and continue — BLE scanning still
     * works, and USB will be retried on next reboot with a dock. */
    esp_err_t usb_ret = switch_pro_usb_init();
    if (usb_ret == ESP_OK) {
        s_usb_init_ok = true;
    } else {
        ESP_LOGW(TAG, "USB init failed (%s) — not connected to Switch?", esp_err_to_name(usb_ret));
    }
#else
    ESP_LOGW(TAG, "TinyUSB disabled (debug build) — USB logging active");
#endif

#if CONFIG_BT_ENABLED
    /* Initialize Bluepad32 (BLE gamepad host) */
    ESP_ERROR_CHECK(bp32_host_init(on_controller_connection));
    s_state = BRIDGE_STATE_SCANNING;
    status_led_set_mode(STATUS_LED_SCANNING);

    ESP_LOGI(TAG, "Waiting for Xbox controller...");
    ESP_LOGI(TAG, "  1. Turn on your Xbox controller (hold Xbox button)");
    ESP_LOGI(TAG, "  2. Put it in pairing mode (hold Pair button)");
    ESP_LOGI(TAG, "  3. Connect ESP32-S3 USB to Switch dock");

    /* Start the bridge task on core 1 (core 0 is used by BTstack/BT) */
    xTaskCreatePinnedToCore(bridge_task, "bridge", 6144, NULL, 5, NULL, 1);

    /* Log stack high-water mark after all init is done */
    ESP_LOGI(TAG, "Stack HWM — main: %u bytes free",
             (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));

    /* Start BTstack event loop on core 0 (app_main task).
     * This call does NOT return - BTstack takes over this task. */
    bp32_host_start();
#else
    /* WiFi-only test mode — no BLE, just verify SoftAP is visible */
    ESP_LOGW(TAG, "BLE DISABLED — WiFi-only test mode");
    ESP_LOGI(TAG, "Connect to 'xbox-bridge-log' WiFi to verify AP visibility");
    ESP_LOGI(TAG, "Stack HWM — main: %u bytes free",
             (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
    status_led_set_mode(STATUS_LED_CONNECTED_NO_USB);
    /* Keep main task alive */
    while (1) {
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
#endif
}
