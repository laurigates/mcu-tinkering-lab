/**
 * @file main.c
 * @brief IT Troubleshooter — Phase 3 entry point.
 *
 * Phase 3 adds Claude API integration. The command-passthrough loop is
 * replaced by a diagnostic loop: bytes received on CDC serial (operator-pasted
 * command output from the target) are forwarded to Claude API, and the
 * suggested next command is typed into the target via HID keyboard injection.
 *
 * Flow:
 *   1. NVS init + load persisted workflow state
 *   2. Status LED init (BOOT mode)
 *   3. USB composite init (HID keyboard + CDC serial)
 *   4. Wait for USB mount (30s timeout)
 *   5. WiFi init → connect to hotspot
 *   6. Wait for WiFi connected (30s timeout)
 *   7. Claude client init
 *   8. Spawn diagnostic loop task on core 1
 */

#include <string.h>

#include "claude_client.h"
#include "credentials.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "state_machine.h"
#include "status_led.h"
#include "usb_composite.h"
#include "wifi_manager.h"

static const char *TAG = "it-troubleshooter";

#define CDC_BUF_SIZE 256
#define NEXT_CMD_BUF_SIZE 256
#define USB_MOUNT_TIMEOUT_MS 30000
#define WIFI_CONNECT_TIMEOUT_MS 30000

/**
 * Diagnostic loop task: reads operator-pasted command output from CDC serial,
 * forwards it to Claude API for analysis, and injects the suggested next
 * command into the target via HID keyboard. Runs on core 1.
 *
 * Task stack: 8 KB (required for TLS + cJSON heap allocations).
 */
static void diagnostic_loop_task(void *arg)
{
    (void)arg;
    uint8_t buf[CDC_BUF_SIZE];
    char next_cmd[NEXT_CMD_BUF_SIZE];

    ESP_LOGI(TAG, "Diagnostic loop task started — paste command output to CDC");
    status_led_set_mode(STATUS_LED_DIAGNOSTIC);

    while (1) {
        int n = usb_cdc_read(buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            ESP_LOGI(TAG, "Received %d bytes — forwarding to Claude API", n);
            status_led_set_mode(STATUS_LED_WIFI_CONNECTING); /* reused: "thinking" */
            status_led_update();

            esp_err_t ret = claude_client_analyze((const char *)buf, next_cmd, sizeof(next_cmd));
            if (ret == ESP_OK) {
                if (strcmp(next_cmd, "DONE") == 0) {
                    ESP_LOGI(TAG, "Claude: troubleshooting complete");
                    status_led_set_mode(STATUS_LED_COMPLETE);
                } else {
                    ESP_LOGI(TAG, "Claude suggests: %s", next_cmd);
                    status_led_set_mode(STATUS_LED_BIOS_MODE);
                    usb_keyboard_type_string(next_cmd);
                    usb_keyboard_type_string("\n");
                    status_led_set_mode(STATUS_LED_DIAGNOSTIC);
                }
            } else {
                ESP_LOGW(TAG, "Claude analysis failed (%s) — continuing", esp_err_to_name(ret));
                status_led_set_mode(STATUS_LED_ERROR);
            }
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

    /* 1a. Load persisted workflow state */
    ret = sm_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "State machine init failed (%s) — continuing with defaults",
                 esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Workflow state: phase=%u step=%u", (unsigned)sm_get_phase(),
             (unsigned)sm_get_step());

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
    sm_set_phase(SM_PHASE_USB_ONLY);

    /* 5. Init WiFi — connect to hotspot */
    status_led_set_mode(STATUS_LED_WIFI_CONNECTING);
    status_led_update();

    ret = wifi_manager_init(WIFI_SSID, WIFI_PASS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed (%s) — aborting", esp_err_to_name(ret));
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
    sm_set_phase(SM_PHASE_WIFI_CONNECTED);

    /* 6b. Init mDNS for it-troubleshooter.local hostname */
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("it-troubleshooter"));
    ESP_ERROR_CHECK(mdns_instance_name_set("IT Troubleshooter"));
    ESP_LOGI(TAG, "mDNS initialized: it-troubleshooter.local");

    /* 7. Init Claude API client */
    ret = claude_client_init(CLAUDE_API_KEY, CLAUDE_MODEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Claude client init failed (%s) — aborting", esp_err_to_name(ret));
        status_led_set_mode(STATUS_LED_ERROR);
        status_led_update();
        return;
    }
    ESP_LOGI(TAG, "Claude client ready");
    sm_set_phase(SM_PHASE_AI_RUNNING);

    /* 8. Start diagnostic loop task on core 1 (8 KB stack for TLS) */
    xTaskCreatePinnedToCore(diagnostic_loop_task, "diag_loop", 8192, NULL, 3, NULL, 1);
}
