/**
 * @file ota_handler.c
 * @brief Robocar-main OTA handler — thin wrapper around the shared
 *        `ota_github` component in TRIGGERED mode.
 *
 * The main controller only downloads firmware when the camera tells it
 * to via I2C (`CMD_TYPE_BEGIN_OTA`). We therefore use
 * `OTA_GITHUB_MODE_TRIGGERED` and hook `pre_download_hook` to bring up
 * WiFi-on-demand from NVS-stored credentials — WiFi is otherwise unused
 * during normal operation.
 *
 * The I2C command dispatcher in `i2c_slave.c` continues to call the
 * same `ota_handler_*` API as before; internally we forward to
 * `ota_github_*`. The `ota_status_t` enum in `i2c_protocol.h` shares
 * numeric values with `ota_github_status_t` so casts are safe.
 */

#include "ota_handler.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota_github.h"
#include "system_config.h"
#include "wifi_manager.h"

static const char *TAG = "OTA_Handler";

/* WiFi-on-demand bring-up, gated by ota_github's pre_download_hook. */
static esp_err_t pre_download_hook(void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "Checking WiFi connectivity...");

    if (wifi_manager_is_connected()) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "WiFi not connected — initializing for OTA...");
    esp_err_t ret = wifi_manager_connect(NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi_manager_connect: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait up to 30 seconds for association. */
    for (int i = 0; i < 60 && !wifi_manager_is_connected(); i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi connection timeout — aborting OTA");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "WiFi connected — proceeding with OTA download");
    return ESP_OK;
}

esp_err_t ota_handler_init(void)
{
    ota_github_config_t cfg       = OTA_GITHUB_CONFIG_DEFAULT();
    cfg.mode                      = OTA_GITHUB_MODE_TRIGGERED;
    cfg.github_org                = OTA_GITHUB_ORG;
    cfg.github_repo               = OTA_GITHUB_REPO;
    cfg.triggered_asset_filename  = "robocar-main.bin";
    cfg.stability_timeout_ms      = OTA_STABILITY_TIMEOUT_MS;
    cfg.http_timeout_ms           = OTA_HTTP_TIMEOUT_MS;
    cfg.task_stack_size           = OTA_TASK_STACK_SIZE;
    cfg.task_priority             = OTA_TASK_PRIORITY;
    cfg.hooks.pre_download_hook   = pre_download_hook;

    esp_err_t ret = ota_github_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ota_github_init: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "OTA handler initialized (TRIGGERED mode)");
    return ESP_OK;
}

esp_err_t ota_handler_begin_update(const char *tag, const uint8_t *hash)
{
    return ota_github_trigger_tag(NULL, tag, hash);
}

uint8_t ota_handler_get_progress(void)
{
    return ota_github_get_progress();
}

ota_status_t ota_handler_get_status(void)
{
    /* ota_github_status_t matches ota_status_t numerically by design. */
    return (ota_status_t)ota_github_get_status();
}

uint8_t ota_handler_get_error_code(void)
{
    return ota_github_get_error_code();
}

esp_err_t ota_handler_confirm_valid(void)
{
    return ota_github_confirm_valid();
}
