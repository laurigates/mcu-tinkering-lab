/**
 * @file ota_handler.c
 * @brief OTA update handler implementation for robocar main controller
 *
 * Performs firmware updates triggered by I2C commands from the camera.
 * Initializes WiFi on-demand (from NVS credentials), constructs the
 * GitHub Release download URL from the release tag, and downloads/flashes
 * firmware using esp_https_ota with certificate bundle for TLS.
 */

#include "ota_handler.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "system_config.h"
#include "wifi_manager.h"

static const char *TAG = "OTA_Handler";

// OTA state
static ota_status_t s_ota_status = OTA_STATUS_IDLE;
static uint8_t s_ota_progress = 0;
static uint8_t s_ota_error_code = 0;
static bool s_firmware_valid_confirmed = false;

// OTA task parameters
typedef struct {
    char tag[OTA_TAG_MAX_LEN];
    uint8_t hash[OTA_HASH_LEN];
} ota_task_params_t;

// Forward declarations
static void ota_update_task(void *pvParameters);
static void ota_stability_timer_callback(TimerHandle_t xTimer);

esp_err_t ota_handler_init(void) {
    ESP_LOGI(TAG, "Initializing OTA handler");

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "Current firmware version: %s", app_desc->version);

    // Start rollback protection stability timer
    TimerHandle_t stability_timer = xTimerCreate(
        "ota_stability", pdMS_TO_TICKS(OTA_STABILITY_TIMEOUT_MS), pdFALSE,
        NULL, ota_stability_timer_callback);
    if (stability_timer) {
        xTimerStart(stability_timer, 0);
        ESP_LOGI(TAG, "Rollback stability timer started (%d ms)",
                 OTA_STABILITY_TIMEOUT_MS);
    }

    ESP_LOGI(TAG, "OTA handler initialized");
    return ESP_OK;
}

esp_err_t ota_handler_begin_update(const char *tag, const uint8_t *hash) {
    if (s_ota_status == OTA_STATUS_IN_PROGRESS) {
        ESP_LOGW(TAG, "OTA update already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    // Allocate parameters for the OTA task
    ota_task_params_t *params = malloc(sizeof(ota_task_params_t));
    if (!params) {
        ESP_LOGE(TAG, "Failed to allocate OTA task params");
        return ESP_ERR_NO_MEM;
    }

    strncpy(params->tag, tag, OTA_TAG_MAX_LEN - 1);
    params->tag[OTA_TAG_MAX_LEN - 1] = '\0';
    if (hash) {
        memcpy(params->hash, hash, OTA_HASH_LEN);
    } else {
        memset(params->hash, 0, OTA_HASH_LEN);
    }

    s_ota_status = OTA_STATUS_IN_PROGRESS;
    s_ota_progress = 0;
    s_ota_error_code = 0;

    // Run OTA in a separate task to avoid blocking I2C
    BaseType_t result = xTaskCreate(ota_update_task, "ota_update",
                                    OTA_TASK_STACK_SIZE, params,
                                    OTA_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA update task");
        free(params);
        s_ota_status = OTA_STATUS_FAILED;
        s_ota_error_code = 1;
        return ESP_FAIL;
    }

    return ESP_OK;
}

uint8_t ota_handler_get_progress(void) {
    return s_ota_progress;
}

ota_status_t ota_handler_get_status(void) {
    return s_ota_status;
}

uint8_t ota_handler_get_error_code(void) {
    return s_ota_error_code;
}

esp_err_t ota_handler_confirm_valid(void) {
    if (s_firmware_valid_confirmed) {
        return ESP_OK;
    }

    esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret == ESP_OK) {
        s_firmware_valid_confirmed = true;
        ESP_LOGI(TAG, "Firmware marked as valid — rollback cancelled");
    } else {
        ESP_LOGW(TAG, "Failed to mark firmware valid: %s (may not be OTA boot)",
                 esp_err_to_name(ret));
    }
    return ret;
}

static void ota_update_task(void *pvParameters) {
    ota_task_params_t *params = (ota_task_params_t *)pvParameters;

    ESP_LOGI(TAG, "OTA update task started for tag: %s", params->tag);

    // Step 1: Ensure WiFi is connected
    ESP_LOGI(TAG, "Checking WiFi connectivity...");
    if (!wifi_manager_is_connected()) {
        ESP_LOGI(TAG, "WiFi not connected — initializing for OTA...");

        // WiFi manager should already be initialized from app_main
        esp_err_t ret = wifi_manager_connect(NULL, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi connection: %s",
                     esp_err_to_name(ret));
            s_ota_status = OTA_STATUS_FAILED;
            s_ota_error_code = 2;  // WiFi connection error
            goto cleanup;
        }

        // Wait for WiFi connection (up to 30 seconds)
        int wait_count = 0;
        while (!wifi_manager_is_connected() && wait_count < 60) {
            vTaskDelay(pdMS_TO_TICKS(500));
            wait_count++;
        }

        if (!wifi_manager_is_connected()) {
            ESP_LOGE(TAG, "WiFi connection timeout — aborting OTA");
            s_ota_status = OTA_STATUS_FAILED;
            s_ota_error_code = 2;
            goto cleanup;
        }
    }

    ESP_LOGI(TAG, "WiFi connected — proceeding with OTA download");
    s_ota_progress = 5;

    // Step 2: Construct download URL from release tag
    char url[256];
    snprintf(url, sizeof(url), "https://github.com/%s/%s/releases/download/%s/robocar-main.bin",
             OTA_GITHUB_ORG, OTA_GITHUB_REPO, params->tag);

    ESP_LOGI(TAG, "Download URL: %s", url);
    s_ota_progress = 10;

    // Step 3: Perform OTA update via esp_https_ota
    esp_http_client_config_t http_config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ret = esp_https_ota_begin(&ota_config, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin failed: %s", esp_err_to_name(ret));
        s_ota_status = OTA_STATUS_FAILED;
        s_ota_error_code = 3;  // OTA begin error
        goto cleanup;
    }

    // Download firmware with progress tracking
    int image_size = esp_https_ota_get_image_size(ota_handle);
    ESP_LOGI(TAG, "Firmware image size: %d bytes", image_size);

    uint8_t last_logged_progress = 0;
    while (true) {
        ret = esp_https_ota_perform(ota_handle);
        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        // Update progress based on bytes read
        int bytes_read = esp_https_ota_get_image_len_read(ota_handle);
        if (image_size > 0) {
            s_ota_progress = 10 + (uint8_t)((bytes_read * 85L) / image_size);
        }

        if (s_ota_progress / 10 != last_logged_progress / 10 &&
            s_ota_progress % 10 == 0) {
            ESP_LOGI(TAG, "OTA progress: %d%% (%d/%d bytes)",
                     s_ota_progress, bytes_read, image_size);
            last_logged_progress = s_ota_progress;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_perform failed: %s", esp_err_to_name(ret));
        esp_https_ota_abort(ota_handle);
        s_ota_status = OTA_STATUS_FAILED;
        s_ota_error_code = 4;  // Download/flash error
        goto cleanup;
    }

    // Finalize the OTA update
    ret = esp_https_ota_finish(ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(ret));
        s_ota_status = OTA_STATUS_FAILED;
        s_ota_error_code = 5;  // Finalization error
        goto cleanup;
    }

    s_ota_progress = 100;
    s_ota_status = OTA_STATUS_SUCCESS;
    ESP_LOGI(TAG, "OTA update completed successfully!");
    ESP_LOGI(TAG, "Device will reboot when reboot command is received via I2C");

cleanup:
    if (s_ota_status == OTA_STATUS_FAILED) {
        ESP_LOGE(TAG, "OTA update failed with error code: %d",
                 s_ota_error_code);
    }
    free(params);
    vTaskDelete(NULL);
}

static void ota_stability_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Stability timeout reached — confirming firmware validity");
    ota_handler_confirm_valid();
}
