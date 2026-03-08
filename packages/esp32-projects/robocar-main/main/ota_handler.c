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
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "system_config.h"
#include "wifi_manager.h"

static const char *TAG = "OTA_Handler";

// OTA state (protected by s_ota_mutex)
static ota_status_t s_ota_status = OTA_STATUS_IDLE;
static uint8_t s_ota_progress = 0;
static uint8_t s_ota_error_code = 0;
static SemaphoreHandle_t s_ota_mutex = NULL;
static bool s_firmware_valid_confirmed = false;
static TimerHandle_t s_stability_timer = NULL;

static inline void ota_set_state(ota_status_t status, uint8_t progress, uint8_t error)
{
    if (s_ota_mutex && xSemaphoreTake(s_ota_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_ota_status = status;
        s_ota_progress = progress;
        s_ota_error_code = error;
        xSemaphoreGive(s_ota_mutex);
    }
}

static inline void ota_set_progress(uint8_t progress)
{
    if (s_ota_mutex && xSemaphoreTake(s_ota_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_ota_progress = progress;
        xSemaphoreGive(s_ota_mutex);
    }
}

// OTA task parameters
typedef struct {
    char tag[OTA_TAG_MAX_LEN];
    uint8_t hash[OTA_HASH_LEN];
} ota_task_params_t;

// Forward declarations
static void ota_update_task(void *pvParameters);
static void ota_stability_timer_callback(TimerHandle_t xTimer);

esp_err_t ota_handler_init(void)
{
    ESP_LOGI(TAG, "Initializing OTA handler");

    if (!s_ota_mutex) {
        s_ota_mutex = xSemaphoreCreateMutex();
        if (!s_ota_mutex) {
            ESP_LOGE(TAG, "Failed to create OTA mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "Current firmware version: %s", app_desc->version);

    // Clean up previous stability timer if re-initializing
    if (s_stability_timer) {
        xTimerStop(s_stability_timer, 0);
        xTimerDelete(s_stability_timer, pdMS_TO_TICKS(100));
        s_stability_timer = NULL;
    }

    // Start rollback protection stability timer
    s_stability_timer = xTimerCreate("ota_stability", pdMS_TO_TICKS(OTA_STABILITY_TIMEOUT_MS),
                                     pdFALSE, NULL, ota_stability_timer_callback);
    if (s_stability_timer) {
        xTimerStart(s_stability_timer, 0);
        ESP_LOGI(TAG, "Rollback stability timer started (%d ms)", OTA_STABILITY_TIMEOUT_MS);
    }

    ESP_LOGI(TAG, "OTA handler initialized");
    return ESP_OK;
}

esp_err_t ota_handler_begin_update(const char *tag, const uint8_t *hash)
{
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

    ota_set_state(OTA_STATUS_IN_PROGRESS, 0, 0);

    // Run OTA in a separate task to avoid blocking I2C
    BaseType_t result = xTaskCreate(ota_update_task, "ota_update", OTA_TASK_STACK_SIZE, params,
                                    OTA_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA update task");
        free(params);
        ota_set_state(OTA_STATUS_FAILED, 0, 1);
        return ESP_FAIL;
    }

    return ESP_OK;
}

uint8_t ota_handler_get_progress(void)
{
    uint8_t val = 0;
    if (s_ota_mutex && xSemaphoreTake(s_ota_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        val = s_ota_progress;
        xSemaphoreGive(s_ota_mutex);
    }
    return val;
}

ota_status_t ota_handler_get_status(void)
{
    ota_status_t val = OTA_STATUS_IDLE;
    if (s_ota_mutex && xSemaphoreTake(s_ota_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        val = s_ota_status;
        xSemaphoreGive(s_ota_mutex);
    }
    return val;
}

uint8_t ota_handler_get_error_code(void)
{
    uint8_t val = 0;
    if (s_ota_mutex && xSemaphoreTake(s_ota_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        val = s_ota_error_code;
        xSemaphoreGive(s_ota_mutex);
    }
    return val;
}

esp_err_t ota_handler_confirm_valid(void)
{
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

static void ota_update_task(void *pvParameters)
{
    ota_task_params_t *params = (ota_task_params_t *)pvParameters;

    ESP_LOGI(TAG, "OTA update task started for tag: %s", params->tag);

    // Step 1: Ensure WiFi is connected
    ESP_LOGI(TAG, "Checking WiFi connectivity...");
    if (!wifi_manager_is_connected()) {
        ESP_LOGI(TAG, "WiFi not connected — initializing for OTA...");

        // WiFi manager should already be initialized from app_main
        esp_err_t ret = wifi_manager_connect(NULL, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi connection: %s", esp_err_to_name(ret));
            ota_set_state(OTA_STATUS_FAILED, 0, 2);  // WiFi connection error
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
            ota_set_state(OTA_STATUS_FAILED, 0, 2);
            goto cleanup;
        }
    }

    ESP_LOGI(TAG, "WiFi connected — proceeding with OTA download");
    ota_set_progress(5);

    // Step 2: Construct download URL from release tag
    char url[256];
    snprintf(url, sizeof(url), "https://github.com/%s/%s/releases/download/%s/robocar-main.bin",
             OTA_GITHUB_ORG, OTA_GITHUB_REPO, params->tag);

    ESP_LOGI(TAG, "Download URL: %s", url);
    ota_set_progress(10);

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
        ota_set_state(OTA_STATUS_FAILED, 0, 3);  // OTA begin error
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
            uint8_t pct = 10 + (uint8_t)((bytes_read * 85L) / image_size);
            ota_set_progress(pct);
        }

        uint8_t cur_progress = ota_handler_get_progress();
        if (cur_progress / 10 != last_logged_progress / 10 && cur_progress % 10 == 0) {
            ESP_LOGI(TAG, "OTA progress: %d%% (%d/%d bytes)", cur_progress, bytes_read, image_size);
            last_logged_progress = cur_progress;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_perform failed: %s", esp_err_to_name(ret));
        esp_err_t abort_ret = esp_https_ota_abort(ota_handle);
        if (abort_ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_https_ota_abort failed: %s", esp_err_to_name(abort_ret));
        }
        ota_set_state(OTA_STATUS_FAILED, 0, 4);  // Download/flash error
        goto cleanup;
    }

    // Finalize the OTA update
    ret = esp_https_ota_finish(ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(ret));
        ota_set_state(OTA_STATUS_FAILED, 0, 5);  // Finalization error
        goto cleanup;
    }

    // Verify hash prefix if provided (non-zero)
    bool hash_provided = false;
    for (int i = 0; i < OTA_HASH_LEN; i++) {
        if (params->hash[i] != 0) {
            hash_provided = true;
            break;
        }
    }

    if (hash_provided) {
        const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition) {
            esp_app_desc_t new_app_desc;
            ret = esp_ota_get_partition_description(update_partition, &new_app_desc);
            if (ret == ESP_OK) {
                if (memcmp(params->hash, new_app_desc.app_elf_sha256, OTA_HASH_LEN) != 0) {
                    ESP_LOGE(TAG, "Hash verification FAILED");
                    ESP_LOGE(TAG, "Expected: %02X%02X%02X%02X, Got: %02X%02X%02X%02X",
                             params->hash[0], params->hash[1], params->hash[2], params->hash[3],
                             new_app_desc.app_elf_sha256[0], new_app_desc.app_elf_sha256[1],
                             new_app_desc.app_elf_sha256[2], new_app_desc.app_elf_sha256[3]);
                    ota_set_state(OTA_STATUS_FAILED, 0, 6);  // Hash mismatch
                    esp_ota_mark_app_invalid_rollback_and_reboot();
                    goto cleanup;
                }
                ESP_LOGI(TAG, "Hash verification passed");
            } else {
                ESP_LOGW(TAG, "Could not read partition description for hash check");
            }
        }
    }

    ota_set_state(OTA_STATUS_SUCCESS, 100, 0);
    ESP_LOGI(TAG, "OTA update completed successfully!");
    ESP_LOGI(TAG, "Device will reboot when reboot command is received via I2C");

cleanup:
    if (ota_handler_get_status() == OTA_STATUS_FAILED) {
        ESP_LOGE(TAG, "OTA update failed with error code: %d", ota_handler_get_error_code());
    }
    free(params);
    vTaskDelete(NULL);
}

static void ota_stability_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Stability timeout reached — confirming firmware validity");
    ota_handler_confirm_valid();
}
