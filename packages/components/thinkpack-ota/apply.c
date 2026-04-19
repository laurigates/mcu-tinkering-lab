/**
 * @file apply.c
 * @brief ESP-only wrapper that writes a verified image to flash and
 *        reboots into it. Guarded behind ESP_PLATFORM so host tests
 *        skip this translation unit entirely.
 */

#ifdef ESP_PLATFORM

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "thinkpack_ota.h"

static const char *TAG = "thinkpack_ota_apply";

esp_err_t thinkpack_ota_apply(const ota_complete_payload_t *cp, const uint8_t *image, size_t len)
{
    if (cp == NULL || image == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!thinkpack_ota_sha256_verify(image, len, cp->final_sha256)) {
        ESP_LOGE(TAG, "Final SHA256 mismatch — refusing to apply image");
        return ESP_FAIL;
    }

    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (next == NULL) {
        ESP_LOGE(TAG, "No next OTA partition available");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Applying image v%u to partition %s (%u bytes)", (unsigned)cp->version,
             next->label, (unsigned)len);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(next, len, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ota_write(handle, image, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
        esp_ota_abort(handle);
        return err;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ota_set_boot_partition(next);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA image written; rebooting into new partition...");
    esp_restart();
    /* Never reached. */
    return ESP_OK;
}

#endif /* ESP_PLATFORM */
