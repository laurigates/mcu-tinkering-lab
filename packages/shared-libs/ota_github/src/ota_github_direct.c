/**
 * @file ota_github_direct.c
 * @brief TRIGGERED mode — direct HTTPS firmware download via esp_https_ota.
 *
 * Used by @ref ota_github_trigger_url (and its tag-based convenience wrapper)
 * in both PULL and TRIGGERED mode. Runs the download in a dedicated task so
 * the caller (typically an I2C/UART command dispatcher) is not blocked.
 *
 * Behaviour mirrors the reference implementation in
 * `robocar-main/main/ota_handler.c`, including the 4-byte SHA256 prefix
 * verification and the error-code taxonomy documented in ota_github.h.
 */

#include <stdlib.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ota_github.h"
#include "ota_github_events.h"
#include "ota_github_internal.h"

static const char *TAG = "ota_github.direct";

typedef struct {
    char url[256];
    uint8_t sha256_prefix[4];
    bool sha256_provided;
} direct_task_params_t;

static void post_failed(uint8_t error_code)
{
    ota_github_set_state(OTA_GITHUB_STATUS_FAILED, 0, error_code);
    ota_github_event_payload_t fp = {.error_code = error_code};
    ota_github_post_event(OTA_GITHUB_EVENT_FAILED, &fp);
}

static void direct_task(void *pvParameters)
{
    direct_task_params_t *p = (direct_task_params_t *)pvParameters;

    ESP_LOGI(TAG, "Direct OTA download: %s", p->url);

    /* Optional pre-download hook (WiFi-on-demand, VPN bring-up, …). */
    if (g_ota_github.cfg.hooks.pre_download_hook) {
        esp_err_t phret = g_ota_github.cfg.hooks.pre_download_hook(g_ota_github.cfg.hooks_user_ctx);
        if (phret != ESP_OK) {
            ESP_LOGE(TAG, "pre_download_hook failed: %s", esp_err_to_name(phret));
            post_failed(2);
            goto cleanup;
        }
    }

    ota_github_set_progress(5);

    esp_http_client_config_t http_config = {
        .url = p->url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = (int)g_ota_github.cfg.http_timeout_ms,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {.http_config = &http_config};

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ret = esp_https_ota_begin(&ota_config, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin: %s", esp_err_to_name(ret));
        post_failed(3);
        goto cleanup;
    }

    int image_size = esp_https_ota_get_image_size(ota_handle);
    ESP_LOGI(TAG, "Image size: %d bytes", image_size);
    ota_github_set_progress(10);

    uint8_t last_logged_tens = 0;
    while (true) {
        ret = esp_https_ota_perform(ota_handle);
        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        int bytes_read = esp_https_ota_get_image_len_read(ota_handle);
        if (image_size > 0) {
            uint8_t pct = 10 + (uint8_t)((bytes_read * 85L) / image_size);
            if (pct > 95) {
                pct = 95;
            }
            ota_github_set_progress(pct);
        }

        uint8_t cur = ota_github_get_progress();
        if (cur / 10 != last_logged_tens) {
            last_logged_tens = cur / 10;
            ESP_LOGI(TAG, "Progress: %u%% (%d/%d bytes)", (unsigned)cur, bytes_read, image_size);
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_perform: %s", esp_err_to_name(ret));
        esp_err_t abort_ret = esp_https_ota_abort(ota_handle);
        if (abort_ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_https_ota_abort: %s", esp_err_to_name(abort_ret));
        }
        post_failed(4);
        goto cleanup;
    }

    ret = esp_https_ota_finish(ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish: %s", esp_err_to_name(ret));
        post_failed(5);
        goto cleanup;
    }

    /* Optional SHA256-prefix verification against the embedded app_desc.
     * This catches corrupted or tampered downloads that still happen to be
     * valid ESP32 images. On mismatch we mark the partition invalid so the
     * bootloader rolls back on the next reset. */
    if (p->sha256_provided) {
        const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition) {
            esp_app_desc_t new_desc;
            ret = esp_ota_get_partition_description(update_partition, &new_desc);
            if (ret == ESP_OK) {
                if (memcmp(p->sha256_prefix, new_desc.app_elf_sha256, 4) != 0) {
                    ESP_LOGE(TAG,
                             "SHA256 prefix mismatch: expected %02X%02X%02X%02X got "
                             "%02X%02X%02X%02X",
                             p->sha256_prefix[0], p->sha256_prefix[1], p->sha256_prefix[2],
                             p->sha256_prefix[3], new_desc.app_elf_sha256[0],
                             new_desc.app_elf_sha256[1], new_desc.app_elf_sha256[2],
                             new_desc.app_elf_sha256[3]);
                    post_failed(6);
                    esp_ota_mark_app_invalid_rollback_and_reboot();
                    goto cleanup; /* unreachable but paranoid */
                }
                ESP_LOGI(TAG, "SHA256 prefix verification passed");
            } else {
                ESP_LOGW(TAG, "esp_ota_get_partition_description: %s — skipping hash check",
                         esp_err_to_name(ret));
            }
        }
    }

    ota_github_set_state(OTA_GITHUB_STATUS_SUCCESS, 100, 0);
    ota_github_post_event(OTA_GITHUB_EVENT_SUCCESS, NULL);
    ESP_LOGI(TAG, "Direct OTA completed successfully");

    /* TRIGGERED-mode callers typically want to control when the reboot
     * happens (e.g. wait for an I2C REBOOT command). We therefore do NOT
     * call esp_restart here; use the on_update_ready_to_reboot hook if
     * you want automatic restart. */
    if (g_ota_github.cfg.hooks.on_update_ready_to_reboot) {
        esp_err_t proceed =
            g_ota_github.cfg.hooks.on_update_ready_to_reboot(g_ota_github.cfg.hooks_user_ctx);
        if (proceed == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
    }

cleanup:
    free(p);
    vTaskDelete(NULL);
}

esp_err_t ota_github_direct_run(const char *url, const uint8_t sha256_prefix[4])
{
    direct_task_params_t *p = calloc(1, sizeof(*p));
    if (!p) {
        return ESP_ERR_NO_MEM;
    }
    strncpy(p->url, url, sizeof(p->url) - 1);
    if (sha256_prefix) {
        memcpy(p->sha256_prefix, sha256_prefix, 4);
        /* Treat an all-zero prefix as "no prefix supplied" — matches the
         * robocar-main convention for optional I2C-provided hashes. */
        for (int i = 0; i < 4; i++) {
            if (sha256_prefix[i] != 0) {
                p->sha256_provided = true;
                break;
            }
        }
    }

    ota_github_set_state(OTA_GITHUB_STATUS_IN_PROGRESS, 0, 0);

    BaseType_t rc = xTaskCreate(direct_task, "ota_github_dl", g_ota_github.cfg.task_stack_size, p,
                                g_ota_github.cfg.task_priority, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(ota_github_dl) failed");
        free(p);
        post_failed(1);
        return ESP_FAIL;
    }
    return ESP_OK;
}
