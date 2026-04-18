/**
 * @file ota_github_pull.c
 * @brief PULL mode — periodic polling of GitHub Releases via esp_ghota.
 *
 * esp_ghota already knows how to:
 *   - Query GitHub's /releases/latest endpoint
 *   - Pick the asset whose filename contains `filenamematch`
 *   - Parse semver from the tag and compare against the running app
 *   - Download + flash via esp_https_ota with the ESP-IDF cert bundle
 *
 * Our job is to wrap it with the component's state/event/hook plumbing and
 * to react to the ghota event stream.
 */

#include <stdbool.h>
#include <string.h>

#include "esp_event.h"
#include "esp_ghota.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ota_github.h"
#include "ota_github_events.h"
#include "ota_github_internal.h"

static const char *TAG = "ota_github.pull";

static ghota_client_handle_t *s_ghota_client = NULL;

static void ghota_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                                void *event_data)
{
    (void)arg;
    (void)event_base;

    switch (event_id) {
        case GHOTA_EVENT_UPDATE_AVAILABLE: {
            ESP_LOGI(TAG, "Update available — starting firmware download");
            ota_github_set_state(OTA_GITHUB_STATUS_IN_PROGRESS, 0, 0);

            ota_github_event_payload_t payload = {0};
            const char *version = event_data ? (const char *)event_data : "";
            strncpy(payload.version, version, sizeof(payload.version) - 1);
            ota_github_post_event(OTA_GITHUB_EVENT_UPDATE_AVAILABLE, &payload);

            if (g_ota_github.cfg.hooks.on_update_available) {
                g_ota_github.cfg.hooks.on_update_available(payload.version,
                                                           g_ota_github.cfg.hooks_user_ctx);
            }

            /* Optional pre-download hook — lets the app bring up WiFi etc. */
            if (g_ota_github.cfg.hooks.pre_download_hook) {
                esp_err_t phret =
                    g_ota_github.cfg.hooks.pre_download_hook(g_ota_github.cfg.hooks_user_ctx);
                if (phret != ESP_OK) {
                    ESP_LOGE(TAG, "pre_download_hook aborted update: %s", esp_err_to_name(phret));
                    ota_github_set_state(OTA_GITHUB_STATUS_FAILED, 0, 2);
                    ota_github_event_payload_t fp = {.error_code = 2};
                    ota_github_post_event(OTA_GITHUB_EVENT_FAILED, &fp);
                    break;
                }
            }

            esp_err_t ret = ghota_start_update(s_ghota_client);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ghota_start_update failed: %s", esp_err_to_name(ret));
                ota_github_set_state(OTA_GITHUB_STATUS_FAILED, 0, 3);
                ota_github_event_payload_t fp = {.error_code = 3};
                ota_github_post_event(OTA_GITHUB_EVENT_FAILED, &fp);
            }
            break;
        }

        case GHOTA_EVENT_NOUPDATE_AVAILABLE:
            ESP_LOGI(TAG, "No update available — firmware is current");
            ota_github_post_event(OTA_GITHUB_EVENT_NO_UPDATE, NULL);
            break;

        case GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS: {
            int progress = event_data ? *((int *)event_data) : 0;
            if (progress < 0) {
                progress = 0;
            }
            if (progress > 100) {
                progress = 100;
            }
            ota_github_set_progress((uint8_t)progress);
            break;
        }

        case GHOTA_EVENT_FINISH_FIRMWARE_UPDATE: {
            ESP_LOGI(TAG, "Firmware update complete — preparing to reboot");
            ota_github_set_state(OTA_GITHUB_STATUS_SUCCESS, 100, 0);
            ota_github_post_event(OTA_GITHUB_EVENT_SUCCESS, NULL);

            /* Let the app quiesce peripherals before we reboot. */
            esp_err_t proceed = ESP_OK;
            if (g_ota_github.cfg.hooks.on_update_ready_to_reboot) {
                proceed = g_ota_github.cfg.hooks.on_update_ready_to_reboot(
                    g_ota_github.cfg.hooks_user_ctx);
            }
            if (proceed != ESP_OK) {
                ESP_LOGW(TAG,
                         "on_update_ready_to_reboot aborted reboot (%s) — "
                         "update remains pending-verify",
                         esp_err_to_name(proceed));
                break;
            }

            /* Brief delay so any MQTT/log flush completes. */
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;
        }

        case GHOTA_EVENT_UPDATE_FAILED: {
            ESP_LOGE(TAG, "Firmware update failed");
            ota_github_set_state(OTA_GITHUB_STATUS_FAILED, 0, 4);
            ota_github_event_payload_t fp = {.error_code = 4};
            ota_github_post_event(OTA_GITHUB_EVENT_FAILED, &fp);
            break;
        }

        default:
            ESP_LOGD(TAG, "Unhandled ghota event: %d", (int)event_id);
            break;
    }
}

esp_err_t ota_github_pull_init(void)
{
    if (!g_ota_github.cfg.firmware_filename_match) {
        ESP_LOGE(TAG, "PULL mode requires cfg.firmware_filename_match");
        return ESP_ERR_INVALID_ARG;
    }

    ghota_config_t ghota_cfg = {
        .filenamematch = (char *)g_ota_github.cfg.firmware_filename_match,
        .hostname = "github.com",
        .orgname = (char *)g_ota_github.cfg.github_org,
        .reponame = (char *)g_ota_github.cfg.github_repo,
        .updateInterval = g_ota_github.cfg.poll_interval_min,
    };

    s_ghota_client = ghota_init(&ghota_cfg);
    if (!s_ghota_client) {
        ESP_LOGE(TAG, "ghota_init failed");
        return ESP_FAIL;
    }

    esp_err_t ret =
        esp_event_handler_register(GHOTA_EVENTS, ESP_EVENT_ANY_ID, &ghota_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register ghota event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    if (g_ota_github.cfg.poll_interval_min > 0) {
        ret = ghota_start_update_timer(s_ghota_client);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ghota_start_update_timer: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Poll timer started (%u min interval)",
                 (unsigned)g_ota_github.cfg.poll_interval_min);
    } else {
        ESP_LOGI(TAG, "Poll interval is 0 — only manual/MQTT checks will fire");
    }

    return ESP_OK;
}

/**
 * @brief Expose the underlying esp_ghota client handle.
 *
 * Escape hatch for callers who need to use esp_ghota APIs directly
 * (e.g. `ghota_get_latest_version` for peer-orchestration logic that
 * lives outside the component). Returns NULL if PULL mode was not
 * initialized.
 */
void *ota_github_pull_get_client_handle(void)
{
    return (void *)s_ghota_client;
}

esp_err_t ota_github_pull_check_now(void)
{
    if (!s_ghota_client) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_ota_github.status == OTA_GITHUB_STATUS_IN_PROGRESS) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Triggering manual update check");
    return ghota_check(s_ghota_client);
}
