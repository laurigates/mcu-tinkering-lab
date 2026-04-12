/**
 * @file ota_github.c
 * @brief Public API + global state + rollback-stability timer.
 *
 * The bulk of the OTA work lives in the mode-specific translation units
 * (ota_github_pull.c, ota_github_direct.c, ota_github_mqtt.c). This file
 * owns the one-time initialization, the state mutex, and the manual
 * "confirm valid" path.
 */

#include "ota_github.h"
#include "ota_github_events.h"
#include "ota_github_internal.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

ESP_EVENT_DEFINE_BASE(OTA_GITHUB_EVENTS);

static const char *TAG = "ota_github";

/** Global component state. Zero-initialized at program load. */
ota_github_state_t g_ota_github;

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

void ota_github_set_state(ota_github_status_t status, uint8_t progress, uint8_t error_code)
{
    if (!g_ota_github.mutex) {
        return;
    }
    if (xSemaphoreTake(g_ota_github.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    g_ota_github.status = status;
    g_ota_github.progress = progress;
    g_ota_github.error_code = error_code;
    xSemaphoreGive(g_ota_github.mutex);
}

void ota_github_set_progress(uint8_t progress)
{
    if (!g_ota_github.mutex) {
        return;
    }
    if (xSemaphoreTake(g_ota_github.mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    g_ota_github.progress = progress;
    xSemaphoreGive(g_ota_github.mutex);

    /* Emit a progress event and fire the optional hook. */
    ota_github_event_payload_t payload = {.progress = progress};
    ota_github_post_event(OTA_GITHUB_EVENT_PROGRESS, &payload);

    if (g_ota_github.cfg.hooks.on_progress) {
        g_ota_github.cfg.hooks.on_progress(progress, g_ota_github.cfg.hooks_user_ctx);
    }
}

void ota_github_post_event(ota_github_event_id_t id, const ota_github_event_payload_t *payload)
{
    ota_github_event_payload_t empty = {0};
    const ota_github_event_payload_t *p = payload ? payload : &empty;
    (void)esp_event_post(OTA_GITHUB_EVENTS, (int32_t)id, p, sizeof(*p), pdMS_TO_TICKS(100));
}

/* --------------------------------------------------------------------------
 * Stability timer — cancels rollback after `stability_timeout_ms` of
 * healthy uptime. This is the standard ESP-IDF "mark valid" pattern.
 * -------------------------------------------------------------------------- */

static void stability_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGI(TAG, "Stability timeout reached — confirming firmware validity");
    ota_github_confirm_valid();
}

esp_err_t ota_github_confirm_valid(void)
{
    if (!g_ota_github.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_ota_github.firmware_valid_confirmed) {
        return ESP_OK;
    }

    esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret == ESP_OK) {
        g_ota_github.firmware_valid_confirmed = true;
        ESP_LOGI(TAG, "Firmware marked as valid — rollback cancelled");
        ota_github_post_event(OTA_GITHUB_EVENT_VALID_CONFIRMED, NULL);
    } else {
        /* Not booted from OTA partition — that's fine on first flash. */
        ESP_LOGW(TAG, "esp_ota_mark_app_valid_cancel_rollback: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ota_github_init(const ota_github_config_t *cfg)
{
    if (!cfg || !cfg->github_org || !cfg->github_repo) {
        return ESP_ERR_INVALID_ARG;
    }
    if (g_ota_github.initialized) {
        ESP_LOGW(TAG, "ota_github_init called twice — ignoring");
        return ESP_ERR_INVALID_STATE;
    }

    /* Fill defaults for any zero-valued timing fields. */
    g_ota_github.cfg = *cfg;
    if (g_ota_github.cfg.stability_timeout_ms == 0) {
        g_ota_github.cfg.stability_timeout_ms = 60000;
    }
    if (g_ota_github.cfg.http_timeout_ms == 0) {
        g_ota_github.cfg.http_timeout_ms = 30000;
    }
    if (g_ota_github.cfg.task_stack_size == 0) {
        g_ota_github.cfg.task_stack_size = 8192;
    }
    if (g_ota_github.cfg.task_priority == 0) {
        g_ota_github.cfg.task_priority = 5;
    }
    if (g_ota_github.cfg.mode == OTA_GITHUB_MODE_PULL && g_ota_github.cfg.poll_interval_min == 0) {
        g_ota_github.cfg.poll_interval_min = 360;  /* 6 hours */
    }

    g_ota_github.mutex = xSemaphoreCreateMutex();
    if (!g_ota_github.mutex) {
        return ESP_ERR_NO_MEM;
    }
    g_ota_github.status = OTA_GITHUB_STATUS_IDLE;

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "Initializing ota_github v%s (mode=%s, repo=%s/%s)",
             app_desc ? app_desc->version : "?",
             cfg->mode == OTA_GITHUB_MODE_PULL ? "PULL" : "TRIGGERED",
             cfg->github_org, cfg->github_repo);

    /* Mode-specific bring-up. */
    esp_err_t ret = ESP_OK;
    if (cfg->mode == OTA_GITHUB_MODE_PULL) {
        ret = ota_github_pull_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PULL init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Optional MQTT push-notify works for both modes. */
    if (cfg->mqtt_enabled && cfg->mqtt_client && cfg->mqtt_notify_topic) {
        ret = ota_github_mqtt_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "MQTT notify setup failed: %s (non-fatal)", esp_err_to_name(ret));
        }
    }

    /* Start rollback-stability timer. */
    g_ota_github.stability_timer =
        xTimerCreate("ota_stability", pdMS_TO_TICKS(g_ota_github.cfg.stability_timeout_ms),
                     pdFALSE, NULL, stability_timer_cb);
    if (g_ota_github.stability_timer) {
        xTimerStart(g_ota_github.stability_timer, 0);
        ESP_LOGI(TAG, "Rollback stability timer started (%u ms)",
                 (unsigned)g_ota_github.cfg.stability_timeout_ms);
    }

    g_ota_github.initialized = true;
    ESP_LOGI(TAG, "ota_github initialized");
    return ESP_OK;
}

esp_err_t ota_github_check_now(void)
{
    if (!g_ota_github.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (g_ota_github.cfg.mode != OTA_GITHUB_MODE_PULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ota_github_pull_check_now();
}

esp_err_t ota_github_trigger_url(const char *url, const uint8_t sha256_prefix[4])
{
    if (!g_ota_github.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!url || !*url) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ota_github_get_status() == OTA_GITHUB_STATUS_IN_PROGRESS) {
        return ESP_ERR_INVALID_STATE;
    }
    return ota_github_direct_run(url, sha256_prefix);
}

esp_err_t ota_github_trigger_tag(const char *asset_override, const char *tag,
                                 const uint8_t sha256_prefix[4])
{
    if (!g_ota_github.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!tag || !*tag) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *asset = asset_override ? asset_override : g_ota_github.cfg.triggered_asset_filename;
    if (!asset || !*asset) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[256];
    int n = snprintf(url, sizeof(url), "https://github.com/%s/%s/releases/download/%s/%s",
                     g_ota_github.cfg.github_org, g_ota_github.cfg.github_repo, tag, asset);
    if (n < 0 || (size_t)n >= sizeof(url)) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ota_github_trigger_url(url, sha256_prefix);
}

uint8_t ota_github_get_progress(void)
{
    uint8_t v = 0;
    if (g_ota_github.mutex &&
        xSemaphoreTake(g_ota_github.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        v = g_ota_github.progress;
        xSemaphoreGive(g_ota_github.mutex);
    }
    return v;
}

ota_github_status_t ota_github_get_status(void)
{
    ota_github_status_t v = OTA_GITHUB_STATUS_IDLE;
    if (g_ota_github.mutex &&
        xSemaphoreTake(g_ota_github.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        v = g_ota_github.status;
        xSemaphoreGive(g_ota_github.mutex);
    }
    return v;
}

uint8_t ota_github_get_error_code(void)
{
    uint8_t v = 0;
    if (g_ota_github.mutex &&
        xSemaphoreTake(g_ota_github.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        v = g_ota_github.error_code;
        xSemaphoreGive(g_ota_github.mutex);
    }
    return v;
}

const char *ota_github_get_version(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    return app_desc ? app_desc->version : "";
}
