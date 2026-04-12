/**
 * @file ota_manager.c
 * @brief Robocar-camera OTA manager — thin wrapper around the shared
 *        `ota_github` component.
 *
 * The generic polling / download / rollback logic lives in
 * `packages/shared-libs/ota_github/`. This file only contains the
 * project-specific glue:
 *
 *   - Robocar-specific configuration (asset filename, poll interval,
 *     MQTT topic)
 *   - Post-update orchestration of the main controller via I2C
 *   - Optional status publishes via the project's `mqtt_logger`
 */

#include "ota_manager.h"

#include <string.h>

#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#include "ota_github.h"
#include "ota_github_events.h"
#include "semver.h"

#if MQTT_LOGGING_ENABLED
#include "mqtt_logger.h"
#endif

static const char *TAG = "OTA_Manager";

/* Polling constants for main controller OTA orchestration. Kept here because
 * they are robocar-specific and not part of the shared component's remit. */
#define OTA_VERSION_POLL_INTERVAL_MS 1000
#define OTA_VERSION_POLL_MAX_ATTEMPTS 30
#define OTA_MAINTENANCE_SETTLE_MS 2000
#define OTA_STATUS_POLL_INTERVAL_MS 5000
#define OTA_STATUS_TIMEOUT_MS (5 * 60 * 1000)

static void orchestrate_main_controller_update(void *pvParameters);
static void ota_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);
#if MQTT_LOGGING_ENABLED
static void mqtt_ota_notify_handler(const char *topic, const char *data, int data_len);
#endif

esp_err_t ota_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing OTA manager");

    ota_github_config_t cfg = OTA_GITHUB_CONFIG_DEFAULT();
    cfg.mode = OTA_GITHUB_MODE_PULL;
    cfg.github_org = OTA_GITHUB_ORG;
    cfg.github_repo = OTA_GITHUB_REPO;
    cfg.firmware_filename_match = OTA_FIRMWARE_FILENAME_MATCH;
    cfg.poll_interval_min = OTA_CHECK_INTERVAL_MIN;
    cfg.stability_timeout_ms = OTA_STABILITY_TIMEOUT_MS;
    cfg.http_timeout_ms = OTA_HTTP_TIMEOUT_MS;
    cfg.task_stack_size = OTA_TASK_STACK_SIZE;
    cfg.task_priority = OTA_TASK_PRIORITY;

    esp_err_t ret = ota_github_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ota_github_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Observe events so we can run robocar-specific orchestration when
     * the camera's own firmware is confirmed healthy after boot, and so
     * we can publish MQTT status messages. */
    ret = esp_event_handler_register(OTA_GITHUB_EVENTS, ESP_EVENT_ANY_ID, &ota_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "register ota_github event handler: %s", esp_err_to_name(ret));
    }

#if MQTT_LOGGING_ENABLED
    /* Preserve the pre-existing MQTT notify subscription so the CI push
     * still triggers an immediate check. The mqtt_logger module is
     * project-specific and stays in the camera firmware. */
    mqtt_logger_subscribe(OTA_MQTT_NOTIFY_TOPIC, mqtt_ota_notify_handler);
    ESP_LOGI(TAG, "Subscribed to MQTT OTA topic: %s", OTA_MQTT_NOTIFY_TOPIC);
#endif

    ESP_LOGI(TAG, "OTA manager initialized (version: %s)", ota_manager_get_version());
    return ESP_OK;
}

esp_err_t ota_manager_check_update(void)
{
    return ota_github_check_now();
}

const char *ota_manager_get_version(void)
{
    return ota_github_get_version();
}

esp_err_t ota_manager_confirm_valid(void)
{
    return ota_github_confirm_valid();
}

/* --------------------------------------------------------------------------
 * Event bridge — reacts to ota_github lifecycle on the robocar side.
 * -------------------------------------------------------------------------- */

static void ota_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;

    switch ((ota_github_event_id_t)id) {
        case OTA_GITHUB_EVENT_UPDATE_AVAILABLE:
#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC, "{\"status\":\"downloading\"}", 1, false);
#endif
            break;

        case OTA_GITHUB_EVENT_SUCCESS:
#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC, "{\"status\":\"rebooting\"}", 1, false);
#endif
            break;

        case OTA_GITHUB_EVENT_FAILED:
#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC, "{\"status\":\"failed\"}", 1, false);
#endif
            break;

        case OTA_GITHUB_EVENT_VALID_CONFIRMED:
            /* Camera is confirmed healthy — now see if the main controller
             * needs updating too. Run as a separate task to keep the event
             * callback fast. */
            xTaskCreate(orchestrate_main_controller_update, "ota_main_ctrl", OTA_TASK_STACK_SIZE,
                        NULL, OTA_TASK_PRIORITY, NULL);
            break;

        default:
            break;
    }
}

#if MQTT_LOGGING_ENABLED
/* Retained for backward compatibility with the pre-refactor layout — the
 * module's forward-declared notify handler is still wired through
 * mqtt_logger_subscribe above. */
static void mqtt_ota_notify_handler(const char *topic, const char *data, int data_len)
{
    (void)topic;
    (void)data;
    (void)data_len;

    /* Rate limit: at most one check per 60 seconds. */
    static int64_t s_last_ota_check_time = 0;
    int64_t now = esp_timer_get_time();
    if (s_last_ota_check_time > 0 && (now - s_last_ota_check_time) < 60 * 1000000LL) {
        ESP_LOGW(TAG, "MQTT OTA trigger rate-limited, skipping");
        return;
    }
    s_last_ota_check_time = now;

    esp_err_t ret = ota_manager_check_update();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to trigger update check: %s", esp_err_to_name(ret));
    }
}
#endif

/* --------------------------------------------------------------------------
 * Dual-controller OTA orchestration — robocar-specific.
 *
 * After the camera has booted its new firmware and the stability window
 * has elapsed, we check whether the main controller is also on the
 * latest release. If not, we put it into maintenance mode, forward the
 * release tag via I2C, poll for status, and send a reboot command on
 * success.
 *
 * NOTE: `ghota_get_latest_version` is exposed by esp_ghota and accessible
 * because the shared component declares esp_ghota in its idf_component.yml.
 * -------------------------------------------------------------------------- */

#include "esp_ghota.h"

/* esp_ghota's client handle is encapsulated inside the shared component.
 * We reach in via the documented escape-hatch accessor so we can reuse
 * esp_ghota's cached semver helpers (ghota_get_latest_version, etc.). */

static void orchestrate_main_controller_update(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Checking if main controller needs OTA update...");

    /* Step 1: Get main controller version via I2C helper. */
    char mc_version_str[VERSION_STRING_LEN];
    esp_err_t ret = i2c_get_version(mc_version_str, sizeof(mc_version_str));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to query main controller version: %s", esp_err_to_name(ret));
        goto done;
    }
    ESP_LOGI(TAG, "Main controller firmware version: %s", mc_version_str);

    semver_t mc_version = {0};
    if (semver_parse(mc_version_str, &mc_version) != 0) {
        ESP_LOGW(TAG, "Failed to parse main controller version: %s", mc_version_str);
        goto done;
    }

    /* Step 2: Get latest release info from esp_ghota. Trigger a check if
     * we don't have cached data yet. */
    ghota_client_handle_t *client = (ghota_client_handle_t *)ota_github_pull_get_client_handle();
    if (!client) {
        ESP_LOGW(TAG, "esp_ghota client not available — cannot orchestrate");
        semver_free(&mc_version);
        goto done;
    }

    semver_t *latest = ghota_get_latest_version(client);
    if (!latest || (!latest->major && !latest->minor && !latest->patch)) {
        ESP_LOGI(TAG, "No cached release info — triggering GitHub check...");
        ghota_check(client);

        for (int i = 0; i < OTA_VERSION_POLL_MAX_ATTEMPTS; i++) {
            vTaskDelay(pdMS_TO_TICKS(OTA_VERSION_POLL_INTERVAL_MS));
            latest = ghota_get_latest_version(client);
            if (latest && (latest->major || latest->minor || latest->patch)) {
                break;
            }
        }
    }
    if (!latest || (!latest->major && !latest->minor && !latest->patch)) {
        ESP_LOGW(TAG, "Could not determine latest release version");
        semver_free(&mc_version);
        goto done;
    }

    char latest_str[32];
    semver_render(latest, latest_str);
    ESP_LOGI(TAG, "Latest release version: %s", latest_str);

    /* Step 3: Skip if the main controller is already up to date. */
    if (!semver_gt(*latest, mc_version)) {
        ESP_LOGI(TAG, "Main controller is up to date (v%s)", mc_version_str);
        semver_free(&mc_version);
        goto done;
    }

    ESP_LOGI(TAG, "Main controller needs update: v%s -> v%s", mc_version_str, latest_str);

    /* Step 4: Maintenance mode (stops motors on main controller). */
    ret = i2c_send_enter_maintenance_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter maintenance mode: %s", esp_err_to_name(ret));
        semver_free(&mc_version);
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(OTA_MAINTENANCE_SETTLE_MS));

    /* Step 5: Send OTA begin with release tag. */
    char release_tag[OTA_TAG_MAX_LEN];
    snprintf(release_tag, sizeof(release_tag), "v%s", latest_str);

    ret = i2c_send_begin_ota(release_tag, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start main controller OTA: %s", esp_err_to_name(ret));
        semver_free(&mc_version);
        goto done;
    }

    /* Step 6: Poll main controller OTA status until success, failure, or timeout. */
    const int max_polls = OTA_STATUS_TIMEOUT_MS / OTA_STATUS_POLL_INTERVAL_MS;
    for (int i = 0; i < max_polls; i++) {
        vTaskDelay(pdMS_TO_TICKS(OTA_STATUS_POLL_INTERVAL_MS));

        ota_status_response_t ota_status;
        ret = i2c_get_ota_status(&ota_status);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to poll OTA status (%d/%d)", i + 1, max_polls);
            continue;
        }

        ESP_LOGI(TAG, "Main controller OTA progress: %d%% (status: %d)", ota_status.progress,
                 ota_status.status);

        if (ota_status.status == OTA_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Main controller OTA successful — sending reboot");
            i2c_send_reboot();
            semver_free(&mc_version);
            goto done;
        }
        if (ota_status.status == OTA_STATUS_FAILED) {
            ESP_LOGE(TAG, "Main controller OTA failed (error: %d)", ota_status.error_code);
            semver_free(&mc_version);
            goto done;
        }
    }

    ESP_LOGE(TAG, "Main controller OTA timed out after %d seconds", OTA_STATUS_TIMEOUT_MS / 1000);
    semver_free(&mc_version);

done:
    vTaskDelete(NULL);
}
