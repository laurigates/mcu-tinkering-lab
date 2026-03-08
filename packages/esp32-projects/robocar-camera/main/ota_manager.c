/**
 * @file ota_manager.c
 * @brief OTA update manager implementation for ESP32-CAM robocar
 *
 * Uses esp_ghota for GitHub release checking and firmware download,
 * with MQTT notifications for immediate update triggers. After
 * self-updating, orchestrates main controller update via I2C OTA commands.
 */

#include "ota_manager.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_event.h"
#include "esp_ghota.h"
#include "config.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#if MQTT_LOGGING_ENABLED
#include "mqtt_logger.h"
#endif

static const char *TAG = "OTA_Manager";

// OTA manager state
static ghota_client_handle_t *s_ghota_client = NULL;
static bool s_ota_in_progress = false;
static bool s_firmware_valid_confirmed = false;
static TimerHandle_t s_stability_timer = NULL;

// Forward declarations
static void ghota_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);
static void ota_stability_timer_callback(TimerHandle_t xTimer);
static void orchestrate_main_controller_update(void *pvParameters);

#if MQTT_LOGGING_ENABLED
static void mqtt_ota_notify_handler(const char *topic, const char *data,
                                    int data_len);
#endif

esp_err_t ota_manager_init(void) {
    ESP_LOGI(TAG, "Initializing OTA manager");
    ESP_LOGI(TAG, "Current firmware version: %s", ota_manager_get_version());

    // Configure esp_ghota
    ghota_config_t ghota_config = {
        .filenamematch = OTA_FIRMWARE_FILENAME_MATCH,
        .hostname = OTA_GITHUB_HOST,
        .orgname = OTA_GITHUB_ORG,
        .reponame = OTA_GITHUB_REPO,
        .updateInterval = OTA_CHECK_INTERVAL_MIN,
    };

    s_ghota_client = ghota_init(&ghota_config);
    if (!s_ghota_client) {
        ESP_LOGE(TAG, "Failed to initialize esp_ghota client");
        return ESP_FAIL;
    }

    // Register event handler for ghota events
    esp_err_t ret = esp_event_handler_register(GHOTA_EVENTS, ESP_EVENT_ANY_ID,
                                               &ghota_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ghota event handler: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Start periodic update timer
    ret = ghota_start_update_timer(s_ghota_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ghota update timer: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "OTA update timer started (interval: %d min)",
             OTA_CHECK_INTERVAL_MIN);

#if MQTT_LOGGING_ENABLED
    // Subscribe to MQTT OTA notification topic
    mqtt_logger_subscribe(OTA_MQTT_NOTIFY_TOPIC, mqtt_ota_notify_handler);
    ESP_LOGI(TAG, "Subscribed to MQTT OTA topic: %s", OTA_MQTT_NOTIFY_TOPIC);
#endif

    // Clean up previous stability timer if re-initializing
    if (s_stability_timer) {
        xTimerStop(s_stability_timer, 0);
        xTimerDelete(s_stability_timer, pdMS_TO_TICKS(100));
        s_stability_timer = NULL;
    }

    // Start rollback protection stability timer
    s_stability_timer = xTimerCreate(
        "ota_stability", pdMS_TO_TICKS(OTA_STABILITY_TIMEOUT_MS), pdFALSE,
        NULL, ota_stability_timer_callback);
    if (s_stability_timer) {
        xTimerStart(s_stability_timer, 0);
        ESP_LOGI(TAG, "Rollback stability timer started (%d ms)",
                 OTA_STABILITY_TIMEOUT_MS);
    }

    ESP_LOGI(TAG, "OTA manager initialized successfully");
    return ESP_OK;
}

esp_err_t ota_manager_check_update(void) {
    if (!s_ghota_client) {
        ESP_LOGE(TAG, "OTA manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ota_in_progress) {
        ESP_LOGW(TAG, "OTA update already in progress");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Triggering manual update check");
    return ghota_check(s_ghota_client);
}

const char *ota_manager_get_version(void) {
    const esp_app_desc_t *app_desc = esp_app_get_description();
    return app_desc->version;
}

esp_err_t ota_manager_confirm_valid(void) {
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

static void ghota_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    switch (event_id) {
        case GHOTA_EVENT_UPDATE_AVAILABLE: {
            ESP_LOGI(TAG, "Update available! Starting firmware download...");
            s_ota_in_progress = true;

#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC,
                                "{\"status\":\"downloading\"}", 1, false);
#endif
            // esp_ghota handles the download and flash automatically
            esp_err_t update_ret = ghota_start_update(s_ghota_client);
            if (update_ret != ESP_OK) {
                ESP_LOGE(TAG, "ghota_start_update failed: %s",
                         esp_err_to_name(update_ret));
                s_ota_in_progress = false;
            }
            break;
        }

        case GHOTA_EVENT_NOUPDATE_AVAILABLE:
            ESP_LOGI(TAG, "No update available — firmware is current");
            break;

        case GHOTA_EVENT_START_STORAGE_UPDATE:
            ESP_LOGI(TAG, "Storage partition update starting...");
            break;

        case GHOTA_EVENT_FINISH_STORAGE_UPDATE:
            ESP_LOGI(TAG, "Storage partition update complete");
            break;

        case GHOTA_EVENT_START_FIRMWARE_UPDATE:
            ESP_LOGI(TAG, "Firmware update starting...");
            break;

        case GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS: {
            int progress = *((int *)event_data);
            if (progress % 10 == 0) {
                ESP_LOGI(TAG, "Firmware download progress: %d%%", progress);
            }
            break;
        }

        case GHOTA_EVENT_FINISH_FIRMWARE_UPDATE:
            ESP_LOGI(TAG, "Firmware update complete — rebooting...");
            s_ota_in_progress = false;

#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC,
                                "{\"status\":\"rebooting\"}", 1, false);
#endif

            // Brief delay to allow MQTT message to send
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
            break;

        case GHOTA_EVENT_UPDATE_FAILED:
            ESP_LOGE(TAG, "Firmware update failed");
            s_ota_in_progress = false;

#if MQTT_LOGGING_ENABLED
            mqtt_logger_publish(OTA_MQTT_STATUS_TOPIC,
                                "{\"status\":\"failed\"}", 1, false);
#endif
            break;

        case GHOTA_EVENT_PENDING_REBOOT:
            ESP_LOGI(TAG, "Reboot pending after successful update");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled ghota event: %d", (int)event_id);
            break;
    }
}

static void ota_stability_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Stability timeout reached — confirming firmware validity");
    ota_manager_confirm_valid();

    // After confirming self-update, check if main controller needs updating
    // This runs as a separate task to avoid blocking the timer callback
    xTaskCreate(orchestrate_main_controller_update,
                "ota_main_ctrl", OTA_TASK_STACK_SIZE, NULL,
                OTA_TASK_PRIORITY, NULL);
}

static void orchestrate_main_controller_update(void *pvParameters) {
    ESP_LOGI(TAG, "Checking if main controller needs OTA update...");

    // Query main controller version via I2C
    i2c_command_packet_t cmd;
    static uint8_t seq = 0;
    prepare_get_version_command(&cmd, seq++);

    i2c_response_packet_t response;
    esp_err_t ret = i2c_master_send_command(&cmd, &response);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to query main controller version: %s",
                 esp_err_to_name(ret));
        goto done;
    }

    if (response.status != 0x00 ||
        response.data_length < sizeof(version_response_t)) {
        ESP_LOGW(TAG, "Invalid version response from main controller");
        goto done;
    }

    version_response_t *version = (version_response_t *)response.data;
    ESP_LOGI(TAG, "Main controller firmware version: %s", version->version);

    // TODO: Compare main controller version against latest release
    // For now, log the version. Full orchestration (entering maintenance mode,
    // sending OTA URL via I2C, polling status) will be implemented when
    // the main controller's ota_handler is wired up to actually perform
    // the download.

done:
    vTaskDelete(NULL);
}

#if MQTT_LOGGING_ENABLED
static void mqtt_ota_notify_handler(const char *topic, const char *data,
                                    int data_len) {
    ESP_LOGI(TAG, "MQTT OTA notification received on topic: %s", topic);
    ESP_LOGI(TAG, "Notification data: %.*s", data_len, data);

    // Rate limit: at most one check per 60 seconds
    static int64_t s_last_ota_check_time = 0;
    int64_t now = esp_timer_get_time();
    if (s_last_ota_check_time > 0 &&
        (now - s_last_ota_check_time) < 60 * 1000000LL) {
        ESP_LOGW(TAG, "MQTT OTA trigger rate-limited, skipping");
        return;
    }
    s_last_ota_check_time = now;

    // Trigger update check via esp_ghota
    esp_err_t ret = ota_manager_check_update();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to trigger update check: %s",
                 esp_err_to_name(ret));
    }
}
#endif
