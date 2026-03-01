/**
 * @file mqtt_logger.c
 * @brief MQTT-based remote logging implementation for ESP32-CAM
 */

#include "mqtt_logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "cJSON.h"
#include "config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_logger";

// Internal structures
typedef struct {
    char *message;
    esp_log_level_t level;
    char *tag;
    int64_t timestamp;
} log_buffer_entry_t;

typedef struct {
    mqtt_logger_config_t config;
    esp_mqtt_client_handle_t client;
    QueueHandle_t log_queue;
    SemaphoreHandle_t mutex;
    mqtt_logger_stats_t stats;
    bool initialized;
    bool connected;
    TaskHandle_t log_task_handle;
} mqtt_logger_context_t;

static mqtt_logger_context_t s_context = {0};

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data);
static void log_processing_task(void *pvParameters);
static esp_err_t send_log_message(const mqtt_log_message_t *log_msg);
static int64_t get_timestamp_ms(void);
static char *create_log_json(const mqtt_log_message_t *log_msg);
static void cleanup_log_entry(log_buffer_entry_t *entry);
static esp_err_t queue_log_message(esp_log_level_t level, const char *tag, const char *message);

esp_err_t mqtt_logger_init(const mqtt_logger_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_context.initialized) {
        ESP_LOGW(TAG, "MQTT logger already initialized");
        return ESP_OK;
    }

    // Copy configuration
    memcpy(&s_context.config, config, sizeof(mqtt_logger_config_t));

    // Allocate strings
    s_context.config.broker_uri = strdup(config->broker_uri);
    s_context.config.client_id = strdup(config->client_id);
    s_context.config.log_topic = strdup(config->log_topic);
    s_context.config.status_topic = strdup(config->status_topic);
    s_context.config.command_topic = strdup(config->command_topic);

    if (config->username) {
        s_context.config.username = strdup(config->username);
    }
    if (config->password) {
        s_context.config.password = strdup(config->password);
    }

    // Create mutex
    s_context.mutex = xSemaphoreCreateMutex();
    if (!s_context.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Create log queue
    s_context.log_queue =
        xQueueCreate(config->buffer_size / sizeof(log_buffer_entry_t), sizeof(log_buffer_entry_t));
    if (!s_context.log_queue) {
        ESP_LOGE(TAG, "Failed to create log queue");
        vSemaphoreDelete(s_context.mutex);
        return ESP_ERR_NO_MEM;
    }

    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = s_context.config.broker_uri,
        .credentials.client_id = s_context.config.client_id,
        .credentials.username = s_context.config.username,
        .credentials.authentication.password = s_context.config.password,
        .session.keepalive = s_context.config.keepalive_interval,
        .session.last_will.topic = s_context.config.status_topic,
        .session.last_will.msg = "{\"status\":\"offline\",\"timestamp\":0}",
        .session.last_will.msg_len = 0,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .network.reconnect_timeout_ms = 5000,
        .network.timeout_ms = 10000,
    };

    s_context.client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_context.client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        vQueueDelete(s_context.log_queue);
        vSemaphoreDelete(s_context.mutex);
        return ESP_ERR_NO_MEM;
    }

    // Register event handler
    esp_mqtt_client_register_event(s_context.client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Start MQTT client
    esp_err_t ret = esp_mqtt_client_start(s_context.client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(s_context.client);
        vQueueDelete(s_context.log_queue);
        vSemaphoreDelete(s_context.mutex);
        return ret;
    }

    // Create log processing task
    BaseType_t task_ret = xTaskCreate(log_processing_task, "mqtt_log_task", 4096, NULL, 5,
                                      &s_context.log_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create log processing task");
        esp_mqtt_client_stop(s_context.client);
        esp_mqtt_client_destroy(s_context.client);
        vQueueDelete(s_context.log_queue);
        vSemaphoreDelete(s_context.mutex);
        return ESP_ERR_NO_MEM;
    }

    // Initialize stats
    memset(&s_context.stats, 0, sizeof(mqtt_logger_stats_t));

    s_context.initialized = true;
    s_context.connected = false;

    ESP_LOGI(TAG, "MQTT logger initialized successfully");
    ESP_LOGI(TAG, "Broker: %s", s_context.config.broker_uri);
    ESP_LOGI(TAG, "Client ID: %s", s_context.config.client_id);
    ESP_LOGI(TAG, "Log topic: %s", s_context.config.log_topic);

    return ESP_OK;
}

esp_err_t mqtt_logger_deinit(void)
{
    if (!s_context.initialized) {
        return ESP_OK;
    }

    // Stop and destroy MQTT client
    if (s_context.client) {
        esp_mqtt_client_stop(s_context.client);
        esp_mqtt_client_destroy(s_context.client);
        s_context.client = NULL;
    }

    // Delete task
    if (s_context.log_task_handle) {
        vTaskDelete(s_context.log_task_handle);
        s_context.log_task_handle = NULL;
    }

    // Clear queue
    if (s_context.log_queue) {
        log_buffer_entry_t entry;
        while (xQueueReceive(s_context.log_queue, &entry, 0) == pdTRUE) {
            cleanup_log_entry(&entry);
        }
        vQueueDelete(s_context.log_queue);
        s_context.log_queue = NULL;
    }

    // Delete mutex
    if (s_context.mutex) {
        vSemaphoreDelete(s_context.mutex);
        s_context.mutex = NULL;
    }

    // Free configuration strings
    free((void *)s_context.config.broker_uri);
    free((void *)s_context.config.client_id);
    free((void *)s_context.config.log_topic);
    free((void *)s_context.config.status_topic);
    free((void *)s_context.config.command_topic);
    if (s_context.config.username)
        free((void *)s_context.config.username);
    if (s_context.config.password)
        free((void *)s_context.config.password);

    memset(&s_context, 0, sizeof(s_context));

    ESP_LOGI(TAG, "MQTT logger deinitialized");
    return ESP_OK;
}

esp_err_t mqtt_logger_log(esp_log_level_t level, const char *tag, const char *format, va_list args)
{
    if (!s_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (level > s_context.config.min_level) {
        return ESP_OK;  // Skip logging if below minimum level
    }

    // Format message
    char *message = NULL;
    int len = vasprintf(&message, format, args);
    if (len < 0 || !message) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = queue_log_message(level, tag, message);
    free(message);

    return ret;
}

esp_err_t mqtt_logger_logf(esp_log_level_t level, const char *tag, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    esp_err_t ret = mqtt_logger_log(level, tag, format, args);
    va_end(args);
    return ret;
}

esp_err_t mqtt_logger_publish_status(const char *status_json)
{
    if (!s_context.initialized || !s_context.connected) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!status_json) {
        return ESP_ERR_INVALID_ARG;
    }

    int msg_id = esp_mqtt_client_publish(
        s_context.client, s_context.config.status_topic, status_json, strlen(status_json),
        s_context.config.qos_level, s_context.config.retain_status);

    if (msg_id < 0) {
        xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
        s_context.stats.messages_failed++;
        xSemaphoreGive(s_context.mutex);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mqtt_logger_get_stats(mqtt_logger_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
    memcpy(stats, &s_context.stats, sizeof(mqtt_logger_stats_t));
    stats->messages_buffered = uxQueueMessagesWaiting(s_context.log_queue);
    stats->is_connected = s_context.connected;
    xSemaphoreGive(s_context.mutex);

    return ESP_OK;
}

esp_err_t mqtt_logger_set_level(esp_log_level_t level)
{
    if (!s_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
    s_context.config.min_level = level;
    xSemaphoreGive(s_context.mutex);

    ESP_LOGI(TAG, "MQTT log level set to %d", level);
    return ESP_OK;
}

bool mqtt_logger_is_connected(void)
{
    return s_context.initialized && s_context.connected;
}

esp_err_t mqtt_logger_reconnect(void)
{
    if (!s_context.initialized || !s_context.client) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_mqtt_client_reconnect(s_context.client);
}

esp_err_t mqtt_logger_flush(void)
{
    if (!s_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Signal the log processing task to process all queued messages
    if (s_context.log_task_handle) {
        xTaskNotifyGive(s_context.log_task_handle);
    }

    return ESP_OK;
}

// Private functions

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_context.connected = true;

            // Subscribe to command topic
            if (s_context.config.command_topic) {
                esp_mqtt_client_subscribe(s_context.client, s_context.config.command_topic, 1);
            }

            // Publish online status
            char status_msg[128];
            snprintf(status_msg, sizeof(status_msg), "{\"status\":\"online\",\"timestamp\":%lld}",
                     get_timestamp_ms());
            mqtt_logger_publish_status(status_msg);

            xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
            s_context.stats.reconnect_count++;
            xSemaphoreGive(s_context.mutex);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_context.connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
            s_context.stats.messages_sent++;
            s_context.stats.last_message_time = get_timestamp_ms();
            xSemaphoreGive(s_context.mutex);
            break;

        case MQTT_EVENT_DATA:
            // Handle remote commands
            if (strncmp(event->topic, s_context.config.command_topic, event->topic_len) == 0) {
                ESP_LOGI(TAG, "Received command: %.*s", event->data_len, event->data);
                // TODO: Implement command processing
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            xSemaphoreTake(s_context.mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS));
            s_context.stats.messages_failed++;
            xSemaphoreGive(s_context.mutex);
            break;

        default:
            break;
    }
}

static void log_processing_task(void *pvParameters)
{
    log_buffer_entry_t entry;

    ESP_LOGI(TAG, "Log processing task started");

    while (1) {
        // Wait for log messages or notification
        if (xQueueReceive(s_context.log_queue, &entry, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (s_context.connected) {
                // Create log message structure
                mqtt_log_message_t log_msg = {
                    .timestamp = entry.timestamp,
                    .level = entry.level,
                    .tag = entry.tag,
                    .message = entry.message,
                    .component = "esp32cam",
                    .heap_free = esp_get_free_heap_size(),
                    .wifi_rssi = 0  // TODO: Get actual RSSI
                };

                // Get WiFi RSSI if available
                wifi_ap_record_t ap_info;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    log_msg.wifi_rssi = ap_info.rssi;
                }

                // Send log message
                esp_err_t ret = send_log_message(&log_msg);
                if (ret != ESP_OK) {
                    ESP_LOGD(TAG, "Failed to send log message, will retry later");
                    // Re-queue the message for retry (at front of queue)
                    if (xQueueSendToFront(s_context.log_queue, &entry, 0) != pdTRUE) {
                        cleanup_log_entry(&entry);
                    }
                } else {
                    cleanup_log_entry(&entry);
                }
            } else {
                // Not connected, keep message in queue for later
                if (xQueueSendToBack(s_context.log_queue, &entry, 0) != pdTRUE) {
                    cleanup_log_entry(&entry);
                }
            }
        }

        // Check for task notifications (flush request)
        if (ulTaskNotifyTake(pdFALSE, 0)) {
            ESP_LOGD(TAG, "Flush requested");
        }
    }
}

static esp_err_t send_log_message(const mqtt_log_message_t *log_msg)
{
    if (!s_context.connected) {
        return ESP_ERR_INVALID_STATE;
    }

    char *json_str = create_log_json(log_msg);
    if (!json_str) {
        return ESP_ERR_NO_MEM;
    }

    // Create topic with log level
    char topic[256];
    const char *level_str = "info";
    switch (log_msg->level) {
        case ESP_LOG_ERROR:
            level_str = "error";
            break;
        case ESP_LOG_WARN:
            level_str = "warn";
            break;
        case ESP_LOG_INFO:
            level_str = "info";
            break;
        case ESP_LOG_DEBUG:
            level_str = "debug";
            break;
        case ESP_LOG_VERBOSE:
            level_str = "verbose";
            break;
        default:
            free(json_str);
            return ESP_OK;
    }

    snprintf(topic, sizeof(topic), "%s/%s", s_context.config.log_topic, level_str);

    int msg_id = esp_mqtt_client_publish(s_context.client, topic, json_str, strlen(json_str),
                                         s_context.config.qos_level, false);

    free(json_str);

    if (msg_id < 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static int64_t get_timestamp_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static char *create_log_json(const mqtt_log_message_t *log_msg)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        return NULL;
    }

    cJSON_AddNumberToObject(json, "timestamp", log_msg->timestamp);
    cJSON_AddNumberToObject(json, "level", log_msg->level);
    cJSON_AddStringToObject(json, "tag", log_msg->tag);
    cJSON_AddStringToObject(json, "message", log_msg->message);
    cJSON_AddStringToObject(json, "component", log_msg->component);
    cJSON_AddNumberToObject(json, "heap_free", log_msg->heap_free);
    cJSON_AddNumberToObject(json, "wifi_rssi", log_msg->wifi_rssi);

    char *json_str = cJSON_Print(json);
    cJSON_Delete(json);

    return json_str;
}

static void cleanup_log_entry(log_buffer_entry_t *entry)
{
    if (entry->message) {
        free(entry->message);
        entry->message = NULL;
    }
    if (entry->tag) {
        free(entry->tag);
        entry->tag = NULL;
    }
}

static esp_err_t queue_log_message(esp_log_level_t level, const char *tag, const char *message)
{
    if (!s_context.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    log_buffer_entry_t entry = {.message = strdup(message),
                                .tag = strdup(tag),
                                .level = level,
                                .timestamp = get_timestamp_ms()};

    if (!entry.message || !entry.tag) {
        cleanup_log_entry(&entry);
        return ESP_ERR_NO_MEM;
    }

    if (xQueueSend(s_context.log_queue, &entry, 0) != pdTRUE) {
        cleanup_log_entry(&entry);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
