/**
 * @file mqtt_logger.h
 * @brief MQTT-based remote logging for ESP32-CAM
 */

#ifndef MQTT_LOGGER_H
#define MQTT_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT Logger configuration structure
 */
typedef struct {
    const char *broker_uri;       ///< MQTT broker URI (e.g., "mqtt://192.168.0.100:1883")
    const char *client_id;        ///< MQTT client ID
    const char *username;         ///< MQTT username (NULL if not required)
    const char *password;         ///< MQTT password (NULL if not required)
    const char *log_topic;        ///< Base topic for logs
    const char *status_topic;     ///< Topic for status messages
    const char *command_topic;    ///< Topic to subscribe for remote commands
    uint32_t buffer_size;         ///< Log buffer size for offline storage
    uint32_t keepalive_interval;  ///< MQTT keepalive interval in seconds
    uint8_t qos_level;            ///< QoS level for log messages (0, 1, or 2)
    esp_log_level_t min_level;    ///< Minimum log level to send via MQTT
    bool retain_status;           ///< Whether to retain status messages
} mqtt_logger_config_t;

/**
 * @brief MQTT Logger statistics
 */
typedef struct {
    uint32_t messages_sent;      ///< Total messages sent successfully
    uint32_t messages_failed;    ///< Total messages that failed to send
    uint32_t messages_buffered;  ///< Messages currently in buffer
    uint32_t reconnect_count;    ///< Number of reconnections
    bool is_connected;           ///< Current connection status
    int64_t last_message_time;   ///< Timestamp of last sent message
} mqtt_logger_stats_t;

/**
 * @brief Log message structure for JSON formatting
 */
typedef struct {
    int64_t timestamp;      ///< Unix timestamp in milliseconds
    esp_log_level_t level;  ///< Log level
    const char *tag;        ///< Log tag
    const char *message;    ///< Log message
    const char *component;  ///< Component name (optional)
    uint32_t heap_free;     ///< Free heap memory
    int32_t wifi_rssi;      ///< WiFi signal strength
} mqtt_log_message_t;

/**
 * @brief Initialize MQTT logger
 *
 * @param config Configuration for MQTT logger
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_init(const mqtt_logger_config_t *config);

/**
 * @brief Deinitialize MQTT logger
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_deinit(void);

/**
 * @brief Send a log message via MQTT
 *
 * @param level Log level
 * @param tag Log tag
 * @param format Printf-style format string
 * @param args Variable arguments
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_log(esp_log_level_t level, const char *tag, const char *format, va_list args);

/**
 * @brief Send a formatted log message via MQTT
 *
 * @param level Log level
 * @param tag Log tag
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_logf(esp_log_level_t level, const char *tag, const char *format, ...);

/**
 * @brief Publish device status to MQTT
 *
 * @param status_json JSON string containing status information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_publish_status(const char *status_json);

/**
 * @brief MQTT subscription callback signature
 *
 * Invoked when a message arrives on a topic registered via
 * mqtt_logger_subscribe(). The callback runs from the MQTT event
 * task — keep work short and copy data if it must outlive the call.
 *
 * @param topic    Topic the message arrived on (NUL-terminated copy)
 * @param data     Message payload (NOT NUL-terminated; use data_len)
 * @param data_len Length of the payload in bytes
 */
typedef void (*mqtt_logger_subscribe_cb_t)(const char *topic, const char *data, int data_len);

/**
 * @brief Publish a message to an arbitrary MQTT topic
 *
 * @param topic   Topic to publish to (must not be NULL)
 * @param payload Message payload (NUL-terminated; may be NULL for empty)
 * @param qos     QoS level (0, 1, or 2)
 * @param retain  Whether the broker should retain the message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_publish(const char *topic, const char *payload, int qos, bool retain);

/**
 * @brief Subscribe to an MQTT topic with a callback
 *
 * Registers @p callback for messages arriving on @p topic. If the
 * client is connected the subscription is sent immediately; otherwise
 * it will be sent on the next connect. Wildcards (`+`, `#`) follow
 * MQTT topic-matching rules.
 *
 * @param topic    Topic filter to subscribe to (must not be NULL)
 * @param callback Callback invoked on each matching message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_subscribe(const char *topic, mqtt_logger_subscribe_cb_t callback);

/**
 * @brief Command-topic callback signature
 *
 * Invoked with an already bound-checked, NUL-terminated command line
 * received on the configured command_topic (see mqtt_command.h for what
 * "bound-checked" means: printable ASCII, in-bounds, and not a fragment of a
 * larger message). Runs from the MQTT event task — keep work short.
 *
 * @param line NUL-terminated command line, e.g. "F" or "plan resume".
 */
typedef void (*mqtt_logger_command_cb_t)(const char *line);

/**
 * @brief Register the handler for messages on the configured command_topic
 *
 * Distinct from mqtt_logger_subscribe(): the command topic is already
 * subscribed internally by mqtt_logger_init() at QoS 1, and this only wires
 * what happens when a message arrives on it. Call after mqtt_logger_init().
 * Passing NULL clears the handler — an inbound command is then logged and
 * dropped rather than silently ignored.
 *
 * @param callback Handler for validated command lines, or NULL to clear it
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not yet initialized
 */
esp_err_t mqtt_logger_set_command_handler(mqtt_logger_command_cb_t callback);

/**
 * @brief Get MQTT logger statistics
 *
 * @param stats Pointer to statistics structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_get_stats(mqtt_logger_stats_t *stats);

/**
 * @brief Set minimum log level for MQTT publishing
 *
 * @param level Minimum log level
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_set_level(esp_log_level_t level);

/**
 * @brief Get current MQTT connection status
 *
 * @return true if connected, false otherwise
 */
bool mqtt_logger_is_connected(void);

/**
 * @brief Force reconnection to MQTT broker
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_reconnect(void);

/**
 * @brief Flush all buffered log messages
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_logger_flush(void);

// Convenience macros for MQTT logging
#define MQTT_LOGE(tag, format, ...) mqtt_logger_logf(ESP_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define MQTT_LOGW(tag, format, ...) mqtt_logger_logf(ESP_LOG_WARN, tag, format, ##__VA_ARGS__)
#define MQTT_LOGI(tag, format, ...) mqtt_logger_logf(ESP_LOG_INFO, tag, format, ##__VA_ARGS__)
#define MQTT_LOGD(tag, format, ...) mqtt_logger_logf(ESP_LOG_DEBUG, tag, format, ##__VA_ARGS__)
#define MQTT_LOGV(tag, format, ...) mqtt_logger_logf(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif  // MQTT_LOGGER_H
