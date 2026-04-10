/**
 * @file unified_logger.h
 * @brief Unified logging system supporting both serial and MQTT output
 */

#ifndef UNIFIED_LOGGER_H
#define UNIFIED_LOGGER_H

#include "config.h"
#include "esp_log.h"

#if MQTT_LOGGING_ENABLED
#include "mqtt_logger.h"

// Enhanced logging macros that send to both serial and MQTT
#define LOG_DUAL(level, tag, format, ...)                        \
    do {                                                         \
        if (LOG_LOCAL_LEVEL >= level) {                          \
            esp_log_write(level, tag, format, ##__VA_ARGS__);    \
        }                                                        \
        if (mqtt_logger_is_connected()) {                        \
            mqtt_logger_logf(level, tag, format, ##__VA_ARGS__); \
        }                                                        \
    } while (0)

#define LOGE_DUAL(tag, format, ...) LOG_DUAL(ESP_LOG_ERROR, tag, format, ##__VA_ARGS__)
#define LOGW_DUAL(tag, format, ...) LOG_DUAL(ESP_LOG_WARN, tag, format, ##__VA_ARGS__)
#define LOGI_DUAL(tag, format, ...) LOG_DUAL(ESP_LOG_INFO, tag, format, ##__VA_ARGS__)
#define LOGD_DUAL(tag, format, ...) LOG_DUAL(ESP_LOG_DEBUG, tag, format, ##__VA_ARGS__)

#else

// Fallback to regular ESP_LOG when MQTT is disabled
#define LOGE_DUAL(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define LOGW_DUAL(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define LOGI_DUAL(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define LOGD_DUAL(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)

#endif  // MQTT_LOGGING_ENABLED

#endif  // UNIFIED_LOGGER_H
