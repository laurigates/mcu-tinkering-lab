/**
 * @file config.h
 * @brief Unified configuration for robocar-unified (XIAO ESP32-S3 Sense)
 *
 * Merged from robocar-camera/config.h and robocar-main/system_config.h
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// System Timing Configuration
// ========================================
#define CAPTURE_INTERVAL_MS 5000
#define WIFI_RETRY_INTERVAL_MS 10000
#define COMMAND_TIMEOUT_MS 1000
#define MAX_RETRY_COUNT 3
#define SYSTEM_STABILIZATION_DELAY_MS 1000
#define TASK_DELAY_SHORT_MS 100
#define STATUS_LED_BLINK_SLOW_ON_MS 200
#define STATUS_LED_BLINK_SLOW_OFF_MS 1800
#define STATUS_LED_BLINK_FAST_MS 100

// Camera Configuration
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA
#define CAMERA_JPEG_QUALITY 15

// Debug Configuration
#define ENABLE_VERBOSE_LOGGING 1
#define ENABLE_SERIAL_ECHO 1

// ========================================
// Buffer and Stack Sizes
// ========================================
#define CAPTURE_TASK_STACK_SIZE 16384
#define MOVEMENT_CMD_MAX_LEN 16
#define SOUND_CMD_MAX_LEN 64

// ========================================
// Semaphore Timeouts
// ========================================
#define SEMAPHORE_TIMEOUT_MS 5000

// ========================================
// AI Backend Configuration
// ========================================
// Gemini Robotics-ER 1.6 is the sole planner backend (see planner_task.c).
// gemini_backend.c owns its own URL and model constants; nothing to configure here.

// ========================================
// MQTT Remote Logging Configuration
// ========================================
#define MQTT_LOGGING_ENABLED 1
#define MQTT_BROKER_URI "mqtt://192.168.0.100:1883"
#define MQTT_CLIENT_ID "robocar_unified"
#define MQTT_LOG_TOPIC_BASE "robocar/logs"
#define MQTT_STATUS_TOPIC "robocar/status"
#define MQTT_COMMAND_TOPIC "robocar/commands"
#define MQTT_USERNAME NULL
#define MQTT_PASSWORD NULL
#define MQTT_LOG_BUFFER_SIZE 2048
#define MQTT_KEEPALIVE_INTERVAL 60
#define MQTT_QOS_LEVEL 1
#define MQTT_MIN_LOG_LEVEL ESP_LOG_INFO
#define MQTT_RETAIN_STATUS true

// ========================================
// OTA Configuration
// ========================================
#define OTA_ENABLED 1
#define OTA_GITHUB_HOST "github.com"
#define OTA_GITHUB_ORG CONFIG_OTA_GITHUB_ORG
#define OTA_GITHUB_REPO CONFIG_OTA_GITHUB_REPO
#define OTA_FIRMWARE_FILENAME_MATCH "robocar-unified"
#define OTA_CHECK_INTERVAL_MIN 360
#define OTA_MQTT_NOTIFY_TOPIC "robocar/ota/notify"
#define OTA_MQTT_STATUS_TOPIC "robocar/ota/status"
#define OTA_STABILITY_TIMEOUT_MS 60000
#define OTA_TASK_STACK_SIZE 8192
#define OTA_TASK_PRIORITY 5
#define OTA_HTTP_TIMEOUT_MS 30000
#define OTA_MAX_RETRY_COUNT 3

// ========================================
// Feature Flags
// ========================================
#define FEATURE_PIEZO_ENABLED 1
#define FEATURE_PCA9685_ENABLED 1
#define FEATURE_OLED_ENABLED 1
#define FEATURE_WIFI_ENABLED 1
#define FEATURE_MQTT_ENABLED 1
#define FEATURE_OTA_ENABLED 1
#define FEATURE_AI_ENABLED 1

// Status LED is not available on XIAO Sense (no user-accessible LED)
#define STATUS_LED_ENABLED 0

#endif  // CONFIG_H
