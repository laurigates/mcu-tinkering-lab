/**
 * @file config.h
 * @brief Configuration settings for ESP32-CAM robocar
 */

#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration - Use secure credential loader
// Note: Credentials are loaded dynamically via credentials_loader module
// This supports both environment variables and credentials.h fallback

// ========================================
// System Timing Configuration
// ========================================
#define CAPTURE_INTERVAL_MS 5000            // Capture image every 5 seconds
#define WIFI_RETRY_INTERVAL_MS 10000        // Retry WiFi connection every 10 seconds
#define COMMAND_TIMEOUT_MS 1000             // Timeout for movement commands
#define MAX_RETRY_COUNT 3                   // Maximum retries for API calls
#define SYSTEM_STABILIZATION_DELAY_MS 1000  // System startup stabilization delay
#define TASK_DELAY_SHORT_MS 100             // Short task delay
#define STATUS_LED_BLINK_SLOW_ON_MS 200     // Status LED slow blink on time
#define STATUS_LED_BLINK_SLOW_OFF_MS 1800   // Status LED slow blink off time
#define STATUS_LED_BLINK_FAST_MS 100        // Status LED fast blink interval

// Camera Configuration
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA  // 320x240 for faster processing
#define CAMERA_JPEG_QUALITY 15            // Lower = higher quality

// Status LED Configuration
#define STATUS_LED_ENABLED 0  // Set to 0 to disable, 1 to enable
#define STATUS_LED_ON_TIME_MS 100
#define STATUS_LED_OFF_TIME_MS 900

// Debug Configuration
#define ENABLE_VERBOSE_LOGGING 1
#define ENABLE_SERIAL_ECHO 1

// ========================================
// Buffer and Stack Sizes
// ========================================
#define CLAUDE_RESPONSE_BUFFER_SIZE 4096  // Response buffer for Claude API
#define CLAUDE_MAX_TOKENS 512             // Maximum tokens in Claude response
#define CAPTURE_TASK_STACK_SIZE 16384     // Stack size for capture/analyze task
#define MOVEMENT_CMD_MAX_LEN 16           // Max length for movement command string
#define SOUND_CMD_MAX_LEN 64              // Max length for sound command string

// ========================================
// Semaphore Timeouts
// ========================================
#define SEMAPHORE_TIMEOUT_MS 5000  // Timeout for semaphore acquisition

// AI Backend Configuration
// #define CONFIG_AI_BACKEND_CLAUDE
#ifndef CONFIG_AI_BACKEND_OLLAMA
#define CONFIG_AI_BACKEND_OLLAMA
#endif

// Claude Configuration
// #define CLAUDE_API_URL "https://api.anthropic.com/v1/messages"
// #define CLAUDE_MODEL "claude-3-haiku-20240307"

// Ollama Configuration
#define OLLAMA_API_URL "http://192.168.0.115:11434/api/generate"
#define OLLAMA_MODEL "gemma3:4b"

// Ollama Service Discovery Configuration
#define OLLAMA_USE_SERVICE_DISCOVERY 1                    // Enable SRV record discovery
#define OLLAMA_SRV_RECORD "_ollama._tcp.local"            // SRV record for mDNS discovery
#define OLLAMA_FALLBACK_URL "http://192.168.0.115:11434"  // Fallback URL if discovery fails
#define OLLAMA_DISCOVERY_TIMEOUT_MS 5000                  // DNS query timeout
#define OLLAMA_USE_MDNS 1                                 // Enable mDNS for local discovery

// MQTT Remote Logging Configuration
#define MQTT_LOGGING_ENABLED 1                          // Enable MQTT logging
#define MQTT_BROKER_URI "mqtt://192.168.0.100:1883"     // MQTT broker URI
#define MQTT_CLIENT_ID "esp32cam_robocar"               // MQTT client ID
#define MQTT_LOG_TOPIC_BASE "robocar/esp32cam/logs"     // Base topic for logs
#define MQTT_STATUS_TOPIC "robocar/esp32cam/status"     // Status topic
#define MQTT_COMMAND_TOPIC "robocar/esp32cam/commands"  // Command topic
#define MQTT_USERNAME NULL                              // MQTT username (NULL if not required)
#define MQTT_PASSWORD NULL                              // MQTT password (NULL if not required)
#define MQTT_LOG_BUFFER_SIZE 2048                       // Log buffer size in bytes
#define MQTT_KEEPALIVE_INTERVAL 60                      // MQTT keepalive interval in seconds
#define MQTT_QOS_LEVEL 1                                // QoS level (0, 1, or 2)
#define MQTT_MIN_LOG_LEVEL ESP_LOG_INFO                 // Minimum log level for MQTT
#define MQTT_RETAIN_STATUS true                         // Retain status messages

#endif  // CONFIG_H
