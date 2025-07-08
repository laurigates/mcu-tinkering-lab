/**
 * @file config.h
 * @brief Configuration settings for ESP32-CAM robocar
 */

#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration - Use secure credential loader
// Note: Credentials are loaded dynamically via credentials_loader module
// This supports both environment variables and credentials.h fallback

// Timing Configuration
#define CAPTURE_INTERVAL_MS     5000    // Capture image every 5 seconds
#define WIFI_RETRY_INTERVAL_MS  10000   // Retry WiFi connection every 10 seconds
#define COMMAND_TIMEOUT_MS      1000    // Timeout for movement commands
#define MAX_RETRY_COUNT         3       // Maximum retries for API calls

// Camera Configuration
#define CAMERA_FRAME_SIZE       FRAMESIZE_QVGA  // 320x240 for faster processing
#define CAMERA_JPEG_QUALITY     15              // Lower = higher quality

// Status LED Configuration
#define STATUS_LED_ENABLED      0       // Set to 0 to disable, 1 to enable
#define STATUS_LED_ON_TIME_MS   100
#define STATUS_LED_OFF_TIME_MS  900

// Debug Configuration
#define ENABLE_VERBOSE_LOGGING  1
#define ENABLE_SERIAL_ECHO      1

// AI Backend Configuration
// #define CONFIG_AI_BACKEND_CLAUDE
#define CONFIG_AI_BACKEND_OLLAMA

// Claude Configuration
// #define CLAUDE_API_URL "https://api.anthropic.com/v1/messages"
// #define CLAUDE_MODEL "claude-3-haiku-20240307"

// Ollama Configuration
#define OLLAMA_API_URL "http://192.168.0.115:11434/api/generate"
#define OLLAMA_MODEL "gemma3:4b"

// Ollama Service Discovery Configuration
#define OLLAMA_USE_SERVICE_DISCOVERY    1                           // Enable SRV record discovery
#define OLLAMA_SRV_RECORD              "_ollama._tcp.local"         // SRV record for mDNS discovery
#define OLLAMA_FALLBACK_URL            "http://192.168.0.115:11434" // Fallback URL if discovery fails
#define OLLAMA_DISCOVERY_TIMEOUT_MS    5000                         // DNS query timeout
#define OLLAMA_USE_MDNS                1                            // Enable mDNS for local discovery

#endif // CONFIG_H
