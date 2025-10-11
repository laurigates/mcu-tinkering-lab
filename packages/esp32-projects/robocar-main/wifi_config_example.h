/**
 * @file wifi_config_example.h
 * @brief Example WiFi configuration file
 *
 * Copy this file to wifi_config.h and update with your credentials.
 * The wifi_config.h file is gitignored to prevent committing credentials.
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi Configuration
// Replace with your WiFi network credentials
#define WIFI_SSID     "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// Optional: Override default settings
// #define WIFI_MAXIMUM_RETRY         10
// #define WIFI_RETRY_BASE_DELAY_MS   1000
// #define WIFI_MAX_RETRY_DELAY_MS    30000
// #define WIFI_HOSTNAME              "RoboCar-Controller"

// Power Management Mode for Robotics
// 0 = Max Performance (no power saving, lowest latency) - Recommended for robotics
// 1 = Balanced (balanced power/performance) - Good compromise
// 2 = Min Modem (minimum modem power saving)
// 3 = Max Modem (maximum modem power saving)
#define WIFI_DEFAULT_POWER_MODE    0

// Auto-reconnect configuration
#define WIFI_AUTO_RECONNECT_ENABLED  true

// Connection monitoring interval (ms)
#define WIFI_MONITOR_INTERVAL_MS     10000

#endif // WIFI_CONFIG_H