/**
 * @file system_state.h
 * @brief Centralized state management for ESP32-CAM system
 */

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ========================================
// System State Definitions
// ========================================

typedef enum {
    SYSTEM_STATE_INITIALIZING = 0,
    SYSTEM_STATE_HARDWARE_READY,
    SYSTEM_STATE_NETWORK_CONNECTING,
    SYSTEM_STATE_NETWORK_CONNECTED,
    SYSTEM_STATE_AI_INITIALIZING,
    SYSTEM_STATE_FULLY_OPERATIONAL,
    SYSTEM_STATE_ERROR
} system_state_t;

typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR
} wifi_state_t;

typedef enum {
    AI_STATE_UNAVAILABLE = 0,
    AI_STATE_INITIALIZING,
    AI_STATE_READY,
    AI_STATE_ERROR
} ai_state_t;

// ========================================
// State Management Structure
// ========================================

typedef struct {
    // System state
    system_state_t system_state;
    wifi_state_t wifi_state;
    ai_state_t ai_state;

    // Operational flags
    bool hardware_initialized;
    bool mqtt_connected;
    bool camera_available;

    // Statistics
    uint32_t capture_count;
    uint32_t ai_analysis_count;
    uint32_t ai_success_count;
    uint32_t ai_error_count;
    uint32_t i2c_command_count;

    // Last operation results
    uint32_t last_capture_time;
    uint32_t last_ai_analysis_time;
    char last_movement_command[16];
    char last_sound_command[64];

    // Thread safety
    SemaphoreHandle_t state_mutex;
} system_state_manager_t;

// ========================================
// State Management Functions
// ========================================

/**
 * @brief Initialize the state management system
 * @return ESP_OK on success
 */
esp_err_t system_state_init(void);

/**
 * @brief Get current system state (thread-safe)
 * @return Current system state
 */
system_state_t system_state_get_current(void);

/**
 * @brief Set system state (thread-safe)
 * @param new_state New system state
 */
void system_state_set(system_state_t new_state);

/**
 * @brief Get WiFi state (thread-safe)
 * @return Current WiFi state
 */
wifi_state_t system_state_get_wifi(void);

/**
 * @brief Set WiFi state (thread-safe)
 * @param new_state New WiFi state
 */
void system_state_set_wifi(wifi_state_t new_state);

/**
 * @brief Get AI state (thread-safe)
 * @return Current AI state
 */
ai_state_t system_state_get_ai(void);

/**
 * @brief Set AI state (thread-safe)
 * @param new_state New AI state
 */
void system_state_set_ai(ai_state_t new_state);

/**
 * @brief Update operational flags (thread-safe)
 * @param hardware_ready Hardware initialization status
 * @param mqtt_ready MQTT connection status
 * @param camera_ready Camera availability status
 */
void system_state_update_flags(bool hardware_ready, bool mqtt_ready, bool camera_ready);

/**
 * @brief Increment capture statistics (thread-safe)
 */
void system_state_increment_capture_count(void);

/**
 * @brief Increment AI analysis statistics (thread-safe)
 * @param success Whether the analysis was successful
 */
void system_state_increment_ai_analysis(bool success);

/**
 * @brief Increment I2C command count (thread-safe)
 */
void system_state_increment_i2c_command(void);

/**
 * @brief Update last movement command (thread-safe)
 * @param command Movement command string
 */
void system_state_update_last_movement(const char *command);

/**
 * @brief Update last sound command (thread-safe)
 * @param command Sound command string
 */
void system_state_update_last_sound(const char *command);

/**
 * @brief Get system statistics (thread-safe)
 * @param stats Pointer to structure to fill with statistics
 */
void system_state_get_statistics(system_state_manager_t *stats);

/**
 * @brief Get human-readable system state string
 * @param state System state enum value
 * @return String representation of state
 */
const char *system_state_to_string(system_state_t state);

/**
 * @brief Get human-readable WiFi state string
 * @param state WiFi state enum value
 * @return String representation of state
 */
const char *wifi_state_to_string(wifi_state_t state);

/**
 * @brief Get human-readable AI state string
 * @param state AI state enum value
 * @return String representation of state
 */
const char *ai_state_to_string(ai_state_t state);

/**
 * @brief Check if system is fully operational
 * @return true if all subsystems are ready
 */
bool system_state_is_operational(void);

/**
 * @brief Clean up state management resources
 */
void system_state_cleanup(void);

#endif  // SYSTEM_STATE_H
