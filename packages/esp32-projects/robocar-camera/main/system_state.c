/**
 * @file system_state.c
 * @brief Implementation of centralized state management for ESP32-CAM system
 */

#include "system_state.h"
#include "config.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "system_state";

// Global state manager instance
static system_state_manager_t g_state_manager = {0};
static bool g_state_initialized = false;

// Helper macro: take mutex with bounded timeout, log on failure
#define STATE_LOCK(action) \
    do { \
        if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) { \
            action; \
            xSemaphoreGive(g_state_manager.state_mutex); \
        } else { \
            ESP_LOGE(TAG, "State mutex timeout in %s", __func__); \
        } \
    } while (0)

// ========================================
// Initialization and Cleanup
// ========================================

esp_err_t system_state_init(void) {
    if (g_state_initialized) {
        ESP_LOGW(TAG, "State manager already initialized");
        return ESP_OK;
    }

    // Initialize mutex
    g_state_manager.state_mutex = xSemaphoreCreateMutex();
    if (!g_state_manager.state_mutex) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return ESP_FAIL;
    }

    // Initialize state values
    g_state_manager.system_state = SYSTEM_STATE_INITIALIZING;
    g_state_manager.wifi_state = WIFI_STATE_DISCONNECTED;
    g_state_manager.ai_state = AI_STATE_UNAVAILABLE;

    // Initialize flags
    g_state_manager.hardware_initialized = false;
    g_state_manager.mqtt_connected = false;
    g_state_manager.camera_available = false;

    // Initialize statistics
    g_state_manager.capture_count = 0;
    g_state_manager.ai_analysis_count = 0;
    g_state_manager.ai_success_count = 0;
    g_state_manager.ai_error_count = 0;
    g_state_manager.i2c_command_count = 0;

    // Initialize last operation data
    g_state_manager.last_capture_time = 0;
    g_state_manager.last_ai_analysis_time = 0;
    strncpy(g_state_manager.last_movement_command, "S",
            sizeof(g_state_manager.last_movement_command) - 1);
    g_state_manager.last_movement_command[sizeof(g_state_manager.last_movement_command) - 1] = '\0';
    g_state_manager.last_sound_command[0] = '\0';

    g_state_initialized = true;
    ESP_LOGI(TAG, "State manager initialized successfully");
    return ESP_OK;
}

void system_state_cleanup(void) {
    if (!g_state_initialized) {
        return;
    }

    if (g_state_manager.state_mutex) {
        vSemaphoreDelete(g_state_manager.state_mutex);
        g_state_manager.state_mutex = NULL;
    }

    g_state_initialized = false;
    ESP_LOGI(TAG, "State manager cleaned up");
}

// ========================================
// State Getters and Setters
// ========================================

system_state_t system_state_get_current(void) {
    if (!g_state_initialized) {
        return SYSTEM_STATE_ERROR;
    }

    system_state_t state = SYSTEM_STATE_ERROR;
    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        state = g_state_manager.system_state;
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
    return state;
}

void system_state_set(system_state_t new_state) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        if (g_state_manager.system_state != new_state) {
            ESP_LOGI(TAG, "System state changed: %s -> %s",
                     system_state_to_string(g_state_manager.system_state),
                     system_state_to_string(new_state));
            g_state_manager.system_state = new_state;
        }
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

wifi_state_t system_state_get_wifi(void) {
    if (!g_state_initialized) {
        return WIFI_STATE_ERROR;
    }

    wifi_state_t state = WIFI_STATE_ERROR;
    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        state = g_state_manager.wifi_state;
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
    return state;
}

void system_state_set_wifi(wifi_state_t new_state) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        if (g_state_manager.wifi_state != new_state) {
            ESP_LOGI(TAG, "WiFi state changed: %s -> %s",
                     wifi_state_to_string(g_state_manager.wifi_state),
                     wifi_state_to_string(new_state));
            g_state_manager.wifi_state = new_state;
        }
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

ai_state_t system_state_get_ai(void) {
    if (!g_state_initialized) {
        return AI_STATE_ERROR;
    }

    ai_state_t state = AI_STATE_ERROR;
    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        state = g_state_manager.ai_state;
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
    return state;
}

void system_state_set_ai(ai_state_t new_state) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        if (g_state_manager.ai_state != new_state) {
            ESP_LOGI(TAG, "AI state changed: %s -> %s",
                     ai_state_to_string(g_state_manager.ai_state),
                     ai_state_to_string(new_state));
            g_state_manager.ai_state = new_state;
        }
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

// ========================================
// Operational Flags and Statistics
// ========================================

void system_state_update_flags(bool hardware_ready, bool mqtt_ready, bool camera_ready) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        g_state_manager.hardware_initialized = hardware_ready;
        g_state_manager.mqtt_connected = mqtt_ready;
        g_state_manager.camera_available = camera_ready;
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_increment_capture_count(void) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        g_state_manager.capture_count++;
        g_state_manager.last_capture_time = esp_timer_get_time() / 1000; // Convert to ms
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_increment_ai_analysis(bool success) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        g_state_manager.ai_analysis_count++;
        g_state_manager.last_ai_analysis_time = esp_timer_get_time() / 1000; // Convert to ms

        if (success) {
            g_state_manager.ai_success_count++;
        } else {
            g_state_manager.ai_error_count++;
        }
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_increment_i2c_command(void) {
    if (!g_state_initialized) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        g_state_manager.i2c_command_count++;
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_update_last_movement(const char* command) {
    if (!g_state_initialized || !command) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        strncpy(g_state_manager.last_movement_command, command, sizeof(g_state_manager.last_movement_command) - 1);
        g_state_manager.last_movement_command[sizeof(g_state_manager.last_movement_command) - 1] = '\0';
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_update_last_sound(const char* command) {
    if (!g_state_initialized || !command) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        strncpy(g_state_manager.last_sound_command, command, sizeof(g_state_manager.last_sound_command) - 1);
        g_state_manager.last_sound_command[sizeof(g_state_manager.last_sound_command) - 1] = '\0';
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

void system_state_get_statistics(system_state_manager_t* stats) {
    if (!g_state_initialized || !stats) {
        return;
    }

    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        memcpy(stats, &g_state_manager, sizeof(system_state_manager_t));
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
}

// ========================================
// String Conversion Functions
// ========================================

const char* system_state_to_string(system_state_t state) {
    switch (state) {
        case SYSTEM_STATE_INITIALIZING: return "INITIALIZING";
        case SYSTEM_STATE_HARDWARE_READY: return "HARDWARE_READY";
        case SYSTEM_STATE_NETWORK_CONNECTING: return "NETWORK_CONNECTING";
        case SYSTEM_STATE_NETWORK_CONNECTED: return "NETWORK_CONNECTED";
        case SYSTEM_STATE_AI_INITIALIZING: return "AI_INITIALIZING";
        case SYSTEM_STATE_FULLY_OPERATIONAL: return "FULLY_OPERATIONAL";
        case SYSTEM_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const char* wifi_state_to_string(wifi_state_t state) {
    switch (state) {
        case WIFI_STATE_DISCONNECTED: return "DISCONNECTED";
        case WIFI_STATE_CONNECTING: return "CONNECTING";
        case WIFI_STATE_CONNECTED: return "CONNECTED";
        case WIFI_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

const char* ai_state_to_string(ai_state_t state) {
    switch (state) {
        case AI_STATE_UNAVAILABLE: return "UNAVAILABLE";
        case AI_STATE_INITIALIZING: return "INITIALIZING";
        case AI_STATE_READY: return "READY";
        case AI_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// ========================================
// Utility Functions
// ========================================

bool system_state_is_operational(void) {
    if (!g_state_initialized) {
        return false;
    }

    bool operational = false;
    if (xSemaphoreTake(g_state_manager.state_mutex, pdMS_TO_TICKS(SEMAPHORE_TIMEOUT_MS)) == pdTRUE) {
        operational = (g_state_manager.system_state == SYSTEM_STATE_FULLY_OPERATIONAL) &&
                     (g_state_manager.hardware_initialized) &&
                     (g_state_manager.camera_available);
        xSemaphoreGive(g_state_manager.state_mutex);
    } else {
        ESP_LOGE(TAG, "State mutex timeout in %s", __func__);
    }
    return operational;
}
