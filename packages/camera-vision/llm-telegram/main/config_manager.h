#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Configuration structure
typedef struct {
    // WiFi settings
    char wifi_ssid[32];
    char wifi_password[64];

    // Telegram settings
    char telegram_bot_token[64];
    int64_t telegram_chat_id;

    // LLM settings
    int llm_backend_type;  // 0=Ollama, 1=Claude
    char claude_api_key[128];
    char ollama_server_url[128];
    char llm_model[32];

    // Camera settings
    int camera_quality;
    int capture_interval_ms;

    // System settings
    bool auto_capture_enabled;
    bool telegram_commands_enabled;
    int log_level;
} app_config_t;

// Initialize configuration manager
esp_err_t config_manager_init(void);

// Load configuration from NVS
esp_err_t config_load(app_config_t *config);

// Save configuration to NVS
esp_err_t config_save(const app_config_t *config);

// Reset configuration to defaults
esp_err_t config_reset_defaults(app_config_t *config);

// Update single configuration value
esp_err_t config_set_string(const char *key, const char *value);
esp_err_t config_set_int(const char *key, int value);

// Get single configuration value
esp_err_t config_get_string(const char *key, char *value, size_t max_len);
esp_err_t config_get_int(const char *key, int *value);

// Validate configuration
bool config_validate(const app_config_t *config);

// Print configuration (for debugging)
void config_print(const app_config_t *config);

#endif  // CONFIG_MANAGER_H
