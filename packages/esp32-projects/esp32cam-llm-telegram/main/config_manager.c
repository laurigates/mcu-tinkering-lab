#include "config_manager.h"
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "CONFIG_MANAGER";
static const char *NVS_NAMESPACE = "esp32cam_llm";

static nvs_handle_t nvs_handle = 0;
static bool is_initialized = false;

// Initialize configuration manager
esp_err_t config_manager_init(void)
{
    if (is_initialized) {
        return ESP_OK;
    }

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    is_initialized = true;
    ESP_LOGI(TAG, "Configuration manager initialized");
    return ESP_OK;
}

// Load configuration from NVS
esp_err_t config_load(app_config_t *config)
{
    if (!is_initialized || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Loading configuration from NVS");

    size_t length = 0;
    esp_err_t err;

    // Load WiFi settings
    length = sizeof(config->wifi_ssid);
    err = nvs_get_str(nvs_handle, "wifi_ssid", config->wifi_ssid, &length);
    if (err != ESP_OK) {
        strcpy(config->wifi_ssid, WIFI_SSID_DEFAULT);
    }

    length = sizeof(config->wifi_password);
    err = nvs_get_str(nvs_handle, "wifi_pass", config->wifi_password, &length);
    if (err != ESP_OK) {
        strcpy(config->wifi_password, WIFI_PASSWORD_DEFAULT);
    }

    // Load Telegram settings
    length = sizeof(config->telegram_bot_token);
    err = nvs_get_str(nvs_handle, "tg_token", config->telegram_bot_token, &length);
    if (err != ESP_OK) {
        strcpy(config->telegram_bot_token, TELEGRAM_BOT_TOKEN);
    }

    err = nvs_get_i64(nvs_handle, "tg_chat_id", &config->telegram_chat_id);
    if (err != ESP_OK) {
        config->telegram_chat_id = 0;  // Will need to be set
    }

    // Load LLM settings
    err = nvs_get_i32(nvs_handle, "llm_backend", &config->llm_backend_type);
    if (err != ESP_OK) {
        config->llm_backend_type = LLM_BACKEND_TYPE;
    }

    length = sizeof(config->claude_api_key);
    err = nvs_get_str(nvs_handle, "claude_key", config->claude_api_key, &length);
    if (err != ESP_OK) {
        strcpy(config->claude_api_key, CLAUDE_API_KEY);
    }

    length = sizeof(config->ollama_server_url);
    err = nvs_get_str(nvs_handle, "ollama_url", config->ollama_server_url, &length);
    if (err != ESP_OK) {
        strcpy(config->ollama_server_url, OLLAMA_SERVER_URL);
    }

    length = sizeof(config->llm_model);
    err = nvs_get_str(nvs_handle, "llm_model", config->llm_model, &length);
    if (err != ESP_OK) {
        strcpy(config->llm_model, config->llm_backend_type ? CLAUDE_MODEL : OLLAMA_MODEL);
    }

    // Load camera settings
    err = nvs_get_i32(nvs_handle, "cam_quality", &config->camera_quality);
    if (err != ESP_OK) {
        config->camera_quality = CAMERA_JPEG_QUALITY;
    }

    err = nvs_get_i32(nvs_handle, "cam_interval", &config->capture_interval_ms);
    if (err != ESP_OK) {
        config->capture_interval_ms = CAMERA_CAPTURE_INTERVAL_MS;
    }

    // Load system settings
    uint8_t auto_capture;
    err = nvs_get_u8(nvs_handle, "auto_capture", &auto_capture);
    config->auto_capture_enabled = (err == ESP_OK) ? auto_capture : true;

    uint8_t tg_commands;
    err = nvs_get_u8(nvs_handle, "tg_commands", &tg_commands);
    config->telegram_commands_enabled = (err == ESP_OK) ? tg_commands : true;

    err = nvs_get_i32(nvs_handle, "log_level", &config->log_level);
    if (err != ESP_OK) {
        config->log_level = LOG_LEVEL_DEBUG;
    }

    ESP_LOGI(TAG, "Configuration loaded successfully");
    return ESP_OK;
}

// Save configuration to NVS
esp_err_t config_save(const app_config_t *config)
{
    if (!is_initialized || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Saving configuration to NVS");

    esp_err_t err;

    // Save WiFi settings
    err = nvs_set_str(nvs_handle, "wifi_ssid", config->wifi_ssid);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_str(nvs_handle, "wifi_pass", config->wifi_password);
    if (err != ESP_OK)
        goto save_error;

    // Save Telegram settings
    err = nvs_set_str(nvs_handle, "tg_token", config->telegram_bot_token);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_i64(nvs_handle, "tg_chat_id", config->telegram_chat_id);
    if (err != ESP_OK)
        goto save_error;

    // Save LLM settings
    err = nvs_set_i32(nvs_handle, "llm_backend", config->llm_backend_type);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_str(nvs_handle, "claude_key", config->claude_api_key);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_str(nvs_handle, "ollama_url", config->ollama_server_url);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_str(nvs_handle, "llm_model", config->llm_model);
    if (err != ESP_OK)
        goto save_error;

    // Save camera settings
    err = nvs_set_i32(nvs_handle, "cam_quality", config->camera_quality);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_i32(nvs_handle, "cam_interval", config->capture_interval_ms);
    if (err != ESP_OK)
        goto save_error;

    // Save system settings
    err = nvs_set_u8(nvs_handle, "auto_capture", config->auto_capture_enabled ? 1 : 0);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_u8(nvs_handle, "tg_commands", config->telegram_commands_enabled ? 1 : 0);
    if (err != ESP_OK)
        goto save_error;

    err = nvs_set_i32(nvs_handle, "log_level", config->log_level);
    if (err != ESP_OK)
        goto save_error;

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
        goto save_error;

    ESP_LOGI(TAG, "Configuration saved successfully");
    return ESP_OK;

save_error:
    ESP_LOGE(TAG, "Error saving configuration: %s", esp_err_to_name(err));
    return err;
}

// Reset configuration to defaults
esp_err_t config_reset_defaults(app_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Resetting configuration to defaults");

    strcpy(config->wifi_ssid, WIFI_SSID_DEFAULT);
    strcpy(config->wifi_password, WIFI_PASSWORD_DEFAULT);
    strcpy(config->telegram_bot_token, TELEGRAM_BOT_TOKEN);
    config->telegram_chat_id = 0;
    config->llm_backend_type = LLM_BACKEND_TYPE;
    strcpy(config->claude_api_key, CLAUDE_API_KEY);
    strcpy(config->ollama_server_url, OLLAMA_SERVER_URL);
    strcpy(config->llm_model, config->llm_backend_type ? CLAUDE_MODEL : OLLAMA_MODEL);
    config->camera_quality = CAMERA_JPEG_QUALITY;
    config->capture_interval_ms = CAMERA_CAPTURE_INTERVAL_MS;
    config->auto_capture_enabled = true;
    config->telegram_commands_enabled = true;
    config->log_level = LOG_LEVEL_DEBUG;

    return ESP_OK;
}

// Update single string configuration value
esp_err_t config_set_string(const char *key, const char *value)
{
    if (!is_initialized || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_set_str(nvs_handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    return err;
}

// Update single int configuration value
esp_err_t config_set_int(const char *key, int value)
{
    if (!is_initialized || !key) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_set_i32(nvs_handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    return err;
}

// Get single string configuration value
esp_err_t config_get_string(const char *key, char *value, size_t max_len)
{
    if (!is_initialized || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_get_str(nvs_handle, key, value, &max_len);
}

// Get single int configuration value
esp_err_t config_get_int(const char *key, int *value)
{
    if (!is_initialized || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_get_i32(nvs_handle, key, value);
}

// Validate configuration
bool config_validate(const app_config_t *config)
{
    if (!config) {
        return false;
    }

    // Check required fields
    if (strlen(config->wifi_ssid) == 0) {
        ESP_LOGE(TAG, "WiFi SSID not set");
        return false;
    }

    if (strlen(config->wifi_password) == 0) {
        ESP_LOGE(TAG, "WiFi password not set");
        return false;
    }

    if (strlen(config->telegram_bot_token) == 0 ||
        strcmp(config->telegram_bot_token, "YOUR_BOT_TOKEN") == 0) {
        ESP_LOGW(TAG, "Telegram bot token not configured");
    }

    if (config->telegram_chat_id == 0) {
        ESP_LOGW(TAG, "Telegram chat ID not configured");
    }

    if (config->llm_backend_type == 1) {  // Claude
        if (strlen(config->claude_api_key) == 0 ||
            strcmp(config->claude_api_key, "YOUR_CLAUDE_API_KEY") == 0) {
            ESP_LOGE(TAG, "Claude API key not configured");
            return false;
        }
    } else {  // Ollama
        if (strlen(config->ollama_server_url) == 0) {
            ESP_LOGE(TAG, "Ollama server URL not configured");
            return false;
        }
    }

    return true;
}

// Print configuration (for debugging)
void config_print(const app_config_t *config)
{
    if (!config) {
        return;
    }

    ESP_LOGI(TAG, "=== Configuration ===");
    ESP_LOGI(TAG, "WiFi SSID: %s", config->wifi_ssid);
    ESP_LOGI(TAG, "Telegram Bot Token: %s...%s",
             config->telegram_bot_token[0] != 0 ? "***" : "NOT SET",
             config->telegram_bot_token[0] != 0
                 ? config->telegram_bot_token + strlen(config->telegram_bot_token) - 4
                 : "");
    ESP_LOGI(TAG, "Telegram Chat ID: %lld", config->telegram_chat_id);
    ESP_LOGI(TAG, "LLM Backend: %s", config->llm_backend_type ? "Claude" : "Ollama");
    if (config->llm_backend_type) {
        ESP_LOGI(TAG, "Claude Model: %s", config->llm_model);
    } else {
        ESP_LOGI(TAG, "Ollama Server: %s", config->ollama_server_url);
        ESP_LOGI(TAG, "Ollama Model: %s", config->llm_model);
    }
    ESP_LOGI(TAG, "Camera Quality: %d", config->camera_quality);
    ESP_LOGI(TAG, "Capture Interval: %d ms", config->capture_interval_ms);
    ESP_LOGI(TAG, "Auto Capture: %s", config->auto_capture_enabled ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "Telegram Commands: %s",
             config->telegram_commands_enabled ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "===================");
}
