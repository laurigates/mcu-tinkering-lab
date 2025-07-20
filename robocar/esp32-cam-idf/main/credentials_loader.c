/**
 * @file credentials_loader.c
 * @brief Secure credential loading implementation
 */

#include "credentials_loader.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "credentials.h"

static const char *TAG = "credentials_loader";

// Global credentials instance
static credentials_t g_credentials = {0};
static bool g_credentials_initialized = false;

/**
 * @brief Attempt to load credential from environment variable
 * 
 * @param env_var Environment variable name
 * @param dest Destination buffer
 * @param max_len Maximum length of destination buffer
 * @return true if loaded successfully, false otherwise
 */
static bool load_from_env(const char* env_var, char* dest, size_t max_len) {
    const char* value = getenv(env_var);
    if (value && strlen(value) > 0 && strlen(value) < max_len) {
        strncpy(dest, value, max_len - 1);
        dest[max_len - 1] = '\0';
        ESP_LOGI(TAG, "Loaded %s from environment variable", env_var);
        return true;
    }
    return false;
}

/**
 * @brief Load credentials from credentials.h file
 * 
 * @param creds Pointer to credentials structure
 * @return true if loaded successfully, false otherwise
 */
static bool load_from_file(credentials_t* creds) {
    // Note: Credentials validation is now done at startup via credentials_validator.h
    // This function assumes credentials are already validated and loads them
    
    strncpy(creds->wifi_ssid, WIFI_SSID, MAX_SSID_LENGTH - 1);
    creds->wifi_ssid[MAX_SSID_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded WiFi SSID from credentials.h");

    strncpy(creds->wifi_password, WIFI_PASSWORD, MAX_PASSWORD_LENGTH - 1);
    creds->wifi_password[MAX_PASSWORD_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded WiFi password from credentials.h");

    #ifdef CLAUDE_API_KEY
    strncpy(creds->claude_api_key, CLAUDE_API_KEY, MAX_API_KEY_LENGTH - 1);
    creds->claude_api_key[MAX_API_KEY_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded Claude API key from credentials.h");
    #else
    ESP_LOGI(TAG, "Claude API key not configured - only required for Claude backend");
    #endif

    return true;
}

bool load_credentials(credentials_t* creds) {
    if (!creds) {
        ESP_LOGE(TAG, "Invalid credentials pointer");
        return false;
    }

    // Clear credentials structure
    memset(creds, 0, sizeof(credentials_t));

    ESP_LOGI(TAG, "Loading credentials...");

    // Try environment variables first
    bool env_loaded = true;
    if (!load_from_env("WIFI_SSID", creds->wifi_ssid, MAX_SSID_LENGTH)) {
        env_loaded = false;
    }
    if (!load_from_env("WIFI_PASSWORD", creds->wifi_password, MAX_PASSWORD_LENGTH)) {
        env_loaded = false;
    }
    // Claude API key is optional (for Ollama backend)
    load_from_env("CLAUDE_API_KEY", creds->claude_api_key, MAX_API_KEY_LENGTH);

    if (env_loaded) {
        ESP_LOGI(TAG, "Successfully loaded credentials from environment variables");
        creds->credentials_loaded = true;
        return true;
    }

    ESP_LOGI(TAG, "Environment variables not available, trying credentials.h file...");

    // Fallback to credentials.h file
    if (load_from_file(creds)) {
        ESP_LOGI(TAG, "Successfully loaded credentials from credentials.h");
        creds->credentials_loaded = true;
        return true;
    }

    ESP_LOGE(TAG, "Failed to load credentials from any source");
    return false;
}

bool validate_credentials(const credentials_t* creds) {
    if (!creds || !creds->credentials_loaded) {
        ESP_LOGE(TAG, "No credentials loaded");
        return false;
    }

    // Validate WiFi credentials (required)
    if (strlen(creds->wifi_ssid) == 0) {
        ESP_LOGE(TAG, "WiFi SSID is empty");
        return false;
    }

    if (strlen(creds->wifi_password) == 0) {
        ESP_LOGE(TAG, "WiFi password is empty");
        return false;
    }

    // Claude API key validation (optional - only required for Claude backend)
    #ifdef CONFIG_AI_BACKEND_CLAUDE
    if (strlen(creds->claude_api_key) == 0) {
        ESP_LOGE(TAG, "Claude API key is required for Claude backend but not provided");
        return false;
    }

    // Basic API key format validation
    if (strncmp(creds->claude_api_key, "sk-ant-api03-", 13) != 0) {
        ESP_LOGE(TAG, "Claude API key format appears invalid");
        return false;
    }
    #endif

    ESP_LOGI(TAG, "Credentials validation successful");
    return true;
}

bool are_credentials_available(void) {
    if (!g_credentials_initialized) {
        g_credentials_initialized = true;
        if (!load_credentials(&g_credentials)) {
            return false;
        }
        if (!validate_credentials(&g_credentials)) {
            return false;
        }
    }
    return g_credentials.credentials_loaded;
}

const char* get_wifi_ssid(void) {
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.wifi_ssid;
}

const char* get_wifi_password(void) {
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.wifi_password;
}

const char* get_claude_api_key(void) {
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.claude_api_key;
}