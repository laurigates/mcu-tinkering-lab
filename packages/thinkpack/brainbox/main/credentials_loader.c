/**
 * @file credentials_loader.c
 * @brief Secure credential loading implementation
 */

#include "credentials_loader.h"
#include <stdlib.h>
#include <string.h>
#include "credentials.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "credentials_loader";

// NVS namespace for Improv WiFi provisioned credentials
#define NVS_NAMESPACE "wifi_config"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASSWORD "password"

// Global credentials instance
static credentials_t g_credentials = {0};
static bool g_credentials_initialized = false;

/**
 * @brief Attempt to load WiFi credentials from NVS (highest priority source).
 *
 * Credentials stored here were provisioned via Improv WiFi.
 */
static bool load_from_nvs(credentials_t *creds)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    char ssid[MAX_SSID_LENGTH] = {0};
    char password[MAX_PASSWORD_LENGTH] = {0};
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);

    bool ok =
        (nvs_get_str(handle, NVS_KEY_SSID, ssid, &ssid_len) == ESP_OK &&
         nvs_get_str(handle, NVS_KEY_PASSWORD, password, &pass_len) == ESP_OK && strlen(ssid) > 0);
    nvs_close(handle);

    if (ok) {
        strncpy(creds->wifi_ssid, ssid, MAX_SSID_LENGTH - 1);
        creds->wifi_ssid[MAX_SSID_LENGTH - 1] = '\0';
        strncpy(creds->wifi_password, password, MAX_PASSWORD_LENGTH - 1);
        creds->wifi_password[MAX_PASSWORD_LENGTH - 1] = '\0';
        ESP_LOGI(TAG, "Loaded WiFi credentials from NVS (SSID: %s)", creds->wifi_ssid);
    }
    return ok;
}

/**
 * @brief Attempt to load credential from environment variable
 *
 * @param env_var Environment variable name
 * @param dest Destination buffer
 * @param max_len Maximum length of destination buffer
 * @return true if loaded successfully, false otherwise
 */
static bool load_from_env(const char *env_var, char *dest, size_t max_len)
{
    const char *value = getenv(env_var);
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
static bool load_from_file(credentials_t *creds)
{
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

#ifdef GEMINI_API_KEY
    strncpy(creds->gemini_api_key, GEMINI_API_KEY, MAX_API_KEY_LENGTH - 1);
    creds->gemini_api_key[MAX_API_KEY_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded Gemini API key from credentials.h");
#else
    ESP_LOGI(TAG, "Gemini API key not configured - only required for Gemini backend");
#endif

    return true;
}

bool load_credentials(credentials_t *creds)
{
    if (!creds) {
        ESP_LOGE(TAG, "Invalid credentials pointer");
        return false;
    }

    // Clear credentials structure
    memset(creds, 0, sizeof(credentials_t));

    ESP_LOGI(TAG, "Loading credentials...");

    // Priority 1: NVS (credentials stored by Improv WiFi provisioner)
    if (load_from_nvs(creds)) {
        // WiFi credentials loaded from NVS; still try env/file for API keys
        load_from_env("CLAUDE_API_KEY", creds->claude_api_key, MAX_API_KEY_LENGTH);
        load_from_env("GEMINI_API_KEY", creds->gemini_api_key, MAX_API_KEY_LENGTH);
        creds->credentials_loaded = true;
        return true;
    }

    // Priority 2: Environment variables (CI/CD builds)
    bool env_loaded = true;
    if (!load_from_env("WIFI_SSID", creds->wifi_ssid, MAX_SSID_LENGTH)) {
        env_loaded = false;
    }
    if (!load_from_env("WIFI_PASSWORD", creds->wifi_password, MAX_PASSWORD_LENGTH)) {
        env_loaded = false;
    }
    // API keys are optional — only needed for their respective backends
    load_from_env("CLAUDE_API_KEY", creds->claude_api_key, MAX_API_KEY_LENGTH);
    load_from_env("GEMINI_API_KEY", creds->gemini_api_key, MAX_API_KEY_LENGTH);

    if (env_loaded) {
        ESP_LOGI(TAG, "Successfully loaded credentials from environment variables");
        creds->credentials_loaded = true;
        return true;
    }

    ESP_LOGI(TAG, "Environment variables not available, trying credentials.h file...");

    // Priority 3: credentials.h file (developer builds)
    // cppcheck-suppress knownConditionTrueFalse // guarded for future failure modes
    if (load_from_file(creds)) {
        ESP_LOGI(TAG, "Successfully loaded credentials from credentials.h");
        creds->credentials_loaded = true;
        return true;
    }

    ESP_LOGW(TAG, "No WiFi credentials found - Improv WiFi provisioning required");
    return false;
}

bool validate_credentials(const credentials_t *creds)
{
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

// Gemini API key validation (optional - only required for Gemini backend)
#ifdef CONFIG_AI_BACKEND_GEMINI
    if (strlen(creds->gemini_api_key) == 0) {
        ESP_LOGE(TAG, "Gemini API key is required for Gemini backend but not provided");
        return false;
    }
#endif

    ESP_LOGI(TAG, "Credentials validation successful");
    return true;
}

bool are_credentials_available(void)
{
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

const char *get_wifi_ssid(void)
{
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.wifi_ssid;
}

const char *get_wifi_password(void)
{
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.wifi_password;
}

const char *get_claude_api_key(void)
{
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.claude_api_key;
}

const char *get_gemini_api_key(void)
{
    if (!are_credentials_available()) {
        ESP_LOGE(TAG, "Credentials not available");
        return NULL;
    }
    return g_credentials.gemini_api_key;
}

bool credentials_nvs_save_wifi(const char *ssid, const char *password)
{
    if (!ssid || !password) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return false;
    }

    bool ok =
        (nvs_set_str(handle, NVS_KEY_SSID, ssid) == ESP_OK &&
         nvs_set_str(handle, NVS_KEY_PASSWORD, password) == ESP_OK && nvs_commit(handle) == ESP_OK);

    nvs_close(handle);
    if (ok) {
        ESP_LOGI(TAG, "WiFi credentials saved to NVS (SSID: %s)", ssid);
    } else {
        ESP_LOGE(TAG, "Failed to save WiFi credentials to NVS");
    }
    return ok;
}

bool credentials_reload(void)
{
    g_credentials_initialized = false;
    memset(&g_credentials, 0, sizeof(g_credentials));
    return are_credentials_available();
}
