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

// NVS namespace for Improv WiFi provisioned credentials (same as robocar-main)
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
    // Note: Credentials validation is now done at startup via credentials_validator.h
    // This function assumes credentials are already validated and loads them

    strncpy(creds->wifi_ssid, WIFI_SSID, MAX_SSID_LENGTH - 1);
    creds->wifi_ssid[MAX_SSID_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded WiFi SSID from credentials.h");

    strncpy(creds->wifi_password, WIFI_PASSWORD, MAX_PASSWORD_LENGTH - 1);
    creds->wifi_password[MAX_PASSWORD_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded WiFi password from credentials.h");

#ifdef GEMINI_API_KEY
    strncpy(creds->gemini_api_key, GEMINI_API_KEY, MAX_API_KEY_LENGTH - 1);
    creds->gemini_api_key[MAX_API_KEY_LENGTH - 1] = '\0';
    ESP_LOGI(TAG, "Loaded Gemini API key from credentials.h");
#else
    ESP_LOGI(TAG, "Gemini API key not configured - planner will fail safe to STOP");
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
        // WiFi credentials loaded from NVS; still try env/file for API key
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
    // Gemini API key is required for the planner
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

    ESP_LOGW(TAG, "No WiFi credentials found — Improv WiFi provisioning required");
    return false;
}

/* Validation is deliberately per-credential rather than all-or-nothing. The
 * credentials here come from three independent sources for two independent
 * subsystems, so one missing value must not invalidate the others: a board
 * provisioned over Improv has WiFi but no Gemini key, and an open network has
 * an SSID but no password. Only a usable SSID is genuinely required. */
bool validate_credentials(const credentials_t *creds)
{
    if (!creds || !creds->credentials_loaded) {
        ESP_LOGE(TAG, "No credentials loaded");
        return false;
    }

    // The SSID is the one hard requirement — without it there is nothing to join.
    if (strlen(creds->wifi_ssid) == 0) {
        ESP_LOGE(TAG, "WiFi SSID is empty");
        return false;
    }

    // An empty password is legitimate: open networks have none.
    if (strlen(creds->wifi_password) == 0) {
        ESP_LOGW(TAG, "WiFi password is empty — assuming an open network");
    }

    // The planner needs the Gemini key, but the robot boots and drives without
    // it (the executor holds STOP until a goal arrives), so this is not fatal.
    if (strlen(creds->gemini_api_key) == 0) {
        ESP_LOGW(TAG, "No Gemini API key — AI planner will stay disabled");
    }

    ESP_LOGI(TAG, "Credentials validation successful");
    return true;
}

/**
 * @brief Load credentials into the module-global cache exactly once.
 *
 * The "once" latch is set only after a *successful* load. An early failed
 * attempt (no credentials yet, Improv provisioning still pending) must not
 * poison every later accessor — credentials_reload() depends on being able to
 * re-run this after provisioning writes NVS.
 */
static bool ensure_loaded(void)
{
    if (g_credentials_initialized) {
        return true;
    }
    if (!load_credentials(&g_credentials)) {
        return false;
    }
    if (!validate_credentials(&g_credentials)) {
        return false;
    }
    g_credentials_initialized = true;
    return true;
}

bool are_credentials_available(void)
{
    return ensure_loaded() && g_credentials.credentials_loaded;
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

/* Independent of the WiFi accessors on purpose: the key can be present when the
 * SSID is not (env-var CI build), and absent when the SSID is (Improv). */
const char *get_gemini_api_key(void)
{
    if (!ensure_loaded()) {
        return NULL;
    }
    return (strlen(g_credentials.gemini_api_key) > 0) ? g_credentials.gemini_api_key : NULL;
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

// ---------------------------------------------------------------------------
// MQTT broker credentials (issue #626)
// ---------------------------------------------------------------------------

#define NVS_MQTT_NAMESPACE "mqtt_auth"
#define NVS_KEY_MQTT_USER "user"
#define NVS_KEY_MQTT_PASS "pass"

/* Empty by default, so a credentials.h written before issue #626 — or the CMake
 * stub — leaves MQTT in read-only mode rather than failing to compile. */
#ifndef MQTT_USERNAME
#define MQTT_USERNAME ""
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif
/* A truncated credential is a wrong credential that looks configured, so an
 * over-long one is a build error rather than a silent strlcpy cut. */
_Static_assert(sizeof(MQTT_USERNAME) <= MAX_MQTT_USERNAME_LENGTH,
               "MQTT_USERNAME in credentials.h is longer than 32 characters");
_Static_assert(sizeof(MQTT_PASSWORD) <= MAX_MQTT_PASSWORD_LENGTH,
               "MQTT_PASSWORD in credentials.h is longer than 64 characters");

static char s_mqtt_user[MAX_MQTT_USERNAME_LENGTH];
static char s_mqtt_pass[MAX_MQTT_PASSWORD_LENGTH];
static const char *s_mqtt_source = "none";
static bool s_mqtt_loaded = false;

/* NVS wins over credentials.h, the same priority the WiFi credentials use, so
 * a board can be given broker credentials over the serial console without a
 * rebuild. Only a COMPLETE NVS pair counts: a half-written entry falls back to
 * credentials.h instead of silently producing a username with no password. */
static void load_mqtt_credentials(void)
{
    /* Read once per boot: the MQTT client is created once, so the mode it
     * reports must be the mode it was started in. main.c primes this during
     * boot, before any other task can race the first load. */
    if (s_mqtt_loaded) {
        return;
    }
    s_mqtt_user[0] = '\0';
    s_mqtt_pass[0] = '\0';
    s_mqtt_source = "none";

    nvs_handle_t handle;
    if (nvs_open(NVS_MQTT_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        char user[MAX_MQTT_USERNAME_LENGTH] = {0};
        char pass[MAX_MQTT_PASSWORD_LENGTH] = {0};
        size_t user_len = sizeof(user);
        size_t pass_len = sizeof(pass);
        const bool ok = nvs_get_str(handle, NVS_KEY_MQTT_USER, user, &user_len) == ESP_OK &&
                        nvs_get_str(handle, NVS_KEY_MQTT_PASS, pass, &pass_len) == ESP_OK &&
                        user[0] != '\0' && pass[0] != '\0';
        nvs_close(handle);
        if (ok) {
            strlcpy(s_mqtt_user, user, sizeof(s_mqtt_user));
            strlcpy(s_mqtt_pass, pass, sizeof(s_mqtt_pass));
            s_mqtt_source = "nvs";
            s_mqtt_loaded = true;
            return;
        }
    }

    if (MQTT_USERNAME[0] != '\0' && MQTT_PASSWORD[0] != '\0') {
        strlcpy(s_mqtt_user, MQTT_USERNAME, sizeof(s_mqtt_user));
        strlcpy(s_mqtt_pass, MQTT_PASSWORD, sizeof(s_mqtt_pass));
        s_mqtt_source = "credentials.h";
    }
    s_mqtt_loaded = true;
}

const char *get_mqtt_username(void)
{
    load_mqtt_credentials();
    return s_mqtt_user[0] != '\0' ? s_mqtt_user : NULL;
}

const char *get_mqtt_password(void)
{
    load_mqtt_credentials();
    return s_mqtt_pass[0] != '\0' ? s_mqtt_pass : NULL;
}

const char *get_mqtt_credentials_source(void)
{
    load_mqtt_credentials();
    return s_mqtt_source;
}

bool credentials_nvs_save_mqtt(const char *username, const char *password)
{
    if (!username || !password || username[0] == '\0' || password[0] == '\0' ||
        strlen(username) >= MAX_MQTT_USERNAME_LENGTH ||
        strlen(password) >= MAX_MQTT_PASSWORD_LENGTH) {
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_MQTT_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for MQTT credentials: %s", esp_err_to_name(err));
        return false;
    }
    const bool ok = nvs_set_str(handle, NVS_KEY_MQTT_USER, username) == ESP_OK &&
                    nvs_set_str(handle, NVS_KEY_MQTT_PASS, password) == ESP_OK &&
                    nvs_commit(handle) == ESP_OK;
    nvs_close(handle);

    /* The username is logged, the password never is. */
    if (ok) {
        ESP_LOGI(TAG, "MQTT credentials saved to NVS (user: %s) — applied at next boot", username);
    } else {
        ESP_LOGE(TAG, "Failed to save MQTT credentials to NVS");
    }
    return ok;
}

bool credentials_nvs_clear_mqtt(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_MQTT_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for MQTT credentials: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear MQTT credentials: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "MQTT credentials cleared from NVS — applied at next boot");
    return true;
}
