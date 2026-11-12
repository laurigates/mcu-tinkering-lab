/**
 * @file credentials_loader.c
 * @brief Secure credential loading implementation
 */

#include "credentials_loader.h"
#include <stdlib.h>
#include <string.h>
#include "credentials.h"
#include "esp_log.h"
#include "gemini_key_state.h"
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
 * credentials here come from three independent sources, so one missing value
 * must not invalidate the others: an open network has an SSID but no
 * password. Only a usable SSID is genuinely required. The Gemini key is
 * loaded separately, below, because it must not depend on where WiFi came
 * from (issue #845). */
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

// ---------------------------------------------------------------------------
// Gemini API key (issue #845)
// ---------------------------------------------------------------------------

#define NVS_GEMINI_NAMESPACE "gemini_auth"
#define NVS_KEY_GEMINI "key"

/* Empty in the CMake stub that CI, web-flasher and OTA images are built with,
 * so a released image never carries a key. */
#ifndef GEMINI_API_KEY
#define GEMINI_API_KEY ""
#endif
_Static_assert(sizeof(GEMINI_API_KEY) <= MAX_API_KEY_LENGTH,
               "GEMINI_API_KEY in credentials.h is longer than 255 characters");

/* Two buffers and an index, so a key changed from the console never rewrites
 * the string a request on another task is sending: the new key is written
 * into the idle buffer and published with one store. A pointer handed out
 * before the change stays valid until the change after that, which is far
 * longer than any request. */
static char s_gemini_key[2][MAX_API_KEY_LENGTH];
static volatile int s_gemini_active = -1; /* -1: not loaded yet */
static const char *volatile s_gemini_source = "none";

/* A key goes into an HTTP header, so anything but printable ASCII without
 * spaces is refused: a CR or LF would end the header early. */
static bool gemini_key_is_valid(const char *key)
{
    if (!key || key[0] == '\0' || strlen(key) >= MAX_API_KEY_LENGTH) {
        return false;
    }
    for (const char *p = key; *p != '\0'; ++p) {
        if (*p <= ' ' || *p > '~') {
            return false;
        }
    }
    return true;
}

/* NVS, then credentials.h, then the environment, whatever the WiFi source.
 * The WiFi path used to read the key from getenv() alone once Improv had
 * written NVS, so provisioning dropped a key compiled in from credentials.h. */
static const char *gemini_key_lookup(char *dest)
{
    dest[0] = '\0';

    nvs_handle_t handle;
    if (nvs_open(NVS_GEMINI_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        size_t len = MAX_API_KEY_LENGTH;
        const bool ok =
            nvs_get_str(handle, NVS_KEY_GEMINI, dest, &len) == ESP_OK && gemini_key_is_valid(dest);
        nvs_close(handle);
        if (ok) {
            return "nvs";
        }
        dest[0] = '\0';
    }

    if (gemini_key_is_valid(GEMINI_API_KEY)) {
        strlcpy(dest, GEMINI_API_KEY, MAX_API_KEY_LENGTH);
        return "credentials.h";
    }

    if (load_from_env("GEMINI_API_KEY", dest, MAX_API_KEY_LENGTH) && gemini_key_is_valid(dest)) {
        return "env";
    }
    dest[0] = '\0';
    return "none";
}

static void gemini_key_publish(void)
{
    const int next = (s_gemini_active == 0) ? 1 : 0;
    const char *source = gemini_key_lookup(s_gemini_key[next]);
    s_gemini_source = source;
    s_gemini_active = next;
    gemini_key_state_reset();
    ESP_LOGI(TAG, "Gemini API key: %s", strcmp(source, "none") == 0 ? "none" : source);
}

const char *get_gemini_api_key(void)
{
    if (s_gemini_active < 0) {
        gemini_key_publish();
    }
    const char *key = s_gemini_key[s_gemini_active];
    return key[0] != '\0' ? key : NULL;
}

const char *get_gemini_api_key_source(void)
{
    (void)get_gemini_api_key();
    return s_gemini_source;
}

bool credentials_nvs_save_gemini(const char *key)
{
    if (!gemini_key_is_valid(key)) {
        return false;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_GEMINI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for the Gemini key: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_str(handle, NVS_KEY_GEMINI, key);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save the Gemini key: %s", esp_err_to_name(err));
        return false;
    }
    /* Presence only: the key itself is never logged. */
    ESP_LOGI(TAG, "Gemini key saved to NVS (%u characters) — in use now", (unsigned)strlen(key));
    gemini_key_publish();
    return true;
}

bool credentials_nvs_clear_gemini(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_GEMINI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for the Gemini key: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear the Gemini key: %s", esp_err_to_name(err));
        return false;
    }
    gemini_key_publish();
    return true;
}
