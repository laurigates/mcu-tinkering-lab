/**
 * @file credentials_validator.h
 * @brief Compile-time validation for credentials
 */

#ifndef CREDENTIALS_VALIDATOR_H
#define CREDENTIALS_VALIDATOR_H

#include <string.h>
#include "credentials.h"
#include "esp_log.h"

// WiFi credentials: allow empty placeholders so that pre-built (web flasher)
// firmware compiles without real credentials.  The Improv WiFi provisioner
// will supply real credentials at runtime and store them in NVS.
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifdef CONFIG_AI_BACKEND_CLAUDE
#ifndef CLAUDE_API_KEY
#error \
    "CLAUDE_API_KEY is not defined in credentials.h but Claude backend is selected. Please add your Claude API key to credentials.h."
#endif
#endif

// Runtime validation function that will be called during initialization.
// WiFi credential checks are advisory: placeholder or empty values mean
// that Improv WiFi provisioning is required at runtime (NVS will supply them).
// API key checks remain fatal because they cannot be provisioned at runtime.
static inline void validate_credentials_at_runtime(void)
{
    // Check for default WiFi SSID values — warn only, Improv WiFi will provision
    if (strcmp(WIFI_SSID, "") == 0 || strcmp(WIFI_SSID, "your_wifi_network_name") == 0 ||
        strcmp(WIFI_SSID, "YOUR_WIFI_NAME") == 0 ||
        strcmp(WIFI_SSID, "YOUR_WIFI_SSID_HERE") == 0) {
        ESP_LOGW("credentials_validator",
                 "WiFi SSID not configured in credentials.h (value: '%s'). "
                 "Improv WiFi provisioning will be required after flashing.",
                 WIFI_SSID);
        return;  // Skip password check when SSID is already invalid
    }

    // Check for default WiFi password values — warn only
    if (strcmp(WIFI_PASSWORD, "") == 0 ||
        strcmp(WIFI_PASSWORD, "your_wifi_password") == 0 ||
        strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD") == 0 ||
        strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD_HERE") == 0) {
        ESP_LOGW("credentials_validator",
                 "WiFi password not configured in credentials.h. "
                 "Improv WiFi provisioning will be required after flashing.");
    }

#ifdef CONFIG_AI_BACKEND_CLAUDE
    // Check for default Claude API key values
    if (strcmp(CLAUDE_API_KEY, "your_claude_api_key_from_anthropic") == 0 ||
        strcmp(CLAUDE_API_KEY, "sk-ant-api03-YOUR_ACTUAL_API_KEY_HERE") == 0) {
        ESP_LOGE("credentials_validator",
                 "CLAUDE_API_KEY in credentials.h contains default value. "
                 "Please edit credentials.h with your actual Claude API key from Anthropic.");
        abort();
    }
#endif
}

#endif  // CREDENTIALS_VALIDATOR_H
