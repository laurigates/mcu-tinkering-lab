/**
 * @file credentials_validator.h
 * @brief Compile-time validation for credentials
 */

#ifndef CREDENTIALS_VALIDATOR_H
#define CREDENTIALS_VALIDATOR_H

#include "credentials.h"
#include <string.h>
#include "esp_log.h"

// Basic validation that credentials.h is included
#ifndef WIFI_SSID
    #error "WIFI_SSID is not defined in credentials.h. Please create credentials.h from credentials.h.example and configure your WiFi credentials."
#endif

#ifndef WIFI_PASSWORD
    #error "WIFI_PASSWORD is not defined in credentials.h. Please create credentials.h from credentials.h.example and configure your WiFi credentials."
#endif

#ifdef CONFIG_AI_BACKEND_CLAUDE
    #ifndef CLAUDE_API_KEY
        #error "CLAUDE_API_KEY is not defined in credentials.h but Claude backend is selected. Please add your Claude API key to credentials.h."
    #endif
#endif

// Runtime validation function that will be called during initialization
static inline void validate_credentials_at_runtime(void) {
    // Check for default WiFi SSID values
    if (strcmp(WIFI_SSID, "your_wifi_network_name") == 0 ||
        strcmp(WIFI_SSID, "YOUR_WIFI_NAME") == 0 ||
        strcmp(WIFI_SSID, "YOUR_WIFI_SSID_HERE") == 0) {
        ESP_LOGE("credentials_validator", 
                "WIFI_SSID in credentials.h contains default value '%s'. "
                "Please edit credentials.h with your actual WiFi network name.", WIFI_SSID);
        abort();
    }
    
    // Check for default WiFi password values
    if (strcmp(WIFI_PASSWORD, "your_wifi_password") == 0 ||
        strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD") == 0 ||
        strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD_HERE") == 0) {
        ESP_LOGE("credentials_validator", 
                "WIFI_PASSWORD in credentials.h contains default value. "
                "Please edit credentials.h with your actual WiFi password.");
        abort();
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

#endif // CREDENTIALS_VALIDATOR_H