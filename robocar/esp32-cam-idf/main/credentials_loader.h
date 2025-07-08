/**
 * @file credentials_loader.h
 * @brief Secure credential loading with environment variable support
 * 
 * This module provides secure credential loading that:
 * 1. Checks environment variables first (for CI/CD)
 * 2. Falls back to credentials.h file
 * 3. Provides validation and secure defaults
 */

#ifndef CREDENTIALS_LOADER_H
#define CREDENTIALS_LOADER_H

#include <stdbool.h>

// Maximum lengths for credentials
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_API_KEY_LENGTH 256

/**
 * @brief Structure to hold loaded credentials
 */
typedef struct {
    char wifi_ssid[MAX_SSID_LENGTH];
    char wifi_password[MAX_PASSWORD_LENGTH];
    char claude_api_key[MAX_API_KEY_LENGTH];
    bool credentials_loaded;
} credentials_t;

/**
 * @brief Load credentials from environment variables or credentials.h
 * 
 * This function attempts to load credentials in the following order:
 * 1. Environment variables (WIFI_SSID, WIFI_PASSWORD, CLAUDE_API_KEY)
 * 2. Fallback to credentials.h file
 * 
 * @param creds Pointer to credentials structure to populate
 * @return true if credentials were loaded successfully, false otherwise
 */
bool load_credentials(credentials_t* creds);

/**
 * @brief Validate loaded credentials
 * 
 * @param creds Pointer to credentials structure to validate
 * @return true if credentials are valid, false otherwise
 */
bool validate_credentials(const credentials_t* creds);

/**
 * @brief Get WiFi SSID
 * 
 * @return Pointer to WiFi SSID string
 */
const char* get_wifi_ssid(void);

/**
 * @brief Get WiFi password
 * 
 * @return Pointer to WiFi password string
 */
const char* get_wifi_password(void);

/**
 * @brief Get Claude API key
 * 
 * @return Pointer to Claude API key string
 */
const char* get_claude_api_key(void);

/**
 * @brief Check if credentials are available
 * 
 * @return true if credentials are loaded and valid, false otherwise
 */
bool are_credentials_available(void);

#endif // CREDENTIALS_LOADER_H