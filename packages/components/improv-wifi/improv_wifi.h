/**
 * @file improv_wifi.h
 * @brief Improv WiFi Serial protocol implementation
 *
 * Implements the Improv WiFi Serial spec (https://www.improv-wifi.com/serial/).
 * ESP Web Tools automatically detects Improv support and shows a WiFi
 * configuration dialog after flashing when this protocol is active.
 *
 * Usage:
 *   1. Call improv_wifi_init(credentials_cb) when no stored credentials are found.
 *   2. Feed received UART bytes to improv_wifi_process_byte().
 *   3. Call improv_wifi_send_state() to broadcast device state (call periodically
 *      while waiting for provisioning, ~1 s interval).
 *   4. Your credentials_cb is invoked when the browser sends SSID + password.
 *   5. After connecting, call improv_wifi_send_state(IMPROV_STATE_PROVISIONED)
 *      and improv_wifi_send_provisioned_result().
 */

#ifndef IMPROV_WIFI_H
#define IMPROV_WIFI_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define IMPROV_WIFI_MAX_SSID_LEN 32
#define IMPROV_WIFI_MAX_PASS_LEN 64

typedef enum {
    IMPROV_STATE_AUTHORIZATION_REQUIRED = 0x01,
    IMPROV_STATE_AUTHORIZED = 0x02,
    IMPROV_STATE_PROVISIONING = 0x03,
    IMPROV_STATE_PROVISIONED = 0x04,
} improv_state_t;

typedef enum {
    IMPROV_ERROR_NONE = 0x00,
    IMPROV_ERROR_INVALID_RPC = 0x01,
    IMPROV_ERROR_UNKNOWN_CMD = 0x02,
    IMPROV_ERROR_TOO_MANY_REQUESTS = 0x03,
    IMPROV_ERROR_UNABLE_CONNECT = 0xFE,
    IMPROV_ERROR_UNKNOWN = 0xFF,
} improv_error_t;

/**
 * @brief Callback invoked when WiFi credentials are received from the browser.
 *
 * Called from within improv_wifi_process_byte() context.
 * The ssid and password buffers are valid only for the duration of the call.
 */
typedef void (*improv_credentials_cb_t)(const char *ssid, const char *password);

/**
 * @brief Initialize the Improv WiFi parser.
 *
 * @param cb  Callback invoked when credentials are received. Must not be NULL.
 * @return ESP_OK on success.
 */
esp_err_t improv_wifi_init(improv_credentials_cb_t cb);

/**
 * @brief Process one byte received from UART0.
 *
 * Call this for every byte read from the serial port while Improv WiFi is
 * active. The function is safe to call even for bytes that belong to normal
 * ASCII commands; non-Improv bytes are silently discarded by the parser.
 */
void improv_wifi_process_byte(uint8_t byte);

/**
 * @brief Send the current device state to the browser.
 *
 * Call this periodically (~1 s) while waiting for provisioning so that
 * ESP Web Tools can detect the device.  Also call after state transitions
 * (e.g. PROVISIONING, PROVISIONED).
 */
void improv_wifi_send_state(improv_state_t state);

/**
 * @brief Send an error code to the browser.
 */
void improv_wifi_send_error(improv_error_t error);

/**
 * @brief Send the RPC result after successful provisioning.
 *
 * @param redirect_url  Optional URL to redirect the browser after provisioning.
 *                      Pass NULL or "" to send no redirect URL.
 */
void improv_wifi_send_provisioned_result(const char *redirect_url);

#endif  // IMPROV_WIFI_H
