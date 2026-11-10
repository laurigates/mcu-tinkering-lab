/**
 * @file improv_wifi.h
 * @brief Improv WiFi Serial protocol implementation
 *
 * Implements the Improv WiFi Serial spec (https://www.improv-wifi.com/serial/).
 * ESP Web Tools automatically detects Improv support and shows a WiFi
 * configuration dialog after flashing when this protocol is active.
 *
 * Usage:
 *   0. If the serial port you read from is not UART0 (e.g. a USB-Serial-JTAG
 *      console), call improv_wifi_set_writer() first so replies go back over
 *      that same port. The default writer is UART0, and the caller must have
 *      installed the UART0 driver for it to transmit.
 *   1. Call improv_wifi_init(credentials_cb) when no stored credentials are found.
 *   2. Feed received serial bytes to improv_wifi_process_byte().
 *   3. Call improv_wifi_send_state() to broadcast device state (call periodically
 *      while waiting for provisioning, ~1 s interval).
 *   4. Your credentials_cb is invoked when the browser sends SSID + password.
 *   5. After connecting, call improv_wifi_send_state(IMPROV_STATE_PROVISIONED)
 *      and improv_wifi_send_provisioned_result().
 *
 * The parser answers RPCs 0x01 (Send Wi-Fi settings), 0x02 (Request current
 * state: the last state sent, plus the provisioning result replayed when
 * provisioned) and 0x03 (Request device info, from improv_wifi_device_info()).
 * Any other RPC gets error 0x02 (Unknown RPC command), which clients treat as
 * "optional command not supported".
 */

#ifndef IMPROV_WIFI_H
#define IMPROV_WIFI_H

#include <stdbool.h>
#include <stddef.h>
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

/* Error codes of the Improv Serial spec. 0xFE is not one: the client
 * (improv-wifi/sdk-serial-js) uses it for its own local timeout. */
typedef enum {
    IMPROV_ERROR_NONE = 0x00,
    IMPROV_ERROR_INVALID_RPC = 0x01,
    IMPROV_ERROR_UNKNOWN_CMD = 0x02,
    IMPROV_ERROR_UNABLE_CONNECT = 0x03,
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
 * @brief Transport for outgoing Improv packets.
 *
 * Receives one complete packet per call, ending in the newline the reference
 * implementations send after the checksum. It must write the bytes unmodified —
 * no line-ending translation — because packets are binary and any byte,
 * including 0x0A, can occur in the length, data or checksum.
 */
typedef void (*improv_wifi_write_fn_t)(const uint8_t *data, size_t len);

/**
 * @brief The four strings RPC 0x03 (Request device info) returns, in order.
 */
typedef struct {
    const char *firmware_name;     ///< Equal to the flasher manifest's `name`
    const char *firmware_version;  ///< Equal to the manifest's `version`
    const char *chip;              ///< Chip family, e.g. "ESP32-S3"
    const char *device_name;       ///< The device's own name
} improv_device_info_t;

/**
 * @brief Fill in the device info strings. Platform-provided.
 *
 * The ESP-IDF definition (improv_wifi_device_info.c) reads them from the
 * running image: the firmware name from the project's flasher.json `name`
 * (falling back to the CMake project name), the version and project name from
 * esp_app_get_description(), and the chip from the build target. The strings
 * must stay valid for the life of the program. Host tests supply their own.
 */
void improv_wifi_device_info(improv_device_info_t *out);

/**
 * @brief Select where Improv replies are written.
 *
 * Replies must leave over the port the requests arrive on, or the browser
 * never hears back. May be called before or after improv_wifi_init(); init
 * does not reset it.
 *
 * @param writer  Transport to use, or NULL for the default,
 *                improv_wifi_uart0_write().
 */
void improv_wifi_set_writer(improv_wifi_write_fn_t writer);

/**
 * @brief Default transport: uart_write_bytes() on UART0.
 *
 * Requires the UART0 driver to be installed (uart_driver_install()); without
 * it the write fails and nothing is transmitted.
 */
void improv_wifi_uart0_write(const uint8_t *data, size_t len);

/**
 * @brief Initialize the Improv WiFi parser.
 *
 * Resets the state RPC 0x02 reports to IMPROV_STATE_AUTHORIZED (ready for
 * credentials) and forgets any stored provisioning result.
 *
 * @param cb  Callback invoked when credentials are received. Must not be NULL.
 * @return ESP_OK on success.
 */
esp_err_t improv_wifi_init(improv_credentials_cb_t cb);

/**
 * @brief Process one byte received from the serial port.
 *
 * Call this for every byte read from the serial port while Improv WiFi is
 * active. Bytes must arrive untranslated: a console that maps CR to LF on
 * input corrupts any packet containing 0x0D. The function is safe to call even for bytes that
 * belong to normal ASCII commands; non-Improv bytes are silently discarded by the parser.
 */
void improv_wifi_process_byte(uint8_t byte);

/**
 * @brief Send the current device state to the browser.
 *
 * Call this periodically (~1 s) while waiting for provisioning so that
 * ESP Web Tools can detect the device.  Also call after state transitions
 * (e.g. PROVISIONING, PROVISIONED). The state is also remembered, and is what
 * RPC 0x02 (Request current state) reports.
 */
void improv_wifi_send_state(improv_state_t state);

/**
 * @brief Send an error code to the browser.
 */
void improv_wifi_send_error(improv_error_t error);

/**
 * @brief Send the RPC result after successful provisioning.
 *
 * The URL is also remembered: RPC 0x02 replays this result while the state
 * is IMPROV_STATE_PROVISIONED, as the spec requires.
 *
 * @param redirect_url  Optional URL to redirect the browser after provisioning.
 *                      Pass NULL or "" to send no redirect URL; the result then
 *                      carries no strings (an empty string would make ESP Web
 *                      Tools show a "Visit Device" link to nowhere).
 */
void improv_wifi_send_provisioned_result(const char *redirect_url);

#endif  // IMPROV_WIFI_H
