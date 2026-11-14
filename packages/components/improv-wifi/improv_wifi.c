/**
 * @file improv_wifi.c
 * @brief Improv WiFi Serial protocol implementation
 *
 * Packet format (https://www.improv-wifi.com/serial/):
 *   HEADER(6)  VERSION(1)  TYPE(1)  LENGTH(1)  DATA(LENGTH)  CHECKSUM(1)  '\n'
 *
 * HEADER   = "IMPROV" (ASCII)
 * VERSION  = 0x01
 * CHECKSUM = low byte of the sum of every byte before it, HEADER included
 *
 * The spec page does not state the checksum algorithm; this is the one both
 * reference implementations use: improv-wifi/sdk-serial-js (the client ESP
 * Web Tools runs; writePacketToStream() and _handleIncomingPacket() in
 * src/serial.ts) and ESPHome's improv_serial component. Both also end every
 * packet with a newline after the checksum.
 *
 * RPC command data:  [command, data length, payload...]
 * RPC result data:   [command, data length, (string length, string)...]
 */

#include "improv_wifi.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "improv_wifi";

// Packet header
static const uint8_t HEADER[] = {'I', 'M', 'P', 'R', 'O', 'V'};
#define HEADER_LEN 6
#define VERSION 0x01
#define MAX_DATA 255

// Packet type constants
#define TYPE_CURRENT_STATE 0x01
#define TYPE_ERROR_STATE 0x02
#define TYPE_RPC_COMMAND 0x03
#define TYPE_RPC_RESULT 0x04

// RPC command IDs
#define CMD_SEND_WIFI_CREDS 0x01
#define CMD_REQUEST_STATE 0x02
#define CMD_REQUEST_INFO 0x03

// Longest redirect URL kept for the RPC 0x02 replay.
#define MAX_URL_LEN 127

// Internal parser state machine
typedef enum {
    S_HEADER,
    S_VERSION,
    S_TYPE,
    S_LENGTH,
    S_DATA,
    S_CHECKSUM,
} parse_state_t;

static struct {
    parse_state_t state;
    uint8_t header_pos;
    uint8_t pkt_version;
    uint8_t pkt_type;
    uint8_t pkt_length;
    uint8_t data[MAX_DATA];
    uint8_t data_pos;
    improv_credentials_cb_t credentials_cb;
    improv_state_t current_state;        // what RPC 0x02 reports
    char redirect_url[MAX_URL_LEN + 1];  // replayed by RPC 0x02 when provisioned
} g_ctx = {.current_state = IMPROV_STATE_AUTHORIZED};

/* Outside g_ctx on purpose: improv_wifi_init() clears g_ctx, and a writer set
 * before init must survive it (robocar sets it first). */
static improv_wifi_write_fn_t s_writer;

// Low byte of the sum of `len` bytes, added to `sum`.
static uint8_t sum_bytes(uint8_t sum, const uint8_t *bytes, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return sum;
}

// Checksum of a packet: header, version, type, length and data.
static uint8_t packet_checksum(uint8_t version, uint8_t type, uint8_t length, const uint8_t *data)
{
    const uint8_t fields[] = {version, type, length};
    uint8_t sum = sum_bytes(0, HEADER, HEADER_LEN);
    sum = sum_bytes(sum, fields, sizeof(fields));
    return sum_bytes(sum, data, length);
}

// Write a complete Improv packet through the configured transport, in one
// call so a concurrent log line cannot land in the middle of it.
static void send_packet(uint8_t type, const uint8_t *data, uint8_t length)
{
    // HEADER_LEN + 1(ver) + 1(type) + 1(len) + MAX_DATA + 1(chk) + 1('\n')
    uint8_t pkt[HEADER_LEN + 3 + MAX_DATA + 2];
    size_t idx = 0;  // up to 266 bytes, which a uint8_t index would wrap

    memcpy(pkt, HEADER, HEADER_LEN);
    idx += HEADER_LEN;
    pkt[idx++] = VERSION;
    pkt[idx++] = type;
    pkt[idx++] = length;
    if (length > 0 && data != NULL) {
        memcpy(pkt + idx, data, length);
        idx += length;
    }
    pkt[idx++] = packet_checksum(VERSION, type, length, data ? data : (const uint8_t *)"");
    pkt[idx++] = '\n';
    improv_wifi_write_fn_t write = (s_writer != NULL) ? s_writer : improv_wifi_uart0_write;
    write(pkt, idx);
}

// Send an RPC result: [command, data length, (string length, string)...].
// NULL or empty strings are skipped; strings that do not fit are truncated.
static void send_rpc_result(uint8_t command, const char *const *strings, size_t count)
{
    uint8_t data[MAX_DATA];
    size_t idx = 2;  // command and data length come first
    data[0] = command;
    for (size_t i = 0; i < count; i++) {
        if (strings[i] == NULL || strings[i][0] == '\0') {
            continue;
        }
        if (sizeof(data) - idx < 2) {
            ESP_LOGW(TAG, "RPC 0x%02x result full; %u string(s) dropped", command,
                     (unsigned)(count - i));
            break;
        }
        const size_t room = sizeof(data) - idx - 1;  // one byte for the length
        const size_t full = strlen(strings[i]);
        // The check above leaves idx <= 253, so room is 1..252. cppcheck takes the
        // branch that breaks and computes room as negative (it is a size_t).
        // cppcheck-suppress knownConditionTrueFalse
        const size_t len = full < room ? full : room;
        // A string of room bytes or more is truncated, and this logs it.
        // cppcheck-suppress knownConditionTrueFalse
        if (len < full) {
            ESP_LOGW(TAG, "RPC 0x%02x string %u truncated from %u to %u bytes", command,
                     (unsigned)i, (unsigned)full, (unsigned)len);
        }
        data[idx++] = (uint8_t)len;
        memcpy(data + idx, strings[i], len);
        idx += len;
    }
    data[1] = (uint8_t)(idx - 2);
    send_packet(TYPE_RPC_RESULT, data, (uint8_t)idx);
}

// The result a successful provisioning sends, echoing `command`: 0x01 after
// provisioning, 0x02 when replayed for Request current state.
static void send_provisioning_result(uint8_t command)
{
    const char *url = g_ctx.redirect_url;
    send_rpc_result(command, &url, 1);
}

// Handle the "Send WiFi Credentials" RPC command
static void handle_send_wifi_creds(const uint8_t *payload, uint8_t length)
{
    // Payload layout: [ssid_len, ssid..., pass_len, pass...]
    unsigned int idx = 0;  // unsigned int, so idx + len cannot wrap a uint8_t

    if (length == 0) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    unsigned int ssid_len = payload[idx++];
    if (idx + ssid_len > (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    char ssid[IMPROV_WIFI_MAX_SSID_LEN + 1];
    uint8_t copy_len =
        (ssid_len < IMPROV_WIFI_MAX_SSID_LEN) ? (uint8_t)ssid_len : IMPROV_WIFI_MAX_SSID_LEN;
    memcpy(ssid, payload + idx, copy_len);
    ssid[copy_len] = '\0';
    idx += ssid_len;

    if (idx >= (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    unsigned int pass_len = payload[idx++];
    if (idx + pass_len > (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    char password[IMPROV_WIFI_MAX_PASS_LEN + 1];
    copy_len = (pass_len < IMPROV_WIFI_MAX_PASS_LEN) ? (uint8_t)pass_len : IMPROV_WIFI_MAX_PASS_LEN;
    memcpy(password, payload + idx, copy_len);
    password[copy_len] = '\0';

    ESP_LOGI(TAG, "Received WiFi credentials for SSID: %s", ssid);
    improv_wifi_send_state(IMPROV_STATE_PROVISIONING);
    if (g_ctx.credentials_cb != NULL) {
        g_ctx.credentials_cb(ssid, password);
    }
}

// Handle "Request current state": the state, and when provisioned the same
// result a successful provisioning sends, echoing command 0x02.
static void handle_request_state(void)
{
    improv_wifi_send_state(g_ctx.current_state);
    if (g_ctx.current_state == IMPROV_STATE_PROVISIONED) {
        send_provisioning_result(CMD_REQUEST_STATE);
    }
}

// Handle "Request device info": firmware name, firmware version, chip, device name.
static void handle_request_info(void)
{
    improv_device_info_t info = {0};
    improv_wifi_device_info(&info);
    const char *strings[] = {info.firmware_name, info.firmware_version, info.chip,
                             info.device_name};
    send_rpc_result(CMD_REQUEST_INFO, strings, sizeof(strings) / sizeof(strings[0]));
}

// Dispatch a parsed RPC command: data = [command, data length, payload...].
static void dispatch_rpc(const uint8_t *data, uint8_t length)
{
    if (length < 2 || data[1] != length - 2) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    // The spec: on every RPC command, first clear the error state.
    improv_wifi_send_error(IMPROV_ERROR_NONE);

    const uint8_t *payload = data + 2;
    const uint8_t payload_len = data[1];
    switch (data[0]) {
        case CMD_SEND_WIFI_CREDS:
            handle_send_wifi_creds(payload, payload_len);
            break;
        case CMD_REQUEST_STATE:
            handle_request_state();
            break;
        case CMD_REQUEST_INFO:
            handle_request_info();
            break;
        default:
            ESP_LOGW(TAG, "Unknown RPC command: 0x%02x", data[0]);
            improv_wifi_send_error(IMPROV_ERROR_UNKNOWN_CMD);
            break;
    }
}

// --- Public API ---

void improv_wifi_set_writer(improv_wifi_write_fn_t writer)
{
    s_writer = writer;
}

esp_err_t improv_wifi_init(improv_credentials_cb_t cb)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.state = S_HEADER;
    g_ctx.credentials_cb = cb;
    g_ctx.current_state = IMPROV_STATE_AUTHORIZED;
    ESP_LOGI(TAG, "Improv WiFi initialized — listening for provisioning");
    return ESP_OK;
}

void improv_wifi_reset_parser(void)
{
    g_ctx.state = S_HEADER;
    g_ctx.header_pos = 0;
    g_ctx.data_pos = 0;
}

bool improv_wifi_parser_idle(void)
{
    return g_ctx.state == S_HEADER && g_ctx.header_pos == 0;
}

bool improv_wifi_process_byte(uint8_t byte)
{
    switch (g_ctx.state) {
        case S_HEADER:
            if (byte == HEADER[g_ctx.header_pos]) {
                g_ctx.header_pos++;
                if (g_ctx.header_pos == HEADER_LEN) {
                    g_ctx.header_pos = 0;
                    g_ctx.state = S_VERSION;
                    return true;  // the header is complete: a packet from here on
                }
            } else {
                // Resync: check if this byte starts a new header
                g_ctx.header_pos = (byte == HEADER[0]) ? 1 : 0;
            }
            return false;

        case S_VERSION:
            g_ctx.pkt_version = byte;
            g_ctx.state = S_TYPE;
            break;

        case S_TYPE:
            g_ctx.pkt_type = byte;
            g_ctx.state = S_LENGTH;
            break;

        case S_LENGTH:
            g_ctx.pkt_length = byte;
            g_ctx.data_pos = 0;
            g_ctx.state = (byte > 0) ? S_DATA : S_CHECKSUM;
            break;

        case S_DATA:
            g_ctx.data[g_ctx.data_pos++] = byte;
            if (g_ctx.data_pos >= g_ctx.pkt_length) {
                g_ctx.state = S_CHECKSUM;
            }
            break;

        case S_CHECKSUM: {
            uint8_t expected =
                packet_checksum(g_ctx.pkt_version, g_ctx.pkt_type, g_ctx.pkt_length, g_ctx.data);
            g_ctx.state = S_HEADER;
            if (byte != expected) {
                ESP_LOGW(TAG, "Checksum mismatch: got=0x%02x expected=0x%02x", byte, expected);
                improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
                break;
            }
            if (g_ctx.pkt_type == TYPE_RPC_COMMAND) {
                dispatch_rpc(g_ctx.data, g_ctx.pkt_length);
            }
            break;
        }
    }
    return true;  // version, type, length, data or checksum
}

void improv_wifi_send_state(improv_state_t state)
{
    g_ctx.current_state = state;
    uint8_t s = (uint8_t)state;
    send_packet(TYPE_CURRENT_STATE, &s, 1);
    ESP_LOGD(TAG, "Sent state: 0x%02x", state);
}

void improv_wifi_announce(void)
{
    improv_wifi_send_state(g_ctx.current_state);
}

void improv_wifi_fail_provisioning(improv_error_t error)
{
    // The order ESPHome's improv_serial uses when an attempt times out
    // (on_wifi_connect_timeout_() in ESPHome's
    // components/improv_serial/improv_serial_component.cpp): the error, then
    // back to Ready.
    improv_wifi_send_error(error);
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
}

void improv_wifi_send_error(improv_error_t error)
{
    uint8_t e = (uint8_t)error;
    send_packet(TYPE_ERROR_STATE, &e, 1);
    if (error != IMPROV_ERROR_NONE) {
        ESP_LOGW(TAG, "Sent error: 0x%02x", error);
    }
}

void improv_wifi_send_provisioned_result(const char *redirect_url)
{
    const char *url = (redirect_url != NULL) ? redirect_url : "";
    if (strlen(url) > MAX_URL_LEN) {
        ESP_LOGW(TAG, "redirect_url truncated from %u to %u bytes", (unsigned)strlen(url),
                 (unsigned)MAX_URL_LEN);
    }
    strncpy(g_ctx.redirect_url, url, MAX_URL_LEN);
    g_ctx.redirect_url[MAX_URL_LEN] = '\0';
    send_provisioning_result(CMD_SEND_WIFI_CREDS);
}
