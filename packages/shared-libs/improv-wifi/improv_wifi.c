/**
 * @file improv_wifi.c
 * @brief Improv WiFi Serial protocol implementation
 *
 * Packet format (https://www.improv-wifi.com/serial/):
 *   HEADER(6)  VERSION(1)  TYPE(1)  LENGTH(1)  DATA(LENGTH)  CHECKSUM(1)
 *
 * HEADER  = "IMPROV" (ASCII)
 * VERSION = 0x01
 * CHECKSUM = (VERSION + TYPE + LENGTH + sum(DATA)) % 256
 */

#include "improv_wifi.h"
#include <string.h>
#include "driver/uart.h"
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
#define CMD_IDENTIFY 0x02
#define CMD_REQUEST_INFO 0x03

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
} g_ctx;

// Calculate packet checksum
static uint8_t packet_checksum(uint8_t version, uint8_t type, uint8_t length, const uint8_t *data)
{
    uint8_t sum = version + type + length;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// Write a complete Improv packet to UART0
static void send_packet(uint8_t type, const uint8_t *data, uint8_t length)
{
    // Max packet size: HEADER_LEN + 1(ver) + 1(type) + 1(len) + MAX_DATA + 1(chk)
    uint8_t pkt[HEADER_LEN + 3 + MAX_DATA + 1];
    uint8_t idx = 0;

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
    uart_write_bytes(UART_NUM_0, (const char *)pkt, idx);
}

// Handle the "Send WiFi Credentials" RPC command
static void handle_send_wifi_creds(const uint8_t *data, uint8_t length)
{
    // DATA layout: [CMD_ID, ssid_len, ssid..., pass_len, pass...]
    if (length < 3) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }

    unsigned int idx = 1;  // Skip CMD_ID byte (use unsigned int to avoid uint8_t overflow)

    unsigned int ssid_len = data[idx++];
    if (idx + ssid_len > (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    char ssid[IMPROV_WIFI_MAX_SSID_LEN + 1];
    uint8_t copy_len =
        (ssid_len < IMPROV_WIFI_MAX_SSID_LEN) ? (uint8_t)ssid_len : IMPROV_WIFI_MAX_SSID_LEN;
    memcpy(ssid, data + idx, copy_len);
    ssid[copy_len] = '\0';
    idx += ssid_len;

    if (idx >= (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    unsigned int pass_len = data[idx++];
    if (idx + pass_len > (unsigned int)length) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    char password[IMPROV_WIFI_MAX_PASS_LEN + 1];
    copy_len = (pass_len < IMPROV_WIFI_MAX_PASS_LEN) ? (uint8_t)pass_len : IMPROV_WIFI_MAX_PASS_LEN;
    memcpy(password, data + idx, copy_len);
    password[copy_len] = '\0';

    ESP_LOGI(TAG, "Received WiFi credentials for SSID: %s", ssid);
    improv_wifi_send_state(IMPROV_STATE_PROVISIONING);
    if (g_ctx.credentials_cb != NULL) {
        g_ctx.credentials_cb(ssid, password);
    }
}

// Handle the "Request Device Info" RPC command
static void handle_request_info(void)
{
    uint8_t result[80];
    uint8_t ri = 0;
    result[ri++] = CMD_REQUEST_INFO;

    // firmware_name, firmware_version, hardware_name, device_url
    const char *parts[4] = {"RoboCar", "1.0", "ESP32", ""};
    for (int i = 0; i < 4; i++) {
        uint8_t plen = (uint8_t)strlen(parts[i]);
        result[ri++] = plen;
        if (plen > 0) {
            memcpy(result + ri, parts[i], plen);
            ri += plen;
        }
    }
    send_packet(TYPE_RPC_RESULT, result, ri);
}

// Dispatch a parsed RPC command
static void dispatch_rpc(const uint8_t *data, uint8_t length)
{
    if (length == 0) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    switch (data[0]) {
        case CMD_SEND_WIFI_CREDS:
            handle_send_wifi_creds(data, length);
            break;
        case CMD_IDENTIFY:
            ESP_LOGI(TAG, "Identify request received");
            // Application can flash an LED here if desired
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

esp_err_t improv_wifi_init(improv_credentials_cb_t cb)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.state = S_HEADER;
    g_ctx.credentials_cb = cb;
    ESP_LOGI(TAG, "Improv WiFi initialized — listening for provisioning");
    return ESP_OK;
}

void improv_wifi_process_byte(uint8_t byte)
{
    switch (g_ctx.state) {
        case S_HEADER:
            if (byte == HEADER[g_ctx.header_pos]) {
                g_ctx.header_pos++;
                if (g_ctx.header_pos == HEADER_LEN) {
                    g_ctx.header_pos = 0;
                    g_ctx.state = S_VERSION;
                }
            } else {
                // Resync: check if this byte starts a new header
                g_ctx.header_pos = (byte == HEADER[0]) ? 1 : 0;
            }
            break;

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
}

void improv_wifi_send_state(improv_state_t state)
{
    uint8_t s = (uint8_t)state;
    send_packet(TYPE_CURRENT_STATE, &s, 1);
    ESP_LOGD(TAG, "Sent state: 0x%02x", state);
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
    uint8_t data[128];
    uint8_t idx = 0;
    data[idx++] = CMD_SEND_WIFI_CREDS;  // RPC result ID
    if (redirect_url != NULL && redirect_url[0] != '\0') {
        uint8_t url_len = (uint8_t)strlen(redirect_url);
        data[idx++] = url_len;
        memcpy(data + idx, redirect_url, url_len);
        idx += url_len;
    } else {
        data[idx++] = 0;  // Empty redirect URL
    }
    send_packet(TYPE_RPC_RESULT, data, idx);
}
