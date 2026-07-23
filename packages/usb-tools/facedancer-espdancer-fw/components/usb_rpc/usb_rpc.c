/**
 * @file usb_rpc.c
 * @brief Control-channel TCP server + framed-TLV dispatcher (Milestone 0).
 *
 * State machine for a single client connection:
 *   1. accept()
 *   2. read loop: resync on magic, parse frame, dispatch, write reply
 *   3. on socket error / disconnect, loop back to accept()
 *
 * Milestone 0 dispatch only implements HELLO + GET_VERSION. The remaining
 * FacedancerBackend primitives route to `components/raw_usb` (Milestone 1),
 * and return ERROR_NOT_IMPLEMENTED until then.
 */
#include "usb_rpc.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>    /* close() */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "sdkconfig.h"

static const char *TAG = "usb_rpc";

/* ── CRC8 (poly 0x07, init 0x00) — over the body bytes only ────────────── */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ── Single global connection state ─────────────────────────────────────── */
typedef struct {
    int sock;
    bool connected;
    uint8_t rxbuf[USB_RPC_MAX_PAYLOAD + 8];
} rpc_conn_t;

static rpc_conn_t s_conn = {.sock = -1, .connected = false};

bool usb_rpc_host_connected(void)
{
    return s_conn.connected;
}

/* ── Send helpers ──────────────────────────────────────────────────────── */

static esp_err_t send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    if (s_conn.sock < 0 || !s_conn.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > USB_RPC_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Body = msg_id + payload; CRC is over the concatenated body (matches
     * the receiver's crc8(body, total)). CRC is NOT distributive over
     * concatenation, so we must build the body in one buffer. */
    uint16_t total = (uint16_t)(len + 1);
    uint8_t body[USB_RPC_MAX_PAYLOAD + 1];
    body[0] = msg_id;
    if (len) memcpy(body + 1, payload, len);
    uint8_t crc = crc8(body, total);

    uint8_t hdr[4];
    hdr[0] = USB_RPC_MAGIC0;
    hdr[1] = USB_RPC_MAGIC1;
    hdr[2] = (uint8_t)(total >> 8);
    hdr[3] = (uint8_t)(total & 0xFF);

    int n = send(s_conn.sock, hdr, 4, 0);
    if (n != 4) return ESP_FAIL;
    n = send(s_conn.sock, body, total, 0);
    if (n != total) return ESP_FAIL;
    n = send(s_conn.sock, &crc, 1, 0);
    if (n != 1) return ESP_FAIL;
    return ESP_OK;
}

static esp_err_t send_ok(uint8_t in_reply_to)
{
    return send_frame(USB_RPC_OK, &in_reply_to, 1);
}

static esp_err_t send_error(uint8_t in_reply_to, uint8_t code, const char *msg)
{
    uint8_t buf[1 + 64];
    buf[0] = code;
    size_t m = msg ? strnlen(msg, 63) : 0;
    if (m) memcpy(buf + 1, msg, m);
    return send_frame(USB_RPC_ERROR, buf, (uint16_t)(1 + m));
}

esp_err_t usb_rpc_send_event(uint8_t event_type, const uint8_t *payload, uint16_t len)
{
    if (len > USB_RPC_MAX_PAYLOAD - 1) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[USB_RPC_MAX_PAYLOAD];
    buf[0] = event_type;
    if (len) memcpy(buf + 1, payload, len);
    return send_frame(USB_RPC_EVENT, buf, (uint16_t)(1 + len));
}

/* ── Dispatch ──────────────────────────────────────────────────────────── */

/* Forward declares filled in by Milestone 1/2. */
extern esp_err_t raw_usb_get_version(char *out, size_t out_len);

static void dispatch(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    switch (msg_id) {
        case USB_RPC_HELLO: {
            uint16_t peer_proto = (len >= 2) ? (uint16_t)((payload[0] << 8) | payload[1]) : 0;
            ESP_LOGI(TAG, "HELLO from host (proto 0x%04X)", peer_proto);
            uint8_t reply[2 + 16];
            reply[0] = (uint8_t)(USB_RPC_PROTO_VERSION >> 8);
            reply[1] = (uint8_t)(USB_RPC_PROTO_VERSION & 0xFF);
            strncpy((char *)reply + 2, "espdancer-0.1.0", 16);
            reply[2 + 15] = 0;
            send_frame(USB_RPC_HELLO_REPLY, reply, sizeof(reply));
            break;
        }
        case USB_RPC_GET_VERSION: {
            char v[32] = {0};
#if CONFIG_HAS_RAW_USB
            raw_usb_get_version(v, sizeof(v));
#else
            strncpy(v, "espdancer-fw 0.1.0 (raw_usb stub)", sizeof(v) - 1);
#endif
            send_frame(USB_RPC_VERSION_REPLY, (uint8_t *)v, (uint16_t)strnlen(v, 31));
            break;
        }
        case USB_RPC_CONNECT:
        case USB_RPC_DISCONNECT:
        case USB_RPC_RESET:
        case USB_RPC_SET_ADDRESS:
        case USB_RPC_CONFIGURE_EP:
        case USB_RPC_SEND:
        case USB_RPC_STALL:
        case USB_RPC_CLEAR_HALT:
        case USB_RPC_ACK_STATUS:
            /* Pended on `components/raw_usb` (Milestone 1). */
            ESP_LOGW(TAG, "msg 0x%02X not implemented yet (raw_usb stub)", msg_id);
            send_error(msg_id, USB_RPC_ERR_NOT_IMPLEMENTED,
                       "raw_usb component not implemented (Milestone 1)");
            break;

        default:
            ESP_LOGW(TAG, "unknown msg 0x%02X", msg_id);
            send_error(msg_id, USB_RPC_ERR_BAD_FRAME, "unknown message id");
            break;
    }
}

/* ── Frame reader ───────────────────────────────────────────────────────── */

static bool read_full(int sock, uint8_t *dst, size_t want)
{
    size_t got = 0;
    while (got < want) {
        int n = recv(sock, dst + got, (int)(want - got), 0);
        if (n <= 0) return false;
        got += (size_t)n;
    }
    return true;
}

static void handle_connection(int sock)
{
    s_conn.sock = sock;
    s_conn.connected = true;
    ESP_LOGI(TAG, "Host control channel connected");
    /* LED state is driven from main's idle loop, which polls
     * usb_rpc_host_connected(). */

    while (s_conn.connected) {
        /* Resync on magic. */
        uint8_t b[2];
        if (!read_full(sock, b, 1)) break;
        if (b[0] != USB_RPC_MAGIC0) continue;
        if (!read_full(sock, b, 1)) break;
        if (b[0] != USB_RPC_MAGIC1) continue;

        /* Length (body = msg_id + payload). */
        uint8_t lenb[2];
        if (!read_full(sock, lenb, 2)) break;
        uint16_t total = (uint16_t)((lenb[0] << 8) | lenb[1]);
        if (total == 0 || total > (USB_RPC_MAX_PAYLOAD + 1)) {
            ESP_LOGW(TAG, "bad frame len %u", total);
            continue;
        }

        /* Body. */
        uint8_t body[USB_RPC_MAX_PAYLOAD + 1];
        if (!read_full(sock, body, total)) break;
        /* CRC. */
        uint8_t crc_in;
        if (!read_full(sock, &crc_in, 1)) break;
        if (crc8(body, total) != crc_in) {
            ESP_LOGW(TAG, "crc mismatch — dropping frame");
            continue;
        }

        uint8_t msg_id = body[0];
        dispatch(msg_id, body + 1, (uint16_t)(total - 1));
    }

    s_conn.connected = false;
    s_conn.sock = -1;
    ESP_LOGW(TAG, "Host control channel disconnected");
}

/* ── TCP server task ────────────────────────────────────────────────────── */

static void rpc_task(void *arg)
{
    (void)arg;
    int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(USB_RPC_PORT);
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno);
        close(srv);
        vTaskDelete(NULL);
        return;
    }
    if (listen(srv, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno %d", errno);
        close(srv);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Control channel listening on TCP %d (AP 192.168.4.1)", USB_RPC_PORT);

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) {
            ESP_LOGW(TAG, "accept() failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        handle_connection(cli);
        close(cli);
    }
    close(srv);
    vTaskDelete(NULL);
}

esp_err_t usb_rpc_start(void)
{
    BaseType_t r = xTaskCreate(rpc_task, "usb_rpc", 6144, NULL, 4, NULL);
    return (r == pdPASS) ? ESP_OK : ESP_FAIL;
}