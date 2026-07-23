/**
 * @file usb_rpc.h
 * @brief Host↔device control channel for the espdancer backend.
 *
 * Wire protocol: length-prefixed framed TLV. See docs/rpc-protocol.md for
 * the authoritative, byte-level specification — this header is the firmware
 * mirror of that contract.
 *
 *   Frame:  [0xF0][0xD0] [u16 len_be] [u8 msg_id] [payload[len-1]] [u8 crc8]
 *
 * The host (facedancer `espdancer.py`) opens a TCP connection to the S3's
 * SoftAP (default 192.168.4.1:4444) and exchanges frames. The firmware
 * dispatches each request to the raw-USB relay (`components/raw_usb`).
 *
 * Milestone 0 (this scaffold): the dispatch table only answers HELLO and
 * GET_VERSION. Endpoint/USB primitives are stubbed to ERROR_NOT_IMPLEMENTED
 * pending `components/raw_usb` (Milestone 1).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Frame framing ──────────────────────────────────────────────────────── */

#define USB_RPC_MAGIC0 0xF0
#define USB_RPC_MAGIC1 0xD0
#define USB_RPC_PORT   4444
#define USB_RPC_PROTO_VERSION 0x0001
#define USB_RPC_MAX_PAYLOAD   512

/* ── Message IDs ────────────────────────────────────────────────────────── */
/* Host → device (requests): even */
#define USB_RPC_HELLO        0x01  /* u16 proto_version */
#define USB_RPC_GET_VERSION  0x02  /* (none) */
#define USB_RPC_CONNECT      0x10  /* u8 ep0_max, u8 speed, u8 quirks */
#define USB_RPC_DISCONNECT   0x11  /* (none) */
#define USB_RPC_RESET        0x18  /* (none) */
#define USB_RPC_SET_ADDRESS  0x17  /* u8 addr, u8 defer */
#define USB_RPC_CONFIGURE_EP 0x12  /* u8 addr, u8 dir, u8 type, u16 max_pkt */
#define USB_RPC_SEND         0x13  /* u8 ep_num, u16 data_len, data[] */
#define USB_RPC_STALL        0x14  /* u8 ep_num, u8 dir */
#define USB_RPC_CLEAR_HALT   0x15  /* u8 ep_num, u8 dir */
#define USB_RPC_ACK_STATUS   0x16  /* u8 dir, u8 ep_num */

/* Device → host (replies + events): odd */
#define USB_RPC_HELLO_REPLY   0x81  /* u16 proto_version, char[16] fw_version */
#define USB_RPC_VERSION_REPLY 0x82  /* char[] */
#define USB_RPC_OK            0x90  /* u8 in_reply_to */
#define USB_RPC_ERROR         0x91  /* u8 in_reply_to, u8 code, char[] */
#define USB_RPC_EVENT         0xA0  /* u8 event_type, ... */

#define USB_RPC_EVENT_BUS_RESET     0x01
#define USB_RPC_EVENT_SETUP         0x02  /* u8 ep, u8 setup[8] */
#define USB_RPC_EVENT_OUT_PACKET    0x03  /* u8 ep, u16 len, data[] */
#define USB_RPC_EVENT_SEND_COMPLETE 0x04  /* u8 ep */
#define USB_RPC_EVENT_NAK           0x05  /* u8 ep */

/* Error codes */
#define USB_RPC_ERR_NOT_IMPLEMENTED 0x01
#define USB_RPC_ERR_BAD_FRAME       0x02
#define USB_RPC_ERR_BAD_ARG         0x03
#define USB_RPC_ERR_USB_FAILED      0x04

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Start the control channel: SoftAP TCP server on USB_RPC_PORT.
 *
 * Spawns a FreeRTOS task that accepts one connection at a time, reads frames,
 * dispatches them, and writes replies. Returns immediately.
 *
 * Requires WiFi AP already up (call log_udp_init() first).
 */
esp_err_t usb_rpc_start(void);

/** True once a host has an open control-channel connection. */
bool usb_rpc_host_connected(void);

/**
 * @brief Send an unsolicited EVENT frame to the connected host.
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if no host is connected.
 */
esp_err_t usb_rpc_send_event(uint8_t event_type, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif