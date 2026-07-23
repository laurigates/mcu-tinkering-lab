/**
 * @file raw_usb.h
 * @brief Minimal USB device stack that relays primitives between the
 *        `usb_rpc` control channel (host PC) and the ESP32-S3 DWC2 OTG-FS
 *        controller.
 *
 * Bypasses TinyUSB's `usbd` (which auto-handles USB standard requests) and
 * drives the DWC2 device controller driver directly, providing our own
 * `dcd_event_handler`. The host PC decides all device behavior — including
 * standard-request handling — see `docs/raw-usb-design.md` and ADR-0006.
 *
 * Single-writer discipline (ADR-0005): the DCD ISR only enqueues events; the
 * relay pump task is the sole writer to endpoint/DCD primitives.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hardware limits (Synopsys DWC2 OTG-FS on ESP32-S3) ─────────────────── */
/** Endpoints incl. EP0 → 6 non-control endpoints max (dwc2_esp32.h). */
#define RAW_USB_MAX_ENDPOINTS 7

/* ── USB speed codes (mirror facedancer DeviceSpeed + rpc-protocol.md) ──── */
#define RAW_USB_SPEED_LOW   1
#define RAW_USB_SPEED_FULL  2
#define RAW_USB_SPEED_HIGH  3

/* ── Endpoint type codes (mirror USBTransferType) ──────────────────────── */
#define RAW_USB_EP_CONTROL     0
#define RAW_USB_EP_ISOCHRONOUS 1
#define RAW_USB_EP_BULK        2
#define RAW_USB_EP_INTERRUPT   3

/* ── Direction ─────────────────────────────────────────────────────────── */
#define RAW_USB_DIR_OUT 0
#define RAW_USB_DIR_IN  1

/* ── Connect quirks (bit mask); mirror moondancer QuirkFlag ────────────── */
/* USB directions used internally by the DCD's ep_addr encoding. */
#define RAW_USB_ADDR_EP_MASK 0x0F
#define RAW_USB_ADDR_DIR_IN  0x80

/* ── Connect quirks (bit mask); mirror moondancer QuirkFlag ────────────── */
/** If set, the host drives SET_ADDRESS explicitly via raw_usb_set_address;
 *  the pump will not call dcd_set_address on an ACK'd SETUP. */
#define RAW_USB_QUIRK_MANUAL_SET_ADDRESS 0x01

/* ── Endpoint state (endpoint table, single-writer-owned by the pump) ─── */
typedef enum {
    RAW_USB_EP_UNUSED = 0,
    RAW_USB_EP_OPEN,
    RAW_USB_EP_STALLED,
} raw_usb_ep_state_t;

/* ── Runtime descriptor table (see docs/raw-usb-design.md) ─────────────── */
typedef struct {
    const uint8_t *device;        /**< 18-byte device descriptor */
    const uint8_t *config;        /**< configuration (9 + Σintf) descriptor  */
    const uint8_t *strings;       /**< packed string-set (LANGID + entries), or NULL */
    uint16_t       device_len;
    uint16_t       config_len;
    uint16_t       strings_len;
} raw_usb_descriptors_t;

/* ── Event types emitted to the host via usb_rpc_send_event() ──────────── */
/** Mirror usb_rpc.h USB_RPC_EVENT_* payload shapes (see docs/rpc-protocol.md). */
typedef enum {
    RAW_USB_EVENT_BUS_RESET = 0x01,
    RAW_USB_EVENT_SETUP     = 0x02,   /**< ep, setup[8] */
    RAW_USB_EVENT_OUT_PKT   = 0x03,   /**< ep, len, data[] */
    RAW_USB_EVENT_SEND_DONE = 0x04,   /**< ep */
    RAW_USB_EVENT_NAK       = 0x05,   /**< ep (suspected; M2/M4) */
} raw_usb_event_t;

/* Lifecycle */
esp_err_t raw_usb_init(void);   /**< Install USB ISR + empty event queue. */

esp_err_t raw_usb_connect(const raw_usb_descriptors_t *desc,
                          uint8_t ep0_max, uint8_t speed, uint8_t quirks);
esp_err_t raw_usb_disconnect(void);
esp_err_t raw_usb_reset(void);

/* Endpoint data plane (pump-task context only) */
esp_err_t raw_usb_configure_ep(uint8_t addr, uint8_t type, uint16_t max_pkt);
esp_err_t raw_usb_send(uint8_t ep, const uint8_t *data, uint16_t len);
esp_err_t raw_usb_stall(uint8_t ep, uint8_t dir);
esp_err_t raw_usb_clear_halt(uint8_t ep, uint8_t dir);
esp_err_t raw_usb_ack_status(uint8_t dir, uint8_t ep);
esp_err_t raw_usb_set_address(uint8_t addr, uint8_t defer);

/* Introspection (valid outside pump task — ro) */
bool            raw_usb_ready(void);                 /**< enumerated on a host? */
raw_usb_ep_state_t raw_usb_ep_state(uint8_t ep, uint8_t dir);

/* Relay pump task (started by raw_usb_init; sole writer to the DCD). */
void raw_usb_pump_task(void *arg);

#ifdef __cplusplus
}
#endif