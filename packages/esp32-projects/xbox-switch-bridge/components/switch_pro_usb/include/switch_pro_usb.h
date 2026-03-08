/**
 * @file switch_pro_usb.h
 * @brief Nintendo Switch Pro Controller USB HID emulation via TinyUSB.
 *
 * Emulates a wired Switch Pro Controller (VID 0x057E, PID 0x2009) using
 * the ESP32-S3 native USB peripheral. Handles the 0x80 handshake that the
 * Switch requires before accepting input reports.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Switch Pro Controller button bitmask (bytes 3-4 of standard input report) */

/* Byte 3 (right-side buttons) */
#define SW_BTN_Y (1 << 0)
#define SW_BTN_X (1 << 1)
#define SW_BTN_B (1 << 2)
#define SW_BTN_A (1 << 3)
#define SW_BTN_RSR (1 << 4) /* Right SR (not used on Pro Con) */
#define SW_BTN_RSL (1 << 5) /* Right SL (not used on Pro Con) */
#define SW_BTN_R (1 << 6)
#define SW_BTN_ZR (1 << 7)

/* Byte 4 (left-side buttons + shared) */
#define SW_BTN_MINUS (1 << 0)
#define SW_BTN_PLUS (1 << 1)
#define SW_BTN_RSTICK (1 << 2)
#define SW_BTN_LSTICK (1 << 3)
#define SW_BTN_HOME (1 << 4)
#define SW_BTN_CAPTURE (1 << 5)
/* Bits 6-7 unused */

/* Byte 5 (D-pad + triggers) */
#define SW_BTN_DPAD_DOWN (1 << 0)
#define SW_BTN_DPAD_UP (1 << 1)
#define SW_BTN_DPAD_RIGHT (1 << 2)
#define SW_BTN_DPAD_LEFT (1 << 3)
#define SW_BTN_LSR (1 << 4) /* Left SR (not used on Pro Con) */
#define SW_BTN_LSL (1 << 5) /* Left SL (not used on Pro Con) */
#define SW_BTN_L (1 << 6)
#define SW_BTN_ZL (1 << 7)

/**
 * @brief Switch Pro Controller input state.
 *
 * Stick values are 12-bit (0-4095), with 2048 as center.
 * Button fields are bitmasks using the SW_BTN_* defines above.
 */
typedef struct {
    uint8_t buttons_right;  /**< Y, X, B, A, R, ZR */
    uint8_t buttons_shared; /**< Minus, Plus, RStick, LStick, Home, Capture */
    uint8_t buttons_left;   /**< DPad, L, ZL */
    uint16_t lx;            /**< Left stick X (0-4095, center=2048) */
    uint16_t ly;            /**< Left stick Y (0-4095, center=2048) */
    uint16_t rx;            /**< Right stick X (0-4095, center=2048) */
    uint16_t ry;            /**< Right stick Y (0-4095, center=2048) */
} switch_pro_input_t;

/**
 * @brief Initialize the Switch Pro Controller USB HID device.
 *
 * Configures TinyUSB with the Pro Controller VID/PID and HID descriptor,
 * then installs the USB driver. Must be called before any other function.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t switch_pro_usb_init(void);

/**
 * @brief Send an input report to the Switch.
 *
 * Encodes the given input state into a standard 0x30 input report and
 * sends it over USB HID. Should be called at ~125 Hz (every 8ms).
 *
 * @param input Pointer to the current input state.
 * @return true if the report was sent successfully.
 */
bool switch_pro_usb_send_report(const switch_pro_input_t *input);

/**
 * @brief Check if the Switch has completed the USB handshake.
 *
 * @return true if the handshake is complete and the Switch is accepting
 *         input reports.
 */
bool switch_pro_usb_is_ready(void);

#ifdef __cplusplus
}
#endif
