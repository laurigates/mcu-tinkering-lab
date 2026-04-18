/**
 * @file usb_composite.h
 * @brief USB composite device: HID boot-protocol keyboard + CDC-ACM serial.
 *
 * Uses TinyUSB on ESP32-S3 USB OTG peripheral. The HID keyboard uses boot
 * protocol (subclass 1, protocol 1) for BIOS compatibility.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize TinyUSB composite device (HID keyboard + CDC serial).
 */
esp_err_t usb_composite_init(void);

/**
 * @brief Check if a USB host has enumerated this device.
 */
bool usb_composite_is_mounted(void);

/* --- HID Keyboard API --- */

/**
 * @brief Press and release a single key.
 * @param keycode USB HID keycode (e.g., HID_KEY_A = 0x04)
 */
esp_err_t usb_keyboard_press(uint8_t keycode);

/**
 * @brief Press and release a key with modifier(s).
 * @param modifier Modifier bitmap (e.g., 0x02 = Left Shift)
 * @param keycode USB HID keycode
 */
esp_err_t usb_keyboard_press_with_modifier(uint8_t modifier, uint8_t keycode);

/**
 * @brief Type an ASCII string character by character.
 * @param str Null-terminated ASCII string
 */
esp_err_t usb_keyboard_type_string(const char *str);

/**
 * @brief Send a raw 8-byte boot keyboard report.
 * @param modifier Modifier bitmap
 * @param keycodes Array of up to 6 simultaneous keycodes
 */
esp_err_t usb_keyboard_send_report(uint8_t modifier, const uint8_t keycodes[6]);

/* --- CDC Serial API --- */

/**
 * @brief Write data to the CDC serial port.
 * @param data Buffer to send
 * @param len Number of bytes
 * @return Number of bytes written, or -1 on error
 */
int usb_cdc_write(const uint8_t *data, size_t len);

/**
 * @brief Read data from the CDC serial port (non-blocking).
 * @param buf Buffer to read into
 * @param buf_size Maximum bytes to read
 * @return Number of bytes read, or 0 if no data available
 */
int usb_cdc_read(uint8_t *buf, size_t buf_size);
