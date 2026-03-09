/**
 * @file status_led.h
 * @brief Status LED driver for Waveshare ESP32-S3-Zero (WS2812 on GPIO21).
 *
 * Signals device state using the on-board RGB LED:
 *   BOOT           — blue fast blink (initializing)
 *   USB_READY      — solid cyan (USB mounted, waiting)
 *   BIOS_MODE      — yellow fast blink (hammering BIOS keys)
 *   WIFI_CONNECTING — blue slow pulse (connecting to hotspot)
 *   DIAGNOSTIC     — green slow blink (running diagnostics)
 *   ERROR          — solid red
 *   COMPLETE       — solid green
 */
#pragma once

#include "esp_err.h"

typedef enum {
    STATUS_LED_OFF,
    STATUS_LED_BOOT,
    STATUS_LED_USB_READY,
    STATUS_LED_BIOS_MODE,
    STATUS_LED_WIFI_CONNECTING,
    STATUS_LED_DIAGNOSTIC,
    STATUS_LED_ERROR,
    STATUS_LED_COMPLETE,
} status_led_mode_t;

/**
 * @brief Initialize the WS2812 LED (RMT, non-DMA).
 */
esp_err_t status_led_init(void);

/**
 * @brief Set the LED mode.
 *
 * Uses a volatile store, which is effectively atomic for a single-word enum
 * on Xtensa/RISC-V but does not provide C11 memory ordering guarantees.
 * Safe for the intended use case (one writer, one reader).
 */
void status_led_set_mode(status_led_mode_t mode);

/**
 * @brief Update LED output. Call from an existing periodic loop (~50+ Hz).
 */
void status_led_update(void);
