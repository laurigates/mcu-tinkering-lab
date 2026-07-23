/**
 * @file status_led.h
 * @brief Status LED driver for Waveshare ESP32-S3-Zero (WS2812 on GPIO21).
 *
 * Signals espdancer state using the on-board RGB LED:
 *   BOOT            — white blink       (booting / initializing)
 *   WIFI_AP_UP      — cyan solid         (SoftAP up, waiting for host PC)
 *   HOST_CONNECTED  — green blink        (host control channel connected)
 *   USB_ACTIVE      — blue solid         (USB enumerated, relay pump running)
 *   EMULATING       — green solid        (actively emulating a device)
 *   USB_ERROR       — red solid          (TinyUSB/DCD init failed)
 *
 * Caller-driven: call status_led_update() from an existing periodic loop
 * (~50+ Hz). No background task, no DMA (avoids the RMT/TinyUSB DMA conflict).
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    STATUS_LED_OFF,
    STATUS_LED_BOOT,           /**< White blink — booting */
    STATUS_LED_WIFI_AP_UP,     /**< Cyan solid — SoftAP up, waiting for host */
    STATUS_LED_HOST_CONNECTED, /**< Green blink — host control channel connected */
    STATUS_LED_USB_ACTIVE,     /**< Blue solid — USB enumerated, relay pump running */
    STATUS_LED_EMULATING,      /**< Green solid — actively emulating a device */
    STATUS_LED_USB_ERROR,      /**< Red solid — TinyUSB/DCD init failed */
} status_led_mode_t;

/**
 * @brief Initialize the WS2812 LED (RMT, non-DMA).
 */
esp_err_t status_led_init(void);

/**
 * @brief Set the LED mode. Thread-safe (volatile set).
 */
void status_led_set_mode(status_led_mode_t mode);

/**
 * @brief Update LED output. Call from an existing periodic loop (~50+ Hz).
 */
void status_led_update(void);

/**
 * @brief Blocking flash for boot-time visual feedback.
 *
 * Sets the LED to the given color, waits @p duration_ms, then turns it off.
 * The next status_led_update() call resumes normal mode-driven output.
 *
 * @param r           Red   (0–255).
 * @param g           Green (0–255).
 * @param b           Blue  (0–255).
 * @param duration_ms How long the flash stays on.
 */
void status_led_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);