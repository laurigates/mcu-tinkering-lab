/**
 * @file status_led.h
 * @brief Status LED driver for Waveshare ESP32-S3-Zero (WS2812 on GPIO21).
 *
 * Signals bridge state using the on-board RGB LED:
 *   SCANNING         — blue blink       (waiting for Xbox controller)
 *   USB_ERROR        — red solid         (TinyUSB init failed)
 *   CONNECTED_NO_USB — purple blink      (Xbox paired, USB not enumerated)
 *   CONNECTED_USB    — yellow blink      (Xbox paired, USB enumerated, handshake pending)
 *   BRIDGING         — green solid       (actively forwarding inputs)
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    STATUS_LED_OFF,
    STATUS_LED_SCANNING,         /**< Blue blink — waiting for Xbox controller */
    STATUS_LED_USB_ERROR,        /**< Red solid — TinyUSB driver install failed */
    STATUS_LED_CONNECTED_NO_USB, /**< Purple blink — Xbox paired, USB not enumerated by Switch */
    STATUS_LED_CONNECTED_USB,    /**< Yellow blink — USB enumerated, waiting for handshake */
    STATUS_LED_BRIDGING,         /**< Green solid — actively forwarding inputs */
} status_led_mode_t;

/**
 * @brief Initialize the WS2812 LED (RMT, non-DMA).
 */
esp_err_t status_led_init(void);

/**
 * @brief Set the LED mode. Thread-safe.
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
 * Does not affect the current mode — the next status_led_update() call
 * will resume normal mode-driven output.
 *
 * @param r          Red   (0–255).
 * @param g          Green (0–255).
 * @param b          Blue  (0–255).
 * @param duration_ms  How long the flash stays on.
 */
void status_led_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);
