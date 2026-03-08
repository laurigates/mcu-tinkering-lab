/**
 * @file status_led.h
 * @brief Status LED driver for Waveshare ESP32-S3-Zero (WS2812 on GPIO21).
 *
 * Signals bridge state using the on-board RGB LED:
 *   SCANNING  — blue blink (500ms period)
 *   CONNECTED — solid yellow (Xbox paired, waiting for Switch)
 *   BRIDGING  — solid green  (actively forwarding inputs)
 */
#pragma once

#include "esp_err.h"

typedef enum {
    STATUS_LED_OFF,
    STATUS_LED_SCANNING,
    STATUS_LED_CONNECTED,
    STATUS_LED_BRIDGING,
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
