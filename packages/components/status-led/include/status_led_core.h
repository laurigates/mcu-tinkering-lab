/**
 * @file status_led_core.h
 * @brief Shared WS2812 status LED layer: RMT init, blink timing, flash.
 *
 * Each project keeps its own status_led component with its mode enum and a
 * table of patterns indexed by that enum; this layer drives the LED from the
 * table. No background task: call status_led_core_update() from an existing
 * loop (~50+ Hz). The RMT channel runs without DMA, which breaks after
 * TinyUSB init on the ESP32-S3.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** One mode's output. */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint32_t blink_us; /**< 0 = solid; otherwise the length of each on and off phase */
} status_led_pattern_t;

/**
 * @brief Initialize a single WS2812 on @p gpio (RMT, non-DMA).
 *
 * The mode is not reset: it starts at 0, and a status_led_core_set_mode()
 * made before init is kept.
 *
 * @param patterns Table indexed by mode; must outlive the driver.
 * @param count    Number of entries. A mode at or past it shows dark.
 */
esp_err_t status_led_core_init(int gpio, const status_led_pattern_t *patterns, size_t count);

/** @brief Select the table entry to show. A volatile store: one writer, one reader. */
void status_led_core_set_mode(unsigned mode);

/** @brief Write the current mode's colour, toggling a blink when its phase has elapsed. */
void status_led_core_update(void);

/**
 * @brief Blocking flash: show the colour for @p duration_ms, then turn the LED off.
 *
 * The mode is unchanged; the next status_led_core_update() resumes it.
 */
void status_led_core_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);
