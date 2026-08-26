/**
 * @file oled.h
 * @brief Minimal SSD1306 text renderer for the bringup sweep.
 *
 * The OLED is declared in robocar-unified's pin_config.h and i2c_bus.h — the
 * multiplexer channel and the 0x3C address are both reserved — but no driver in
 * that firmware has ever driven it. This is the first code that does.
 *
 * It exists because it is the only indicator on the board that can show WHICH
 * check failed without a tether. The buzzer says a check failed; the LEDs say
 * something failed; the screen says `ULTRASONIC FAIL`. With a soldering iron in
 * one hand that difference is the whole point.
 *
 * Deliberately not an esp_lcd panel driver. esp_lcd owns its own I2C
 * transactions, and this display sits BEHIND the TCA9548A on channel 1 — the
 * mux channel has to be selected, and the i2cdev bus mutex held, across every
 * transfer. Going through i2cdev like every other device on this board keeps
 * that ordering in one place instead of racing a second I2C master abstraction.
 *
 * Text only, uppercase, 5x7 glyphs in a 6x8 cell: 21 columns by 8 rows. A
 * proportional font or a graphics layer would be a nicer screen and a worse
 * bringup tool — the failure mode to avoid is a rendering bug that reads as a
 * hardware fault.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_COLS 21
#define OLED_ROWS 8

/**
 * @brief Probe for the panel and, if present, initialise and blank it.
 *
 * Non-fatal by design, like every other optional device in this sweep: a board
 * with no display fitted is the normal state for most of a build.
 *
 * @return ESP_OK when a panel answered and was initialised;
 *         ESP_ERR_NOT_FOUND when nothing ACKed at OLED_I2C_ADDR on channel
 *         I2C_BUS_CHANNEL_OLED; ESP_ERR_INVALID_STATE if the I2C bus itself is
 *         not up; otherwise the underlying i2cdev error.
 */
esp_err_t oled_init(void);

/** True once oled_init() has succeeded. Every call below is a no-op until then,
 *  so callers do not need to guard each one. */
bool oled_available(void);

/** Clear the framebuffer. Does not touch the panel until oled_flush(). */
void oled_clear(void);

/**
 * @brief Draw @p text at cell (@p col, @p row), clipped to the panel.
 *
 * Lowercase is folded to uppercase and unrepresentable characters render as
 * `?` — the font carries only A-Z, 0-9 and a handful of punctuation. Rendering
 * into the framebuffer only; call oled_flush() to make it visible.
 */
void oled_text(uint8_t col, uint8_t row, const char *text);

/** Push the framebuffer to the panel. */
esp_err_t oled_flush(void);

#ifdef __cplusplus
}
#endif
