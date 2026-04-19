/**
 * @file button.h
 * @brief Debounced button driver on GPIO9 (active LOW, internal pull-up).
 *
 * Installs a GPIO ISR and fires user callbacks after a 50 ms software
 * debounce window.  Short-press callback fires on release; long-press
 * callback fires once when the button has been held for
 * BUTTON_LONG_PRESS_MS and inhibits the short-press callback for the
 * same press.  Callbacks run from a small helper task, not from
 * interrupt context, so they may call FreeRTOS APIs.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPIO connected to the momentary push-button (active LOW). */
#define BUTTON_GPIO 9

/** Software debounce window in milliseconds. */
#define BUTTON_DEBOUNCE_MS 50

/** Long-press threshold in milliseconds (2 s factory-reset hold). */
#define BUTTON_LONG_PRESS_MS 2000

/** Callback type fired once per confirmed short press or long press. */
typedef void (*button_callback_t)(void);

/**
 * @brief Initialise the button GPIO and ISR.
 *
 * Configures GPIO9 as input with internal pull-up, registers a
 * falling-edge ISR, and spawns the debounce task.
 *
 * @param on_short  Called on a confirmed short press (release before
 *                  BUTTON_LONG_PRESS_MS).  Must not be NULL.
 * @param on_long   Called once when the button crosses
 *                  BUTTON_LONG_PRESS_MS of continuous hold.  May be
 *                  NULL if no long-press action is required.
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t button_init(button_callback_t on_short, button_callback_t on_long);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
