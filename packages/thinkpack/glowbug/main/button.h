/**
 * @file button.h
 * @brief Debounced button driver on GPIO9 (active LOW, internal pull-up).
 *
 * Installs a GPIO ISR and fires a user callback after a 50 ms software
 * debounce window.  The callback runs from a small helper task, not from
 * interrupt context, so it may call FreeRTOS APIs.
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

/** Callback type fired once per confirmed button press. */
typedef void (*button_callback_t)(void);

/**
 * @brief Initialise the button GPIO and ISR.
 *
 * Configures GPIO9 as input with internal pull-up, registers a
 * falling-edge ISR, and spawns the debounce task.
 *
 * @param cb  Function to call on each confirmed press.  Must not be NULL.
 * @return ESP_OK on success, forwarded ESP-IDF error otherwise.
 */
esp_err_t button_init(button_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
