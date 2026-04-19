/**
 * @file button.h
 * @brief GPIO button with interrupt-driven debounce.
 *
 * GPIO 9, internal pull-up, active LOW.
 * Short press: callback fires once. 2-second hold: hold callback fires.
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Callback type for button events. */
typedef void (*button_cb_t)(void);

/**
 * @brief Initialise button GPIO with interrupt and debounce.
 *
 * @param on_short_press  Called on a short press (< 2 s hold).
 * @param on_long_press   Called when button is held >= 2 s. May be NULL.
 * @return ESP_OK on success.
 */
esp_err_t button_init(button_cb_t on_short_press, button_cb_t on_long_press);

/**
 * @brief Service the button state machine.
 *
 * Must be called periodically (e.g., every 10 ms) to detect long-press
 * timeouts and emit debounced events.
 *
 * @param now_ms  Current time in milliseconds.
 */
void button_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
