/**
 * @file touch_driver.h
 * @brief ESP32-S3 native capacitive touch wrapper for Chatterbox.
 *
 * Wraps the touch_pad driver in a small callback-based API.  Emits edge
 * events (press / release) that feed a thinkpack_audio_sm_t via the mode
 * modules.  Hardware-unverified.
 */

#ifndef TOUCH_DRIVER_H
#define TOUCH_DRIVER_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*touch_driver_cb_t)(bool pressed);

/**
 * @brief Initialise the touch peripheral and start the polling task.
 *
 * @param cb  Callback invoked on every edge (pressed=true / pressed=false).
 *            Called from the polling task — must not block.
 */
esp_err_t touch_driver_init(touch_driver_cb_t cb);

/** Return the last cached pressed state (for diagnostics). */
bool touch_driver_is_pressed(void);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_DRIVER_H */
