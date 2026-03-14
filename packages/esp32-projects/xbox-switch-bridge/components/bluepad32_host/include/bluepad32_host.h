/**
 * @file bluepad32_host.h
 * @brief Bluepad32 BLE gamepad host wrapper for Xbox controller input.
 *
 * Wraps the Bluepad32 library to scan for and connect to Xbox Series
 * controllers over Bluetooth Low Energy, providing a simple polling API.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Raw gamepad state from an Xbox controller.
 *
 * Axes range: -511 to 512 (Bluepad32 default).
 * Triggers range: 0 to 1023.
 */
typedef struct {
    /* Buttons (bitmask, using Bluepad32 BUTTON_* constants) */
    uint16_t buttons;

    /* D-pad (bitmask, using Bluepad32 DPAD_* constants) */
    uint8_t dpad;

    /* Misc buttons (Home, Share, etc.) */
    uint8_t misc_buttons;

    /* Left stick */
    int16_t axis_x;
    int16_t axis_y;

    /* Right stick */
    int16_t axis_rx;
    int16_t axis_ry;

    /* Triggers */
    int16_t brake;    /**< Left trigger (0-1023) */
    int16_t throttle; /**< Right trigger (0-1023) */

    /* Metadata */
    bool connected;
} xbox_gamepad_state_t;

/**
 * @brief Callback invoked when a controller connects or disconnects.
 */
typedef void (*bp32_connection_cb_t)(bool connected);

/**
 * @brief Register connection callback and initialize state.
 *
 * Must be called before bp32_host_start().
 *
 * @param conn_cb Optional callback for connection state changes.
 * @return ESP_OK on success.
 */
esp_err_t bp32_host_init(bp32_connection_cb_t conn_cb);

/**
 * @brief Start Bluepad32 and the BTstack event loop.
 *
 * This call does NOT return. It must be called from the task that will
 * run the Bluetooth stack (typically app_main). Create any other tasks
 * before calling this function.
 */
void bp32_host_start(void);

/**
 * @brief Get the current gamepad state.
 *
 * @param state Output: current gamepad state.
 * @return true if a controller is connected and state is valid.
 */
bool bp32_host_get_state(xbox_gamepad_state_t *state);

/**
 * @brief Send rumble/vibration to the connected Xbox controller.
 *
 * Schedules a play_dual_rumble call on the BTstack main thread (core 0).
 * Safe to call from any thread. Values are 0-255 intensity.
 *
 * @param weak Weak motor (high-frequency) intensity (0-255).
 * @param strong Strong motor (low-frequency) intensity (0-255).
 */
void bp32_host_set_rumble(uint8_t weak, uint8_t strong);

#ifdef __cplusplus
}
#endif
