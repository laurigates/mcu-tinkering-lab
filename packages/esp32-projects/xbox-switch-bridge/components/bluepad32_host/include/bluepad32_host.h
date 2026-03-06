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
 * @brief Initialize Bluepad32 as a BLE host.
 *
 * Configures BTstack and registers the Bluepad32 custom platform.
 * Must be followed by bp32_host_start() to begin the event loop.
 *
 * @param conn_cb Optional callback for connection state changes.
 * @return ESP_OK on success.
 */
esp_err_t bp32_host_init(bp32_connection_cb_t conn_cb);

/**
 * @brief Start the BTstack event loop.
 *
 * This call does NOT return. It must be called from the task that will
 * run the Bluetooth stack (typically app_main). Create any other tasks
 * before calling this function.
 */
void bp32_host_start(void);

/**
 * @brief Process Bluepad32 events (no-op in v4.x).
 *
 * In Bluepad32 v4.x, the event loop runs via btstack_run_loop_execute(),
 * so no explicit polling is needed. Kept for API compatibility.
 */
void bp32_host_process(void);

/**
 * @brief Get the current gamepad state.
 *
 * @param state Output: current gamepad state.
 * @return true if a controller is connected and state is valid.
 */
bool bp32_host_get_state(xbox_gamepad_state_t *state);

#ifdef __cplusplus
}
#endif
