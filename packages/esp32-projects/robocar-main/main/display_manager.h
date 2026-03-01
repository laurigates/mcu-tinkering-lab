/**
 * @file display_manager.h
 * @brief Hardware abstraction layer for OLED display and action tracking
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define DISPLAY_MAX_LINE_LENGTH 16
#define DISPLAY_MAX_LINES 8

/**
 * @brief Robot state for display
 */
typedef enum {
    DISPLAY_STATE_STOPPED = 0,
    DISPLAY_STATE_FORWARD,
    DISPLAY_STATE_BACKWARD,
    DISPLAY_STATE_LEFT,
    DISPLAY_STATE_RIGHT,
    DISPLAY_STATE_ROTATE_CW,
    DISPLAY_STATE_ROTATE_CCW,
    DISPLAY_STATE_UNKNOWN
} display_robot_state_t;

/**
 * @brief Action information for tracking
 */
typedef struct {
    char action[32];
    char source[16];
    unsigned long counter;
    unsigned long timestamp;
} display_action_t;

/**
 * @brief Initialize display manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_manager_init(void);

/**
 * @brief Show startup screen
 * @return ESP_OK on success
 */
esp_err_t display_show_startup(void);

/**
 * @brief Update display with current robot state
 * @param state Current robot state
 * @param pan_angle Current pan servo angle
 * @param tilt_angle Current tilt servo angle
 * @param last_command_time Timestamp of last command
 * @return ESP_OK on success
 */
esp_err_t display_update_status(display_robot_state_t state, int pan_angle, int tilt_angle,
                                unsigned long last_command_time);

/**
 * @brief Show action debug information
 * @param action Action information to display
 * @param state Current robot state
 * @param pan_angle Current pan servo angle
 * @param tilt_angle Current tilt servo angle
 * @param last_command_time Timestamp of last command
 * @return ESP_OK on success
 */
esp_err_t display_show_action_debug(const display_action_t *action, display_robot_state_t state,
                                    int pan_angle, int tilt_angle, unsigned long last_command_time);

/**
 * @brief Show message on specific line
 * @param line Display line (0-7)
 * @param message Message to display
 * @return ESP_OK on success
 */
esp_err_t display_show_message(int line, const char *message);

/**
 * @brief Track an action and update display
 * @param action Action description
 * @param source Action source
 * @return ESP_OK on success
 */
esp_err_t display_track_action(const char *action, const char *source);

/**
 * @brief Get current action counter
 * @return Current action counter value
 */
unsigned long display_get_action_counter(void);

/**
 * @brief Clear display
 * @return ESP_OK on success
 */
esp_err_t display_clear(void);

/**
 * @brief Check if display is initialized
 * @return true if initialized, false otherwise
 */
bool display_is_initialized(void);

#endif  // DISPLAY_MANAGER_H
