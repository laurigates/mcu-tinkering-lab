/**
 * @file led_controller.h
 * @brief Hardware abstraction layer for PCA9685 RGB LED control
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief LED position identifier
 */
typedef enum { LED_LEFT = 0, LED_RIGHT = 1, LED_BOTH = 2 } led_position_t;

/**
 * @brief RGB color structure
 */
typedef struct {
    uint8_t red;    // 0-255
    uint8_t green;  // 0-255
    uint8_t blue;   // 0-255
} rgb_color_t;

/**
 * @brief Predefined colors
 */
extern const rgb_color_t LED_COLOR_OFF;
extern const rgb_color_t LED_COLOR_RED;
extern const rgb_color_t LED_COLOR_GREEN;
extern const rgb_color_t LED_COLOR_BLUE;
extern const rgb_color_t LED_COLOR_WHITE;
extern const rgb_color_t LED_COLOR_YELLOW;
extern const rgb_color_t LED_COLOR_PURPLE;
extern const rgb_color_t LED_COLOR_CYAN;

/**
 * @brief Initialize LED controller
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_controller_init(void);

/**
 * @brief Set RGB color for specific LED position
 * @param position LED position (LEFT, RIGHT, or BOTH)
 * @param color RGB color to set
 * @return ESP_OK on success
 */
esp_err_t led_set_color(led_position_t position, const rgb_color_t *color);

/**
 * @brief Set RGB values directly for specific LED position
 * @param position LED position (LEFT, RIGHT, or BOTH)
 * @param red Red intensity (0-255)
 * @param green Green intensity (0-255)
 * @param blue Blue intensity (0-255)
 * @return ESP_OK on success
 */
esp_err_t led_set_rgb(led_position_t position, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set left LED color
 * @param color RGB color to set
 * @return ESP_OK on success
 */
esp_err_t led_set_left(const rgb_color_t *color);

/**
 * @brief Set right LED color
 * @param color RGB color to set
 * @return ESP_OK on success
 */
esp_err_t led_set_right(const rgb_color_t *color);

/**
 * @brief Set both LEDs to same color
 * @param color RGB color to set
 * @return ESP_OK on success
 */
esp_err_t led_set_both(const rgb_color_t *color);

/**
 * @brief Turn off specific LED(s)
 * @param position LED position (LEFT, RIGHT, or BOTH)
 * @return ESP_OK on success
 */
esp_err_t led_turn_off(led_position_t position);

/**
 * @brief Turn off all LEDs
 * @return ESP_OK on success
 */
esp_err_t led_turn_off_all(void);

/**
 * @brief Update LED hardware (commit pending changes)
 * @return ESP_OK on success
 */
esp_err_t led_update(void);

/**
 * @brief Get current color of specified LED
 * @param position LED position (LEFT or RIGHT only)
 * @param color Pointer to store current color
 * @return ESP_OK on success
 */
esp_err_t led_get_color(led_position_t position, rgb_color_t *color);

/**
 * @brief Blink LED(s) with specified color and timing
 * @param position LED position (LEFT, RIGHT, or BOTH)
 * @param color RGB color to blink
 * @param on_time_ms Time to stay on (milliseconds)
 * @param off_time_ms Time to stay off (milliseconds)
 * @param blink_count Number of blinks (0 = infinite)
 * @return ESP_OK on success
 */
esp_err_t led_blink(led_position_t position, const rgb_color_t *color, uint32_t on_time_ms,
                    uint32_t off_time_ms, uint32_t blink_count);

/**
 * @brief Stop any ongoing blink operation
 * @return ESP_OK on success
 */
esp_err_t led_stop_blink(void);

/**
 * @brief Check if LED controller is initialized
 * @return true if initialized, false otherwise
 */
bool led_is_initialized(void);

#endif  // LED_CONTROLLER_H
