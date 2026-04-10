/**
 * @file led_controller.h
 * @brief Hardware abstraction layer for PCA9685 RGB LED control
 *
 * Same public API as robocar-main. Internal implementation uses the i2c_bus
 * layer for thread-safe TCA9548A-routed PCA9685 access.
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum { LED_LEFT = 0, LED_RIGHT = 1, LED_BOTH = 2 } led_position_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

extern const rgb_color_t LED_COLOR_OFF;
extern const rgb_color_t LED_COLOR_RED;
extern const rgb_color_t LED_COLOR_GREEN;
extern const rgb_color_t LED_COLOR_BLUE;
extern const rgb_color_t LED_COLOR_WHITE;
extern const rgb_color_t LED_COLOR_YELLOW;
extern const rgb_color_t LED_COLOR_PURPLE;
extern const rgb_color_t LED_COLOR_CYAN;

esp_err_t led_controller_init(void);
esp_err_t led_set_color(led_position_t position, const rgb_color_t *color);
esp_err_t led_set_rgb(led_position_t position, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t led_set_left(const rgb_color_t *color);
esp_err_t led_set_right(const rgb_color_t *color);
esp_err_t led_set_both(const rgb_color_t *color);
esp_err_t led_turn_off(led_position_t position);
esp_err_t led_turn_off_all(void);
esp_err_t led_update(void);
esp_err_t led_get_color(led_position_t position, rgb_color_t *color);
esp_err_t led_blink(led_position_t position, const rgb_color_t *color,
                    uint32_t on_time_ms, uint32_t off_time_ms, uint32_t blink_count);
esp_err_t led_stop_blink(void);
bool led_is_initialized(void);

#endif  // LED_CONTROLLER_H
