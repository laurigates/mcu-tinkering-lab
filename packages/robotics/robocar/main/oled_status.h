/**
 * @file oled_status.h
 * @brief Status screen on the 0.91-inch SSD1306 (TCA9548A channel 1).
 *
 * Four rows, refreshed by peripheral_task once a second: the IP address, which
 * I2C peripherals came up, the planner's state and spend, and both
 * rangefinders. The rows are built by oled_status_core.c; this file reads the
 * live accessors and pushes the result through oled.c, which writes only the
 * pages that changed.
 *
 * Optional like every peripheral behind the bus: with no panel fitted,
 * oled_status_init() returns ESP_ERR_NOT_FOUND and oled_status_refresh() does
 * nothing.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Probe and initialise the panel, then show a boot splash.
 *
 * @return ESP_OK with a panel; ESP_ERR_NOT_FOUND when nothing answers at
 *         OLED_I2C_ADDR; ESP_ERR_INVALID_STATE when the I2C bus is not up.
 */
esp_err_t oled_status_init(void);

/** Read the current state and redraw. A no-op when the panel is absent. */
void oled_status_refresh(void);

#ifdef __cplusplus
}
#endif
