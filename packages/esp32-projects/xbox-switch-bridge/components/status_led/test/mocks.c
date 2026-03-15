/**
 * @file mocks.c
 * @brief Mock implementations for led_strip hardware driver.
 *
 * Provides stub functions that record their arguments so tests can
 * verify the LED colour and refresh behaviour without hardware.
 */

#include "led_strip.h"

#include <stddef.h>

/* ---------------------------------------------------------------------- */
/* Mock state — defined here, declared extern in led_strip.h              */
/* ---------------------------------------------------------------------- */
uint8_t mock_led_r         = 0;
uint8_t mock_led_g         = 0;
uint8_t mock_led_b         = 0;
int     mock_set_pixel_calls = 0;
int     mock_refresh_calls   = 0;

/** Controllable time for esp_timer_get_time() stub (declared in esp_timer.h). */
int64_t mock_timer_us = 0;

/* ---------------------------------------------------------------------- */
/* Mock implementations                                                    */
/* ---------------------------------------------------------------------- */

static int dummy_handle;

esp_err_t led_strip_new_rmt_device(const led_strip_config_t     *strip_config,
                                   const led_strip_rmt_config_t *rmt_config,
                                   led_strip_handle_t           *ret_strip)
{
    (void)strip_config;
    (void)rmt_config;
    *ret_strip = &dummy_handle;
    return ESP_OK;
}

esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index,
                              uint8_t red, uint8_t green, uint8_t blue)
{
    (void)strip;
    (void)index;
    mock_led_r = red;
    mock_led_g = green;
    mock_led_b = blue;
    mock_set_pixel_calls++;
    return ESP_OK;
}

esp_err_t led_strip_refresh(led_strip_handle_t strip)
{
    (void)strip;
    mock_refresh_calls++;
    return ESP_OK;
}

esp_err_t led_strip_clear(led_strip_handle_t strip)
{
    (void)strip;
    mock_led_r = 0;
    mock_led_g = 0;
    mock_led_b = 0;
    return ESP_OK;
}
