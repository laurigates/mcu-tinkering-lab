/**
 * @file led_strip.h
 * @brief Stub for espressif/led_strip component used in host tests.
 *
 * Mock implementations record the last colour written and the number of
 * refresh calls so tests can verify LED behaviour without hardware.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef void *led_strip_handle_t;

typedef enum {
    LED_MODEL_WS2812 = 0,
} led_strip_led_model_t;

typedef enum {
    LED_STRIP_COLOR_COMPONENT_FMT_GRB = 0,
} led_strip_color_component_format_t;

typedef struct {
    int                                 strip_gpio_num;
    int                                 max_leds;
    led_strip_led_model_t               led_model;
    led_strip_color_component_format_t  color_component_format;
} led_strip_config_t;

typedef struct {
    int resolution_hz;
    int mem_block_symbols;
    struct {
        int with_dma;
    } flags;
} led_strip_rmt_config_t;

/* ---------------------------------------------------------------------- */
/* Mock state — written by mock functions, read by tests                   */
/* ---------------------------------------------------------------------- */
extern uint8_t mock_led_r;
extern uint8_t mock_led_g;
extern uint8_t mock_led_b;
extern int     mock_set_pixel_calls;
extern int     mock_refresh_calls;

/* ---------------------------------------------------------------------- */
/* Mock function declarations                                              */
/* ---------------------------------------------------------------------- */
esp_err_t led_strip_new_rmt_device(const led_strip_config_t     *strip_config,
                                   const led_strip_rmt_config_t *rmt_config,
                                   led_strip_handle_t           *ret_strip);
esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index,
                              uint8_t red, uint8_t green, uint8_t blue);
esp_err_t led_strip_refresh(led_strip_handle_t strip);
esp_err_t led_strip_clear(led_strip_handle_t strip);
