/**
 * @file status_led.c
 * @brief espdancer's status LED modes, on the shared WS2812 layer.
 *
 * The RMT driver (non-DMA, the RMT+TinyUSB DMA conflict documented in
 * xbox-switch-bridge ADR-0008), blink timing and flash live in
 * packages/components/status-led (status_led_core.c, compiled into this
 * component); this file holds only the mode table.
 */
#include "status_led.h"

#include "status_led_core.h"

#define LED_GPIO 21
#define LED_BRIGHTNESS 32
#define BLINK_SLOW_US (500 * 1000) /* 500ms per phase (1 Hz) */
#define BLINK_FAST_US (200 * 1000) /* 200ms per phase (2.5 Hz) */

static const status_led_pattern_t k_patterns[] = {
    [STATUS_LED_OFF] = {0, 0, 0, 0},
    [STATUS_LED_BOOT] = {LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS, BLINK_FAST_US}, /* White */
    [STATUS_LED_WIFI_AP_UP] = {0, LED_BRIGHTNESS, LED_BRIGHTNESS, 0},                    /* Cyan */
    [STATUS_LED_HOST_CONNECTED] = {0, LED_BRIGHTNESS, 0, BLINK_SLOW_US},                 /* Green */
    [STATUS_LED_USB_ACTIVE] = {0, 0, LED_BRIGHTNESS, 0},                                 /* Blue */
    [STATUS_LED_EMULATING] = {0, LED_BRIGHTNESS, 0, 0},                                  /* Green */
    [STATUS_LED_USB_ERROR] = {LED_BRIGHTNESS, 0, 0, 0},                                  /* Red */
};

esp_err_t status_led_init(void)
{
    return status_led_core_init(LED_GPIO, k_patterns, sizeof(k_patterns) / sizeof(k_patterns[0]));
}

void status_led_set_mode(status_led_mode_t mode)
{
    status_led_core_set_mode((unsigned)mode);
}

void status_led_update(void)
{
    status_led_core_update();
}

void status_led_flash(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    status_led_core_flash(r, g, b, duration_ms);
}
