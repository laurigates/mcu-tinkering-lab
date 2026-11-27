/**
 * @file status_led.c
 * @brief The troubleshooter's status LED modes, on the shared WS2812 layer.
 *
 * The RMT driver (non-DMA, which conflicts with TinyUSB on the ESP32-S3),
 * blink timing and flash live in packages/components/status-led
 * (status_led_core.c, compiled into this component); this file holds only
 * the mode table.
 */
#include "status_led.h"

#include "status_led_core.h"

#define LED_GPIO 21
#define LED_BRIGHTNESS 32
#define FAST_BLINK_US (250 * 1000)
#define SLOW_BLINK_US (1000 * 1000)

static const status_led_pattern_t k_patterns[] = {
    [STATUS_LED_OFF] = {0, 0, 0, 0},
    [STATUS_LED_BOOT] = {0, 0, LED_BRIGHTNESS, FAST_BLINK_US},
    [STATUS_LED_USB_READY] = {0, LED_BRIGHTNESS, LED_BRIGHTNESS, 0},
    [STATUS_LED_BIOS_MODE] = {LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0, FAST_BLINK_US},
    [STATUS_LED_WIFI_CONNECTING] = {0, 0, LED_BRIGHTNESS, SLOW_BLINK_US},
    [STATUS_LED_DIAGNOSTIC] = {0, LED_BRIGHTNESS, 0, SLOW_BLINK_US},
    [STATUS_LED_ERROR] = {LED_BRIGHTNESS, 0, 0, 0},
    [STATUS_LED_COMPLETE] = {0, LED_BRIGHTNESS, 0, 0},
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
