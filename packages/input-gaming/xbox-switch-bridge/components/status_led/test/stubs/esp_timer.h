/**
 * @file esp_timer.h
 * @brief Stub for ESP-IDF high-resolution timer used in host tests.
 *
 * mock_timer_us is externally controlled by the test file so blink
 * timing can be driven deterministically.
 */
#pragma once

#include <stdint.h>

/** Controllable mock time source. Set before calling status_led_update(). */
extern int64_t mock_timer_us;

static inline int64_t esp_timer_get_time(void)
{
    return mock_timer_us;
}
