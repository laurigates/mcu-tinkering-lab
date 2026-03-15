/**
 * @file task.h
 * @brief FreeRTOS task stub for host-based unit tests.
 *
 * vTaskDelay is a no-op so status_led_flash() returns immediately in tests.
 */
#pragma once

#include "FreeRTOS.h"

static inline void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}
