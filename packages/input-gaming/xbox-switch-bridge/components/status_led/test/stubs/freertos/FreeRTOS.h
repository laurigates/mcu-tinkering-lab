/**
 * @file FreeRTOS.h
 * @brief Minimal FreeRTOS stub for host-based unit tests.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t TickType_t;

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
