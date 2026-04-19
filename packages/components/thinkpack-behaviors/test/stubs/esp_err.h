/**
 * @file esp_err.h
 * @brief Minimal esp_err_t stub for host-based unit tests.
 *
 * Provides only the types and constants used by command_executor.h so the
 * test suite compiles without ESP-IDF installed.
 */
#pragma once

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
