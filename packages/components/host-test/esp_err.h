/**
 * @file esp_err.h
 * @brief Minimal esp_err_t stub for the component host suites.
 *
 * Shared by every component test/Makefile that puts packages/components/host-test
 * on its include path (`just components::test`). Provides only the types,
 * constants and helper the components under test use, so the suites compile
 * without ESP-IDF installed.
 */
#pragma once

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102

static inline const char *esp_err_to_name(esp_err_t e)
{
    return e == ESP_OK ? "ESP_OK" : "ESP_FAIL";
}
