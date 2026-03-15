/**
 * @file esp_err.h
 * @brief Stub for ESP-IDF esp_err types used in host tests.
 */
#pragma once

typedef int esp_err_t;

#define ESP_OK   0
#define ESP_FAIL (-1)

static inline const char *esp_err_to_name(esp_err_t e)
{
    return e == ESP_OK ? "ESP_OK" : "ESP_FAIL";
}
