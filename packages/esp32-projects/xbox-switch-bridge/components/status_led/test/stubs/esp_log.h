/**
 * @file esp_log.h
 * @brief Stub for ESP-IDF logging used in host tests.
 */
#pragma once

#include <stdio.h>

#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[E][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[W][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stdout, "[I][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) /* debug suppressed */
#define ESP_LOGV(tag, fmt, ...) /* verbose suppressed */
