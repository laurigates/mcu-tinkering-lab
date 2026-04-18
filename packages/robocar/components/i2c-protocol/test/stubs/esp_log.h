/**
 * @file esp_log.h
 * @brief Stub for ESP-IDF logging — prints to stderr/stdout for host tests.
 */
#pragma once

#include <stdio.h>

/* cppcheck-suppress [missingIncludeSystem] */
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[E][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[W][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stdout, "[I][%s] " fmt "\n", (tag), ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) /* debug suppressed in host tests */
#define ESP_LOGV(tag, fmt, ...) /* verbose suppressed in host tests */
