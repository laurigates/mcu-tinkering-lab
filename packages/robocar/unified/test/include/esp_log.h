/**
 * @file esp_log.h — host-test shim.
 *
 * ESP-IDF's logging macros are no-ops in the host-test build. Keeping them
 * silent avoids polluting test output, and matches the way test_goal_state.c
 * stays quiet during its fuzz run.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H

/* The real esp_log.h pulls these in, and modules rely on it: self_report.c
 * calls snprintf() with no <stdio.h> of its own, and mqtt_logger.h declares a
 * va_list vprintf hook. */
#include <stdarg.h>
#include <stdio.h>

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE,
} esp_log_level_t;

#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H */
