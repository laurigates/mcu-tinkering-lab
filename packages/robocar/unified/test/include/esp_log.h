/**
 * @file esp_log.h — host-test shim.
 *
 * ESP-IDF's logging macros print nothing in the host-test build. Keeping them
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

/* Print nothing, but still use the tag and every argument: a module's
 * `static const char *TAG`, or a local read only to be logged, would otherwise
 * be unused in the host build and need -Wno-unused-variable, which would also
 * hide real unused variables in the test code (#686). The arguments sit inside
 * sizeof, so they are type-checked against the format but never evaluated. */
#define ESP_LOG_HOST_DISCARD(tag, fmt, ...) ((void)(tag), (void)sizeof(printf(fmt, ##__VA_ARGS__)))

#define ESP_LOGE(tag, fmt, ...) ESP_LOG_HOST_DISCARD(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) ESP_LOG_HOST_DISCARD(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) ESP_LOG_HOST_DISCARD(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ESP_LOG_HOST_DISCARD(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) ESP_LOG_HOST_DISCARD(tag, fmt, ##__VA_ARGS__)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H */
