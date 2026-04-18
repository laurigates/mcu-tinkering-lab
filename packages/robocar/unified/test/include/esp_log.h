/**
 * @file esp_log.h — host-test shim.
 *
 * ESP-IDF's logging macros are no-ops in the host-test build. Keeping them
 * silent avoids polluting test output, and matches the way test_goal_state.c
 * stays quiet during its fuzz run.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H

#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_LOG_H */
