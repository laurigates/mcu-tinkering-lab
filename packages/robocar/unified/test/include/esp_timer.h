/**
 * @file esp_timer.h — host-test shim.
 *
 * Microseconds since boot. The self_report host test never starts the monitor
 * task, so the stub returns a constant; a test that exercised the announcement
 * rate limit would need an injectable clock instead.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_TIMER_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_TIMER_H

#include <stdint.h>

int64_t esp_timer_get_time(void);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_TIMER_H */
