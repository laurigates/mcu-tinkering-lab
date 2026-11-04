/**
 * @file esp_random.h — host-test shim. Deterministic; defined by the test.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_RANDOM_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_RANDOM_H

#include <stdint.h>

uint32_t esp_random(void);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_RANDOM_H */
