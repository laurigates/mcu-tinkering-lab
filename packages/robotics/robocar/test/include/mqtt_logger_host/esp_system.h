/**
 * @file esp_system.h — shim for test_mqtt_logger (issue #1054).
 *
 * mqtt_logger.c reads the free heap into each log record, on a task the host
 * test never runs. test_mqtt_logger.c implements it.
 */

#ifndef ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_SYSTEM_H
#define ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_SYSTEM_H

#include <stdint.h>

uint32_t esp_get_free_heap_size(void);

#endif /* ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_SYSTEM_H */
