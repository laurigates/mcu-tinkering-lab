/**
 * @file esp_wifi.h — shim for test_mqtt_logger (issue #1054).
 *
 * Shadows test/include/esp_wifi.h for this one target: mqtt_logger.c reads the
 * AP's RSSI into each log record, on a task the host test never runs.
 * test_mqtt_logger.c implements esp_wifi_sta_get_ap_info().
 */

#ifndef ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_WIFI_H
#define ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_WIFI_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int8_t rssi;
} wifi_ap_record_t;

esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info);

#endif /* ROBOCAR_HOST_TEST_MQTT_LOGGER_ESP_WIFI_H */
