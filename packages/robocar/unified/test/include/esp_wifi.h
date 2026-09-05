/**
 * @file esp_wifi.h — host-test shim.
 *
 * wifi_manager.h includes this for nothing but the bool it then uses in
 * wifi_is_connected()'s declaration. The host tests only ever call that one
 * accessor, so the shim carries the type and nothing else.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_WIFI_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_WIFI_H

#include <stdbool.h>

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_WIFI_H */
