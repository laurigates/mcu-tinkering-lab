/**
 * @file nvs.h — host-test shim.
 *
 * The slice of ESP-IDF's NVS API that credentials_loader.c uses, backed by an
 * in-memory store in test/nvs_shim.c. The error codes, the open modes and the
 * return-code contract are copied from ESP-IDF v5.4's
 * components/nvs_flash/include/nvs.h, because the loader branches on them:
 *
 *  - nvs_open(NVS_READONLY) on a namespace that was never written returns
 *    ESP_ERR_NVS_NOT_FOUND; NVS_READWRITE creates it.
 *  - nvs_get_str() on a missing key returns ESP_ERR_NVS_NOT_FOUND, and on a
 *    buffer too small for the stored string ESP_ERR_NVS_INVALID_LENGTH.
 *  - writes through a read-only handle return ESP_ERR_NVS_READ_ONLY.
 *
 * test/nvs_shim.h has the seeding/inspection API the tests drive it with.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_NVS_H
#define ROBOCAR_UNIFIED_HOST_TEST_NVS_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef uint32_t nvs_handle_t;

#define ESP_ERR_NVS_BASE 0x1100
#define ESP_ERR_NVS_NOT_FOUND (ESP_ERR_NVS_BASE + 0x02)
#define ESP_ERR_NVS_READ_ONLY (ESP_ERR_NVS_BASE + 0x04)
#define ESP_ERR_NVS_INVALID_HANDLE (ESP_ERR_NVS_BASE + 0x07)
#define ESP_ERR_NVS_INVALID_LENGTH (ESP_ERR_NVS_BASE + 0x0c)

typedef enum {
    NVS_READONLY,
    NVS_READWRITE,
} nvs_open_mode_t;

esp_err_t nvs_open(const char *namespace_name, nvs_open_mode_t open_mode, nvs_handle_t *out_handle);
esp_err_t nvs_get_str(nvs_handle_t handle, const char *key, char *out_value, size_t *length);
esp_err_t nvs_set_str(nvs_handle_t handle, const char *key, const char *value);
esp_err_t nvs_erase_all(nvs_handle_t handle);
esp_err_t nvs_commit(nvs_handle_t handle);
void nvs_close(nvs_handle_t handle);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_NVS_H */
