/**
 * @file nvs_shim.h — test-side control of the in-memory NVS in nvs_shim.c.
 *
 * The loader under test only sees include/nvs.h. These calls let a test put
 * the store into a state a board could be in (a half-written pair, a value
 * longer than the loader's buffer, an NVS partition that will not open) and
 * then read back what the loader left there.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_NVS_SHIM_H
#define ROBOCAR_UNIFIED_HOST_TEST_NVS_SHIM_H

#include "nvs.h"

/** Empty every namespace, close every handle, clear injected faults and counters. */
void nvs_shim_reset(void);

/** Store a string directly, creating the namespace if needed. Not counted as a write. */
void nvs_shim_put(const char *ns, const char *key, const char *value);

/** The stored string, or NULL when the namespace or key does not exist. */
const char *nvs_shim_get(const char *ns, const char *key);

/** Make every following nvs_open() return @p err (ESP_OK restores normal behaviour). */
void nvs_shim_fail_open(esp_err_t err);

/** nvs_set_str() and nvs_erase_all() calls that reached the store since the last reset. */
int nvs_shim_write_count(void);

/** Handles opened and not yet closed. */
int nvs_shim_open_handles(void);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_NVS_SHIM_H */
