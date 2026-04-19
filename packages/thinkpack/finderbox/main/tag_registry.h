/**
 * @file tag_registry.h
 * @brief NVS-backed persistence for the thinkpack-nfc registry.
 *
 * The registry data structure and (de)serialisation logic live in the
 * pure-logic component `thinkpack-nfc` (host-testable, no ESP-IDF
 * dependencies).  This wrapper only adds the NVS load/save calls.
 */

#ifndef FINDERBOX_TAG_REGISTRY_H
#define FINDERBOX_TAG_REGISTRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "thinkpack_nfc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Load the registry blob from NVS, or initialise empty on miss.
 *
 * @return ESP_OK on successful load or when initialising empty.
 *         Other esp_err_t on NVS failures.
 */
esp_err_t tag_registry_load(thinkpack_nfc_registry_t *reg);

/**
 * @brief Persist the registry blob to NVS.
 */
esp_err_t tag_registry_save(const thinkpack_nfc_registry_t *reg);

/**
 * @brief Lookup helper — forwards to @ref thinkpack_nfc_lookup.
 *
 * Provided so callers can depend on one API surface instead of two.
 */
const thinkpack_nfc_entry_t *tag_registry_lookup(const thinkpack_nfc_registry_t *reg,
                                                 const uint8_t *uid, uint8_t uid_len);

/** Upsert and immediately persist. */
esp_err_t tag_registry_upsert(thinkpack_nfc_registry_t *reg, const thinkpack_nfc_entry_t *entry);

/** Remove and immediately persist. */
esp_err_t tag_registry_remove(thinkpack_nfc_registry_t *reg, const uint8_t *uid, uint8_t uid_len);

#ifdef __cplusplus
}
#endif

#endif /* FINDERBOX_TAG_REGISTRY_H */
