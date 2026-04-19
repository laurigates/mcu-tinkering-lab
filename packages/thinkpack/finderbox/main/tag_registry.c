/**
 * @file tag_registry.c
 * @brief NVS-backed persistence for the thinkpack-nfc registry.
 *
 * The serialisation format and registry logic live in `thinkpack-nfc`
 * and are already unit-tested on the host.  This file only wires the
 * blob (de)serialisation to NVS.
 */

#include "tag_registry.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "tag_reg";
static const char *NVS_NAMESPACE = "finderbox";
static const char *NVS_KEY_BLOB = "tag_blob";

esp_err_t tag_registry_load(thinkpack_nfc_registry_t *reg)
{
    if (reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    thinkpack_nfc_registry_init(reg);

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved registry — starting empty");
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t buf[THINKPACK_NFC_BLOB_MAX_SIZE];
    size_t len = sizeof(buf);
    ret = nvs_get_blob(h, NVS_KEY_BLOB, buf, &len);
    nvs_close(h);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No blob in NVS — starting empty");
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!thinkpack_nfc_registry_deserialize(reg, buf, len)) {
        ESP_LOGW(TAG, "Stored blob is malformed — ignoring");
        thinkpack_nfc_registry_init(reg);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Loaded %u tag entries from NVS", (unsigned)reg->count);
    return ESP_OK;
}

esp_err_t tag_registry_save(const thinkpack_nfc_registry_t *reg)
{
    if (reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[THINKPACK_NFC_BLOB_MAX_SIZE];
    size_t len = thinkpack_nfc_registry_serialize(reg, buf, sizeof(buf));
    if (len == 0) {
        ESP_LOGE(TAG, "Serialisation failed");
        return ESP_FAIL;
    }

    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(h, NVS_KEY_BLOB, buf, len);
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob/commit failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Saved %u tag entries (%u bytes)", (unsigned)reg->count, (unsigned)len);
    }
    return ret;
}

const thinkpack_nfc_entry_t *tag_registry_lookup(const thinkpack_nfc_registry_t *reg,
                                                 const uint8_t *uid, uint8_t uid_len)
{
    return thinkpack_nfc_lookup(reg, uid, uid_len);
}

esp_err_t tag_registry_upsert(thinkpack_nfc_registry_t *reg, const thinkpack_nfc_entry_t *entry)
{
    if (!thinkpack_nfc_registry_upsert(reg, entry)) {
        return ESP_ERR_NO_MEM;
    }
    return tag_registry_save(reg);
}

esp_err_t tag_registry_remove(thinkpack_nfc_registry_t *reg, const uint8_t *uid, uint8_t uid_len)
{
    if (!thinkpack_nfc_registry_remove(reg, uid, uid_len)) {
        return ESP_ERR_NOT_FOUND;
    }
    return tag_registry_save(reg);
}
