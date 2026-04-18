#include "game_logic.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "game";

#define NVS_NAMESPACE "scavenger"
#define NVS_KEY_FOUND "found_mask"

/*
 * Bitmask of found tags. Bit N corresponds to TAG_REGISTRY[N].
 * Stored in NVS for persistence across power cycles.
 * Supports up to 32 tags (uint32_t).
 */
static uint32_t found_mask = 0;

static esp_err_t save_progress(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u32(handle, NVS_KEY_FOUND, found_mask);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}

static esp_err_t load_progress(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        found_mask = 0;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_get_u32(handle, NVS_KEY_FOUND, &found_mask);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        found_mask = 0;
        ret = ESP_OK;
    }

    nvs_close(handle);
    return ret;
}

static const tag_entry_t *find_tag(const uint8_t *uid, uint8_t uid_len)
{
    for (size_t i = 0; i < TAG_COUNT; i++) {
        if (TAG_REGISTRY[i].uid_len == uid_len && memcmp(TAG_REGISTRY[i].uid, uid, uid_len) == 0) {
            return &TAG_REGISTRY[i];
        }
    }
    return NULL;
}

static int tag_index(const tag_entry_t *entry)
{
    return (int)(entry - TAG_REGISTRY);
}

esp_err_t game_logic_init(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased — reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = load_progress();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Game initialized: %d/%d tags found (mask=0x%08lX)", game_logic_found_count(),
             game_logic_total_count(), (unsigned long)found_mask);

    return ESP_OK;
}

tag_result_t game_logic_process_tag(const uint8_t *uid, uint8_t uid_len, const tag_entry_t **entry)
{
    *entry = NULL;

    const tag_entry_t *found = find_tag(uid, uid_len);
    if (!found) {
        return TAG_RESULT_UNKNOWN;
    }

    *entry = found;
    int idx = tag_index(found);

    if (found_mask & (1U << idx)) {
        return TAG_RESULT_ALREADY_FOUND;
    }

    /* New find! */
    found_mask |= (1U << idx);
    save_progress();

    ESP_LOGI(TAG, "New find: %s (#%d) — %d/%d", found->name, found->track_id,
             game_logic_found_count(), game_logic_total_count());

    return TAG_RESULT_NEW_FIND;
}

int game_logic_found_count(void)
{
    int count = 0;
    uint32_t mask = found_mask;
    while (mask) {
        count += mask & 1;
        mask >>= 1;
    }
    return count;
}

int game_logic_total_count(void)
{
    return (int)TAG_COUNT;
}

bool game_logic_all_found(void)
{
    return game_logic_found_count() >= game_logic_total_count();
}

esp_err_t game_logic_reset(void)
{
    found_mask = 0;
    esp_err_t ret = save_progress();
    ESP_LOGI(TAG, "Game reset — all progress cleared");
    return ret;
}
