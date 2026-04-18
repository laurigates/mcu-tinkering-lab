/**
 * @file group_manager.c
 * @brief Peer table and capability aggregation for the ThinkPack mesh.
 */

#include "group_manager.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "group";

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

static thinkpack_peer_t s_peers[THINKPACK_MAX_PEERS];
static size_t s_peer_count = 0;
static SemaphoreHandle_t s_mutex = NULL;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

esp_err_t group_manager_init(void)
{
    memset(s_peers, 0, sizeof(s_peers));
    s_peer_count = 0;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create peer table mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Group manager initialised (capacity %d)", THINKPACK_MAX_PEERS);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Mutation                                                            */
/* ------------------------------------------------------------------ */

bool group_manager_upsert(const thinkpack_peer_t *peer)
{
    if (!peer) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Search for existing entry with matching MAC. */
    for (size_t i = 0; i < s_peer_count; i++) {
        if (memcmp(s_peers[i].mac, peer->mac, 6) == 0) {
            s_peers[i] = *peer;
            xSemaphoreGive(s_mutex);
            return false;
        }
    }

    /* New peer — append if there is capacity. */
    if (s_peer_count >= THINKPACK_MAX_PEERS) {
        ESP_LOGW(TAG, "Peer table full (%d), ignoring " MACSTR, THINKPACK_MAX_PEERS,
                 MAC2STR(peer->mac));
        xSemaphoreGive(s_mutex);
        return false;
    }

    s_peers[s_peer_count] = *peer;
    s_peer_count++;
    ESP_LOGI(TAG, "New peer " MACSTR " '%s' (count=%zu)", MAC2STR(peer->mac), peer->name,
             s_peer_count);

    xSemaphoreGive(s_mutex);
    return true;
}

void group_manager_prune(uint32_t now_ms, uint32_t stale_ms, group_manager_removed_cb_t cb,
                         void *user_ctx)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    size_t i = 0;
    while (i < s_peer_count) {
        uint32_t age_ms = now_ms - s_peers[i].last_seen_ms;
        if (age_ms > stale_ms) {
            ESP_LOGI(TAG, "Pruning stale peer " MACSTR " (age %" PRIu32 " ms)",
                     MAC2STR(s_peers[i].mac), age_ms);

            if (cb) {
                cb(&s_peers[i], user_ctx);
            }

            /* Shift-remove: overwrite this slot with the last entry. */
            if (i < s_peer_count - 1) {
                s_peers[i] = s_peers[s_peer_count - 1];
            }
            memset(&s_peers[s_peer_count - 1], 0, sizeof(thinkpack_peer_t));
            s_peer_count--;
            /* Do not increment i — recheck the slot we just filled. */
        } else {
            i++;
        }
    }

    xSemaphoreGive(s_mutex);
}

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

size_t group_manager_count(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    size_t count = s_peer_count;
    xSemaphoreGive(s_mutex);
    return count;
}

const thinkpack_peer_t *group_manager_get(size_t index)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const thinkpack_peer_t *result = (index < s_peer_count) ? &s_peers[index] : NULL;
    xSemaphoreGive(s_mutex);
    return result;
}

const thinkpack_peer_t *group_manager_find(const uint8_t mac[6])
{
    if (!mac) {
        return NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    const thinkpack_peer_t *result = NULL;
    for (size_t i = 0; i < s_peer_count; i++) {
        if (memcmp(s_peers[i].mac, mac, 6) == 0) {
            result = &s_peers[i];
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return result;
}

uint16_t group_manager_aggregate_capabilities(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint16_t aggregate = 0;
    for (size_t i = 0; i < s_peer_count; i++) {
        aggregate |= s_peers[i].capabilities;
    }
    xSemaphoreGive(s_mutex);
    return aggregate;
}
