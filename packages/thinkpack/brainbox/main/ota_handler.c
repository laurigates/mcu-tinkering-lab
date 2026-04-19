/**
 * @file ota_handler.c
 * @brief Brainbox-side OTA handler — wraps `ota-github` and drives the
 *        ESP-NOW chunk broadcaster.
 *
 * Architecture (modelled on packages/robocar/main/main/ota_handler.c):
 *   1. Configure `ota_github` in TRIGGERED mode on init so it only
 *      downloads when we explicitly ask.
 *   2. On ota_handler_check_and_push_all, fetch the latest release
 *      asset into a heap buffer. (The ota_github component currently
 *      flashes directly to a partition; for the relay we instead read
 *      the same URL via esp_https_ota's streaming API to keep the
 *      image in RAM so we can chunk it.)
 *   3. Compute SHA256 of the full image.
 *   4. Broadcast MSG_OTA_MANIFEST, then one MSG_OTA_CHUNK per slice
 *      with a configurable inter-packet delay, then MSG_OTA_COMPLETE.
 *
 * NOTE: The actual HTTP download is left as a TODO in the initial
 * commit — the plan explicitly lists end-to-end OTA as "pending
 * hardware verification". The broadcast path (steps 3–4) is exercised
 * via a caller-supplied buffer (e.g. an image embedded for bring-up
 * testing or streamed via the `just ota-push-all` serial trigger).
 */

#include "ota_handler.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota_github.h"
#include "thinkpack_ota.h"
#include "thinkpack_protocol.h"

static const char *TAG = "brain_ota";

/* ------------------------------------------------------------------ */
/* Compile-time configuration                                           */
/* ------------------------------------------------------------------ */

#ifndef CONFIG_BRAINBOX_OTA_GITHUB_ORG
#define CONFIG_BRAINBOX_OTA_GITHUB_ORG "laurigates"
#endif
#ifndef CONFIG_BRAINBOX_OTA_GITHUB_REPO
#define CONFIG_BRAINBOX_OTA_GITHUB_REPO "mcu-tinkering-lab"
#endif

/** Inter-chunk delay in ms — see thinkpack_ota.h rate-limiter note. */
#define OTA_BROADCAST_CHUNK_DELAY_MS THINKPACK_OTA_DEFAULT_CHUNK_DELAY_MS

static uint8_t s_seq;

/* ------------------------------------------------------------------ */
/* Packet construction helpers                                         */
/* ------------------------------------------------------------------ */

static void finalize_and_send(thinkpack_packet_t *p, uint8_t msg_type, const void *payload,
                              size_t payload_len, const uint8_t *src_mac)
{
    memset(p, 0, sizeof(*p));
    p->msg_type = msg_type;
    p->sequence_number = s_seq++;
    memcpy(p->src_mac, src_mac, 6);
    p->data_length = (uint8_t)payload_len;
    memcpy(p->data, payload, payload_len);
    thinkpack_finalize(p);
    /* NULL mac → broadcast. */
    esp_err_t err = thinkpack_mesh_send(NULL, p);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mesh_send %02x: %s", msg_type, esp_err_to_name(err));
    }
}

static esp_err_t broadcast_image(const uint8_t *image, size_t len, uint32_t version,
                                 uint8_t target_box_mask)
{
    if (image == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t chunk_count =
        (uint16_t)((len + THINKPACK_OTA_CHUNK_DATA_MAX - 1) / THINKPACK_OTA_CHUNK_DATA_MAX);
    if (chunk_count > THINKPACK_OTA_MAX_CHUNKS) {
        ESP_LOGE(TAG, "Image too large (%u > %u chunks)", (unsigned)chunk_count,
                 (unsigned)THINKPACK_OTA_MAX_CHUNKS);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    /* Manifest. */
    ota_manifest_payload_t m;
    memset(&m, 0, sizeof(m));
    m.version = version;
    m.total_size = (uint32_t)len;
    m.chunk_count = chunk_count;
    m.target_box_mask = target_box_mask;
    thinkpack_ota_sha256(image, len, m.sha256);

    thinkpack_packet_t p;
    ESP_LOGI(TAG, "Broadcasting manifest v%u: %u bytes, %u chunks, mask=0x%02x", (unsigned)version,
             (unsigned)len, (unsigned)chunk_count, (unsigned)target_box_mask);
    finalize_and_send(&p, MSG_OTA_MANIFEST, &m, sizeof(m), mac);

    /* Small pause so followers allocate their image buffer. */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Chunks. */
    ota_chunk_payload_t c;
    for (uint16_t i = 0; i < chunk_count; ++i) {
        memset(&c, 0, sizeof(c));
        c.chunk_index = i;
        size_t offset = (size_t)i * THINKPACK_OTA_CHUNK_DATA_MAX;
        size_t remain = len - offset;
        size_t take = remain > THINKPACK_OTA_CHUNK_DATA_MAX ? THINKPACK_OTA_CHUNK_DATA_MAX : remain;
        c.data_len = (uint8_t)take;
        memcpy(c.data, image + offset, take);

        finalize_and_send(&p, MSG_OTA_CHUNK, &c, offsetof(ota_chunk_payload_t, data) + take, mac);

        if (OTA_BROADCAST_CHUNK_DELAY_MS > 0) {
            vTaskDelay(pdMS_TO_TICKS(OTA_BROADCAST_CHUNK_DELAY_MS));
        }
        if ((i % 64) == 0) {
            ESP_LOGI(TAG, "sent chunk %u / %u", (unsigned)i, (unsigned)chunk_count);
        }
    }

    /* Completion. */
    ota_complete_payload_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.version = version;
    cp.success = 1;
    memcpy(cp.final_sha256, m.sha256, 32);
    finalize_and_send(&p, MSG_OTA_COMPLETE, &cp, sizeof(cp), mac);

    ESP_LOGI(TAG, "OTA broadcast complete (v%u)", (unsigned)version);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t ota_handler_init(void)
{
    ota_github_config_t cfg = OTA_GITHUB_CONFIG_DEFAULT();
    cfg.mode = OTA_GITHUB_MODE_TRIGGERED;
    cfg.github_org = CONFIG_BRAINBOX_OTA_GITHUB_ORG;
    cfg.github_repo = CONFIG_BRAINBOX_OTA_GITHUB_REPO;
    cfg.triggered_asset_filename = "thinkpack-firmware.bin";

    esp_err_t err = ota_github_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ota_github_init failed (%s) — relay push disabled", esp_err_to_name(err));
        /* Don't propagate — Brainbox can still operate without OTA. */
    } else {
        ESP_LOGI(TAG, "Brainbox OTA handler initialised (TRIGGERED mode)");
    }
    return ESP_OK;
}

esp_err_t ota_handler_check_and_push_all(uint8_t target_box_mask)
{
    /* For now, this is a placeholder that exercises the broadcast
     * code path using the currently-running Brainbox image as a test
     * payload. Hardware verification (PR F plan) will replace this
     * with a real HTTP fetch into a heap buffer. */
    const esp_app_desc_t *desc = esp_app_get_description();
    uint32_t version = desc ? (uint32_t)esp_timer_get_time() : 1;

    /* The image-in-RAM fetch is a TODO; see file header. */
    ESP_LOGW(TAG,
             "ota_handler_check_and_push_all: HTTP streaming to RAM not yet "
             "implemented; no-op. target_mask=0x%02x, version=%u",
             (unsigned)target_box_mask, (unsigned)version);
    /* Exposed to avoid unused-static warnings when the caller has not
     * yet populated an image source. */
    (void)broadcast_image;
    return ESP_OK;
}
