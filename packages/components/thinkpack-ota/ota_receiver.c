/**
 * @file ota_receiver.c
 * @brief Mesh-side OTA receiver.
 *
 * ESP-only. Registers a mesh event callback, drives the chunk-state
 * machine, and applies the image on MSG_OTA_COMPLETE. Keeps the
 * firmware-side diff tiny by consolidating the logic here rather than
 * duplicating it across the 4 follower variants.
 *
 * Design trade-offs:
 *  - Uses a single static session state; concurrent OTA sessions are
 *    not supported (nor needed — one Brainbox pushes at a time).
 *  - Allocates the image buffer on demand via heap_caps_malloc; the
 *    Brainbox side pre-computes total size so callers can plan for
 *    RAM pressure.
 *  - The mesh component supports a single event callback. To chain
 *    the OTA receiver alongside the existing group_mode callback, we
 *    install our own wrapper that forwards to both.
 */

#ifdef ESP_PLATFORM

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "espnow_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "thinkpack_ota.h"
#include "thinkpack_protocol.h"

static const char *TAG = "thinkpack_ota_rx";

typedef struct {
    bool active;
    uint8_t my_box_type;
    thinkpack_ota_chunk_state_t state;
    uint8_t *image_buf;
    size_t image_buf_size;
    uint32_t *bitmap_buf;
    size_t bitmap_buf_words;
    thinkpack_ota_chained_cb_t chained_cb;
    void *chained_ctx;
} ota_rx_state_t;

static ota_rx_state_t s_rx;

static void release_buffers(void)
{
    if (s_rx.image_buf != NULL) {
        heap_caps_free(s_rx.image_buf);
        s_rx.image_buf = NULL;
        s_rx.image_buf_size = 0;
    }
    if (s_rx.bitmap_buf != NULL) {
        heap_caps_free(s_rx.bitmap_buf);
        s_rx.bitmap_buf = NULL;
        s_rx.bitmap_buf_words = 0;
    }
    s_rx.active = false;
}

static void handle_manifest(const ota_manifest_payload_t *m)
{
    thinkpack_ota_error_t err = thinkpack_ota_manifest_validate(m);
    if (err != THINKPACK_OTA_ERR_NONE) {
        ESP_LOGW(TAG, "Rejecting manifest: err=%d", (int)err);
        return;
    }
    if (!thinkpack_ota_manifest_targets_box(m->target_box_mask, s_rx.my_box_type)) {
        ESP_LOGI(TAG, "Manifest not for box_type=%u — ignoring", (unsigned)s_rx.my_box_type);
        return;
    }

    release_buffers();

    s_rx.image_buf = heap_caps_malloc(m->total_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (s_rx.image_buf == NULL) {
        /* Fall back to internal RAM if PSRAM missing. */
        s_rx.image_buf = heap_caps_malloc(m->total_size, MALLOC_CAP_8BIT);
    }
    if (s_rx.image_buf == NULL) {
        ESP_LOGE(TAG, "Out of memory for %u-byte OTA image", (unsigned)m->total_size);
        return;
    }
    s_rx.image_buf_size = m->total_size;

    size_t bw = (size_t)(((uint32_t)m->chunk_count + 31u) / 32u);
    s_rx.bitmap_buf = heap_caps_calloc(bw, sizeof(uint32_t), MALLOC_CAP_8BIT);
    if (s_rx.bitmap_buf == NULL) {
        ESP_LOGE(TAG, "Out of memory for chunk bitmap");
        release_buffers();
        return;
    }
    s_rx.bitmap_buf_words = bw;

    if (thinkpack_ota_chunk_state_init(&s_rx.state, m, s_rx.image_buf, s_rx.image_buf_size,
                                       s_rx.bitmap_buf, s_rx.bitmap_buf_words) != ESP_OK) {
        ESP_LOGE(TAG, "chunk_state_init failed");
        release_buffers();
        return;
    }
    s_rx.active = true;
    ESP_LOGI(TAG, "OTA session open: v%u, %u bytes, %u chunks", (unsigned)m->version,
             (unsigned)m->total_size, (unsigned)m->chunk_count);
}

static void handle_chunk(const ota_chunk_payload_t *c)
{
    if (!s_rx.active) {
        return;
    }
    thinkpack_ota_error_t err = thinkpack_ota_chunk_state_update(&s_rx.state, c);
    if (err != THINKPACK_OTA_ERR_NONE) {
        ESP_LOGW(TAG, "Chunk %u rejected: err=%d", (unsigned)c->chunk_index, (int)err);
        release_buffers();
    }
}

static void handle_complete(const ota_complete_payload_t *cp)
{
    if (!s_rx.active) {
        return;
    }
    if (cp->version != s_rx.state.version) {
        ESP_LOGW(TAG, "COMPLETE version mismatch (%u vs %u)", (unsigned)cp->version,
                 (unsigned)s_rx.state.version);
        release_buffers();
        return;
    }
    thinkpack_ota_error_t err = thinkpack_ota_chunk_state_finalize(&s_rx.state);
    if (err != THINKPACK_OTA_ERR_NONE) {
        ESP_LOGE(TAG, "Finalize failed: err=%d", (int)err);
        release_buffers();
        return;
    }
    ESP_LOGI(TAG, "OTA assembly complete — applying image");
    esp_err_t ar = thinkpack_ota_apply(cp, s_rx.image_buf, s_rx.image_buf_size);
    if (ar != ESP_OK) {
        ESP_LOGE(TAG, "thinkpack_ota_apply: %s", esp_err_to_name(ar));
        release_buffers();
    }
    /* On success we do not return: apply() reboots. */
}

static void on_mesh_event(const thinkpack_mesh_event_data_t *ev, void *user_ctx)
{
    /* Forward first to the chained (non-OTA) callback if any. The
     * chained callback typedef uses the opaque struct declared in
     * thinkpack_ota.h; the actual concrete type is the same. */
    if (s_rx.chained_cb != NULL) {
        s_rx.chained_cb((const struct thinkpack_mesh_event_data_s *)ev, s_rx.chained_ctx);
    }
    (void)user_ctx;

    if (ev == NULL || ev->type != THINKPACK_EVENT_COMMAND_RECEIVED || ev->packet == NULL) {
        return;
    }
    const thinkpack_packet_t *p = ev->packet;
    switch (p->msg_type) {
        case MSG_OTA_MANIFEST:
            if (p->data_length >= sizeof(ota_manifest_payload_t)) {
                handle_manifest((const ota_manifest_payload_t *)p->data);
            }
            break;
        case MSG_OTA_CHUNK:
            if (p->data_length >= offsetof(ota_chunk_payload_t, data)) {
                handle_chunk((const ota_chunk_payload_t *)p->data);
            }
            break;
        case MSG_OTA_COMPLETE:
            if (p->data_length >= sizeof(ota_complete_payload_t)) {
                handle_complete((const ota_complete_payload_t *)p->data);
            }
            break;
        default:
            break;
    }
}

esp_err_t thinkpack_ota_receiver_init(uint8_t my_box_type)
{
    memset(&s_rx, 0, sizeof(s_rx));
    s_rx.my_box_type = my_box_type;
    /* Install our wrapper; all existing callers set their callback
     * *before* calling this init, so we capture it here. */
    extern esp_err_t thinkpack_mesh_set_event_callback(thinkpack_mesh_event_cb_t, void *);
    /* The mesh API does not expose a getter; the caller is responsible
     * for re-registering any prior callback via the chain hooks. */
    return thinkpack_mesh_set_event_callback(on_mesh_event, NULL);
}

/**
 * @brief Chain an existing event callback under the OTA receiver.
 *
 * Must be called *before* thinkpack_ota_receiver_init so that the
 * chained pointer is installed before we overwrite the mesh callback.
 */
esp_err_t thinkpack_ota_receiver_chain_callback(thinkpack_ota_chained_cb_t cb, void *ctx)
{
    s_rx.chained_cb = cb;
    s_rx.chained_ctx = ctx;
    return ESP_OK;
}

#endif /* ESP_PLATFORM */
