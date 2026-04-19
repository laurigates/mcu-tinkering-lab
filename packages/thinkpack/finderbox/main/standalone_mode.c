/**
 * @file standalone_mode.c
 * @brief Finderbox main loop — scan, lookup, dispatch behaviour.
 */

#include "standalone_mode.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_ring.h"
#include "piezo.h"
#include "tag_registry.h"
#include "thinkpack_rc522.h"

static const char *TAG = "standalone";

/* Poll interval (ms) between RC522 scans. */
#define SCAN_INTERVAL_MS 50

/* Suppress repeat triggers of the same UID within this window. */
#define REPEAT_SUPPRESS_MS 1500

/* Chime tone timings — short upbeat bleep. */
#define CHIME_FREQ_HZ 1760 /* A6 */
#define CHIME_DURATION_MS 120

/* Colour feedback timings. */
#define COLOR_DURATION_MS 600

static thinkpack_nfc_registry_t *s_registry = NULL;
static TaskHandle_t s_task;

/* Last detected tag state (for replay) */
static uint8_t s_last_uid[RC522_MAX_UID_LEN];
static uint8_t s_last_uid_len = 0;
static uint32_t s_last_detect_ms = 0;
static thinkpack_nfc_entry_t s_last_entry;
static bool s_have_last = false;

/* ------------------------------------------------------------------ */
/* Behaviour dispatch                                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief Paint a hue across the whole ring (HSV at max saturation).
 *
 * Called at the end of a chime (COLOR_DURATION_MS), and used directly
 * by COLOR behaviour.
 */
static void paint_hue(uint8_t hue)
{
    for (uint32_t i = 0; i < LED_RING_COUNT; i++) {
        led_ring_set_pixel_hsv(i, hue, 255, 255);
    }
    led_ring_show();
}

static void dispatch_behavior(const thinkpack_nfc_entry_t *entry)
{
    if (entry == NULL) {
        return;
    }

    switch ((thinkpack_nfc_behavior_t)entry->behavior) {
        case THINKPACK_NFC_BEHAVIOR_CHIME:
            ESP_LOGI(TAG, "CHIME — hue=%u label='%s'", (unsigned)entry->param, entry->label);
            piezo_play_tone(CHIME_FREQ_HZ, CHIME_DURATION_MS);
            paint_hue(entry->param);
            vTaskDelay(pdMS_TO_TICKS(COLOR_DURATION_MS));
            led_ring_clear();
            led_ring_show();
            break;

        case THINKPACK_NFC_BEHAVIOR_COLOR:
            ESP_LOGI(TAG, "COLOR — hue=%u label='%s'", (unsigned)entry->param, entry->label);
            paint_hue(entry->param);
            break;

        case THINKPACK_NFC_BEHAVIOR_STORY:
            /* STORY is mesh-mediated; PR E wires this up. */
            ESP_LOGI(TAG, "STORY track=%u (deferred to PR E)", (unsigned)entry->param);
            piezo_play_tone(CHIME_FREQ_HZ, CHIME_DURATION_MS);
            break;

        case THINKPACK_NFC_BEHAVIOR_SEEK:
            /* Standalone-mode no-op; group mode uses this. */
            ESP_LOGI(TAG, "SEEK label='%s' (no-op standalone)", entry->label);
            break;

        case THINKPACK_NFC_BEHAVIOR_NONE:
        default:
            ESP_LOGD(TAG, "NONE — ignoring UID");
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Scan task                                                            */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void scan_task(void *arg)
{
    (void)arg;

    uint8_t uid[RC522_MAX_UID_LEN];
    uint8_t uid_len = 0;

    for (;;) {
        if (thinkpack_rc522_poll_tag(uid, &uid_len)) {
            uint32_t t = now_ms();
            bool same_as_last =
                (uid_len == s_last_uid_len) && (memcmp(uid, s_last_uid, uid_len) == 0);
            bool within_window = same_as_last && (t - s_last_detect_ms < REPEAT_SUPPRESS_MS);

            if (!within_window) {
                ESP_LOGI(TAG, "Tag detected — %u bytes", (unsigned)uid_len);
                const thinkpack_nfc_entry_t *entry = tag_registry_lookup(s_registry, uid, uid_len);
                if (entry != NULL) {
                    memcpy(&s_last_entry, entry, sizeof(s_last_entry));
                    s_have_last = true;
                    dispatch_behavior(entry);
                } else {
                    ESP_LOGI(TAG, "Unknown UID — no registry entry");
                }

                memcpy(s_last_uid, uid, uid_len);
                s_last_uid_len = uid_len;
            }
            s_last_detect_ms = t;
        }
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

esp_err_t standalone_mode_start(thinkpack_nfc_registry_t *registry)
{
    if (registry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_registry = registry;

    BaseType_t ok = xTaskCreate(scan_task, "nfc_scan", 4096, NULL, 5, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Scan task creation failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Standalone scan task running (poll=%d ms)", SCAN_INTERVAL_MS);
    return ESP_OK;
}

void standalone_mode_replay_last(void)
{
    if (!s_have_last) {
        ESP_LOGI(TAG, "Replay requested but no tag has been seen");
        return;
    }
    ESP_LOGI(TAG, "Replay — behaviour=%u label='%s'", (unsigned)s_last_entry.behavior,
             s_last_entry.label);
    dispatch_behavior(&s_last_entry);
}
