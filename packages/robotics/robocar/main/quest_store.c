/**
 * @file quest_store.c
 * @brief SPIFFS persistence for the quest log (ADR-039 §1).
 *
 * The file is written whole: serialise under the lock into a PSRAM image,
 * release the lock, write `log.new`, then replace `log.tsv` with it. SPIFFS
 * refuses a rename onto an existing name, so the old file is removed first.
 * A power loss leaves one of three states, and the load handles each:
 *
 *   - only `log.tsv`: the normal case;
 *   - both: the write of `log.new` may be partial, so `log.tsv` is read;
 *   - only `log.new`: it was complete (removal comes after fclose), so it is
 *     read.
 *
 * The parser also skips an incomplete last line, so even a torn file loads
 * every whole record it holds.
 */

#include "quest_store.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "quest_store";

#define QUEST_STORE_BASE "/quest"
#define QUEST_STORE_PARTITION "spiffs"
#define QUEST_STORE_FILE QUEST_STORE_BASE "/log.tsv"
#define QUEST_STORE_TEMP QUEST_STORE_BASE "/log.new"
#define QUEST_STORE_IMAGE_BYTES (QUEST_LOG_SERIALIZED_MAX + 1)

typedef enum {
    STORE_LOADING = 0,
    STORE_READY,
    STORE_NO_MEMORY,
    STORE_NO_PARTITION,
} store_status_t;

static quest_log_t *s_log;
static char *s_image;
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task;
static volatile store_status_t s_status = STORE_LOADING;
static volatile size_t s_file_bytes;

const char *quest_store_state(void)
{
    switch (s_status) {
        case STORE_LOADING:
            return "loading";
        case STORE_READY:
            return "ready (" QUEST_STORE_FILE " on the spiffs partition)";
        case STORE_NO_MEMORY:
            return "unavailable: no PSRAM for the log";
        case STORE_NO_PARTITION:
            return "unavailable: spiffs partition missing or unmountable (flash over USB to "
                   "update the partition table)";
    }
    return "?";
}

size_t quest_store_file_bytes(void)
{
    return s_file_bytes;
}

quest_log_t *quest_store_lock(void)
{
    if (s_status != STORE_READY || s_lock == NULL) {
        return NULL;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    return s_log;
}

void quest_store_unlock(bool changed)
{
    xSemaphoreGive(s_lock);
    if (changed && s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

static bool file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

/* Read @p path into s_image. @return bytes read, or -1 when it cannot open. */
static long read_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return -1;
    }
    const size_t n = fread(s_image, 1, QUEST_STORE_IMAGE_BYTES - 1, f);
    fclose(f);
    s_image[n] = '\0';
    return (long)n;
}

static void load(void)
{
    const bool have_main = file_exists(QUEST_STORE_FILE);
    const bool have_temp = file_exists(QUEST_STORE_TEMP);
    const char *path = have_main ? QUEST_STORE_FILE : (have_temp ? QUEST_STORE_TEMP : NULL);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    quest_log_init(s_log);
    if (path != NULL) {
        const long n = read_file(path);
        if (n >= 0) {
            const size_t records = quest_log_parse(s_log, s_image, (size_t)n);
            s_file_bytes = (size_t)n;
            ESP_LOGI(TAG,
                     "loaded %u record(s) from %s (%ld bytes): %u quest(s), %u open, %u note(s)",
                     (unsigned)records, path, n, (unsigned)s_log->quest_count,
                     (unsigned)quest_log_open_count(s_log), (unsigned)s_log->note_count);
        } else {
            ESP_LOGW(TAG, "cannot open %s — starting an empty log", path);
        }
    } else {
        ESP_LOGI(TAG, "no quest log yet — starting an empty one");
    }
    xSemaphoreGive(s_lock);

    if (have_main && have_temp) {
        remove(QUEST_STORE_TEMP); /* a write that did not finish */
    }
}

static void save(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const size_t n = quest_log_serialize(s_log, s_image, QUEST_STORE_IMAGE_BYTES);
    xSemaphoreGive(s_lock);
    if (n >= QUEST_STORE_IMAGE_BYTES) {
        /* Cannot happen while QUEST_LOG_SERIALIZED_MAX bounds the caps
         * (test_quest_log_core.c fills every cap and checks). */
        ESP_LOGE(TAG, "log image %u bytes exceeds %u — not saved", (unsigned)n,
                 (unsigned)QUEST_STORE_IMAGE_BYTES);
        return;
    }

    FILE *f = fopen(QUEST_STORE_TEMP, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "cannot create %s — quest log not saved", QUEST_STORE_TEMP);
        return;
    }
    const size_t written = fwrite(s_image, 1, n, f);
    const int closed = fclose(f);
    if (written != n || closed != 0) {
        ESP_LOGE(TAG, "write of %s failed (%u/%u bytes) — keeping the previous log",
                 QUEST_STORE_TEMP, (unsigned)written, (unsigned)n);
        remove(QUEST_STORE_TEMP);
        return;
    }
    remove(QUEST_STORE_FILE);
    if (rename(QUEST_STORE_TEMP, QUEST_STORE_FILE) != 0) {
        /* log.new is complete and is what the next boot reads. */
        ESP_LOGE(TAG, "rename to %s failed — the log stays in %s", QUEST_STORE_FILE,
                 QUEST_STORE_TEMP);
        return;
    }
    s_file_bytes = n;
    ESP_LOGI(TAG, "saved %u bytes", (unsigned)n);
}

static esp_err_t mount(void)
{
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = QUEST_STORE_BASE,
        .partition_label = QUEST_STORE_PARTITION,
        .max_files = 2,
        /* The partition has never held anything, so the first boot formats it
         * (a few seconds, on this task, not the boot path). */
        .format_if_mount_failed = true,
    };
    const esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS mount failed (%s) — quests unavailable", esp_err_to_name(err));
        return err;
    }
    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info(QUEST_STORE_PARTITION, &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: %u of %u bytes used", (unsigned)used, (unsigned)total);
    }
    return ESP_OK;
}

static void quest_store_task(void *arg)
{
    (void)arg;
    if (mount() != ESP_OK) {
        s_status = STORE_NO_PARTITION;
        vTaskDelete(NULL);
        return;
    }
    load();
    s_status = STORE_READY;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Let a burst of changes land, then write them once. */
        vTaskDelay(pdMS_TO_TICKS(QUEST_STORE_SAVE_DELAY_MS));
        ulTaskNotifyTake(pdTRUE, 0);
        save();
    }
}

esp_err_t quest_store_start(void)
{
    s_log = heap_caps_calloc(1, sizeof(*s_log), MALLOC_CAP_SPIRAM);
    s_image = heap_caps_malloc(QUEST_STORE_IMAGE_BYTES, MALLOC_CAP_SPIRAM);
    s_lock = xSemaphoreCreateMutex();
    if (s_log == NULL || s_image == NULL || s_lock == NULL) {
        s_status = STORE_NO_MEMORY;
        ESP_LOGW(TAG, "no memory for the quest log — quests unavailable");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(quest_store_task, "quest_store", QUEST_STORE_TASK_STACK_SIZE, NULL,
                                QUEST_STORE_TASK_PRIORITY, &s_task,
                                QUEST_STORE_TASK_CORE) != pdPASS) {
        s_status = STORE_NO_MEMORY;
        ESP_LOGW(TAG, "quest store task not created — quests unavailable");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
