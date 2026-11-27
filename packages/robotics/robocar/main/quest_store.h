/**
 * @file quest_store.h
 * @brief The quest log on the robot: SPIFFS file, mutex, save task (ADR-039 §1).
 *
 * Firmware glue around quest_log_core.c. The log lives in PSRAM; a task on
 * core 1 mounts the `spiffs` partition, loads `/quest/log.tsv`, and writes the
 * whole file back after every change, through a temporary file and a rename.
 * A robot whose partition table predates the `spiffs` partition, or whose
 * mount fails, runs with quests unavailable and says so in `quest`.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "quest_log_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Allocate the log and start the store task. Non-fatal for the boot. */
esp_err_t quest_store_start(void);

/** Lock the log for reading or changing. @return NULL while it is loading or
 *  when storage is unavailable; then do not call quest_store_unlock(). */
quest_log_t *quest_store_lock(void);

/** Release the lock. @p changed schedules a write of the file. */
void quest_store_unlock(bool changed);

/** One line on where the store is: loading, ready, or why it is unavailable. */
const char *quest_store_state(void);

/** Bytes of the file as last written or read, 0 before. */
size_t quest_store_file_bytes(void);

#ifdef __cplusplus
}
#endif
