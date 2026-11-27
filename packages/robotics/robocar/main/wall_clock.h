/**
 * @file wall_clock.h
 * @brief Wall time from SNTP, for timestamps that outlive a boot (ADR-039 §1).
 *
 * TLS here does not validate certificate dates (`CONFIG_MBEDTLS_HAVE_TIME_DATE`
 * is unset), so nothing else needs the clock. The quest log does: a note
 * written today has to read as "yesterday" after a reboot. Until the first
 * sync a time is seconds since boot, flagged unsynced (quest_time_from()).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "quest_log_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start SNTP. Call on a link-up; later calls do nothing. Never blocks. */
void wall_clock_start(void);

/** True once the clock reads a time past QUEST_LOG_EPOCH_VALID_S. */
bool wall_clock_synced(void);

/** Now, as Unix seconds when synced, else seconds since boot. */
quest_time_t wall_clock_now(void);

/** "2026-11-27 14:05" in local time when @p t is synced, else "boot+<s>s". */
size_t wall_clock_format(quest_time_t t, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
