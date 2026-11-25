/**
 * @file ota_interval.c
 * @brief See ota_interval.h.
 */

#include "ota_interval.h"

#include <stddef.h>

bool ota_interval_ticks_from_minutes(uint32_t minutes, uint32_t tick_rate_hz, uint32_t *out_ticks)
{
    if (out_ticks == NULL) {
        return false;
    }

    /* Same formula pdMS_TO_TICKS() uses — (ms * tick_rate_hz) / 1000 — but
     * with every intermediate in a 64-bit type, so the multiply cannot wrap
     * before the division runs. */
    uint64_t ms = (uint64_t)minutes * 60ull * 1000ull;
    uint64_t ticks = (ms * (uint64_t)tick_rate_hz) / 1000ull;

    if (ticks > (uint64_t)UINT32_MAX) {
        return false;
    }

    *out_ticks = (uint32_t)ticks;
    return true;
}

uint32_t ota_interval_next_wait_ticks(uint32_t checks_run, bool fetched_yet,
                                      uint32_t first_delay_ticks, uint32_t retry_ticks,
                                      uint32_t interval_ticks)
{
    uint32_t wait = interval_ticks;
    if (checks_run == 0u) {
        wait = first_delay_ticks;
    } else if (!fetched_yet && checks_run <= OTA_FIRST_CHECK_MAX_RETRIES) {
        wait = retry_ticks;
    }
    return (wait < interval_ticks) ? wait : interval_ticks;
}
