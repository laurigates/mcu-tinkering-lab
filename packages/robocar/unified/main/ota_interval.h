/**
 * @file ota_interval.h
 * @brief 64-bit tick-count arithmetic for the OTA manifest poll interval.
 *
 * Pure C, no ESP-IDF or FreeRTOS dependency — see
 * .claude/rules/testing.md's shared-core pattern.
 *
 * This project builds the non-SMP FreeRTOS kernel
 * (`# CONFIG_FREERTOS_SMP is not set`), whose pdMS_TO_TICKS() macro
 * (components/freertos/FreeRTOS-Kernel/include/freertos/projdefs.h in the
 * espressif/idf:v5.4 image) computes `((x) * configTICK_RATE_HZ) / 1000`
 * entirely in TickType_t (uint32_t on this target). Feeding it
 * `OTA_CHECK_INTERVAL_MIN * 60 * 1000` milliseconds — 21,600,000 for the
 * shipped 360-minute interval — overflows that 32-bit multiply before the
 * division ever runs (only the SMP kernel's projdefs.h casts to uint64_t).
 * The result was 125,163 ticks (~125 s) instead of 21,600,000 (~21,600 s):
 * the OTA task polled the manifest roughly every two minutes instead of
 * every six hours.
 *
 * ota_interval_ticks_from_minutes() does the same arithmetic the macro does,
 * in a 64-bit intermediate, and fails CLOSED when the result would not fit
 * back into a uint32_t TickType_t — the caller must fall back to a
 * known-safe interval rather than silently truncating.
 */

#ifndef OTA_INTERVAL_H
#define OTA_INTERVAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute the number of RTOS ticks in @p minutes minutes at @p tick_rate_hz
 * ticks per second, replicating pdMS_TO_TICKS()'s own formula
 * (`(ms * tick_rate_hz) / 1000`) but entirely in a 64-bit intermediate.
 *
 * Fails CLOSED: returns false — and leaves *out_ticks untouched by the
 * caller's contract, since the caller must not act on it — when @p out_ticks
 * is NULL, or when the computed tick count exceeds UINT32_MAX (the widest a
 * TickType_t can be on this target). The caller must treat a false return as
 * "use a known-safe default interval", never guess or truncate.
 *
 * @param minutes       Interval length in minutes.
 * @param tick_rate_hz  The RTOS tick rate (configTICK_RATE_HZ).
 * @param out_ticks     Receives the tick count on success.
 */
bool ota_interval_ticks_from_minutes(uint32_t minutes, uint32_t tick_rate_hz, uint32_t *out_ticks);

#ifdef __cplusplus
}
#endif

#endif  // OTA_INTERVAL_H
