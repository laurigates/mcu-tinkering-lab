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

/**
 * Delay from the OTA task's start to its first manifest check (issue #1055).
 * The task starts on the boot's first link-up; the delay lets MQTT connect so
 * a refusal or announcement status reaches the broker. Under a minute, so an
 * online robot sees a newer release soon after link-up rather than only after
 * staying up a whole OTA_CHECK_INTERVAL_MIN. The rollback-confirm timer is
 * separate, and an install still waits out OTA_ANNOUNCE_WINDOW_MS (60 s) after
 * the check, past OTA_STABILITY_TIMEOUT_MS (60 s) from the task's start.
 */
#define OTA_FIRST_CHECK_DELAY_MS 15000U

/**
 * Ticks the OTA task waits before its next manifest check: @p first_delay_ticks
 * before the boot's first check, @p interval_ticks before every later one. The
 * first wait is capped at @p interval_ticks, so it is never longer than a
 * later one (a short bench interval, or the overflow fallback).
 *
 * @param checked_yet        True once the task has run a check this boot.
 * @param first_delay_ticks  OTA_FIRST_CHECK_DELAY_MS in ticks.
 * @param interval_ticks     OTA_CHECK_INTERVAL_MIN in ticks.
 */
uint32_t ota_interval_next_wait_ticks(bool checked_yet, uint32_t first_delay_ticks,
                                      uint32_t interval_ticks);

#ifdef __cplusplus
}
#endif

#endif  // OTA_INTERVAL_H
