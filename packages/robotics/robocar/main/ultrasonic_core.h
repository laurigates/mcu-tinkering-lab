/**
 * @file ultrasonic_core.h
 * @brief Two-phase read state machine for an RCWL-1601 in I2C mode.
 *
 * Hardware-agnostic: no ESP-IDF header, so the identical code runs in the
 * firmware and in the host tests (the shared-core pattern in
 * .claude/rules/testing.md). ultrasonic.c supplies the bus operations.
 *
 * Protocol (docs/reference/datasheets/sensor--rcwl-1601.md § I2C mode): write
 * the single byte 0x01 to 0x57 to start a ranging cycle, wait for the
 * conversion, then read three bytes, most significant first, which are the
 * distance in micrometres.
 *
 * The read is split into two phases on purpose. Phase 1 triggers and releases
 * the bus; phase 2, on a later call once ULTRASONIC_CONVERSION_MS has passed,
 * reads and releases. Each bus operation is one short transaction, so the bus
 * is never held across a conversion and the PCA9685 writes that share it are
 * never blocked behind a 120 ms wait.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Sentinel written to *distance_cm when there is no valid reading. */
#define ULTRASONIC_DIST_ERROR UINT16_MAX

/** Shortest distance accepted as a reading: 2 cm at 3.3 V (Tinytronics). */
#define ULTRASONIC_MIN_RANGE_CM 2U

/** Longest distance accepted as a reading: 400 cm at 3.3 V (Tinytronics). A
 *  value beyond it is reported as an error, not clamped to it. */
#define ULTRASONIC_MAX_RANGE_CM 400U

/**
 * Wait between the 0x01 trigger and the 3-byte read, in ms.
 *
 * Not settled by any datasheet. Adafruit #4742 says "a few milliseconds";
 * M5Stack's RCWL-9620 library waits 120 ms and the RCWL_1X05 library defaults
 * to 100 ms. The reference page advises planning for 100–120 ms until a bench
 * measurement settles it, so this takes the conservative end. Shortening it is
 * a one-line change once the board page's "shortest wait" check is done.
 */
#define ULTRASONIC_CONVERSION_MS 120U

/** The trigger command byte (Adafruit #4742; Olimex SNS-RCWL1604). */
#define RCWL1601_CMD_TRIGGER 0x01U

/** Bytes in one reading: H, M, L of the distance in micrometres. */
#define RCWL1601_READ_LEN 3U

/** Outcome of one ultrasonic_core_step() call. */
typedef enum {
    ULTRASONIC_PENDING = 0, /**< No new reading on this call.                    */
    ULTRASONIC_READY,       /**< *distance_cm holds a fresh, in-range reading.   */
    ULTRASONIC_ERROR,       /**< Bus error, zero or out-of-range reading.         */
} ultrasonic_result_t;

/**
 * The two bus operations, each a complete select → transfer → release.
 * Return true on success. @p ctx is passed through unchanged.
 */
typedef struct {
    bool (*trigger)(void *ctx, uint8_t channel);
    bool (*read)(void *ctx, uint8_t channel, uint8_t raw[RCWL1601_READ_LEN]);
    void *ctx;
} ultrasonic_bus_t;

/** Per-sensor state. Zero-initialised (or ultrasonic_core_reset()) is idle. */
typedef struct {
    bool converting;     /**< Triggered, read not yet taken.             */
    bool ever_triggered; /**< trigger_ms is meaningful.                  */
    uint32_t trigger_ms; /**< When the last trigger was attempted.       */
} ultrasonic_core_t;

/** Return @p s to idle: the next step triggers immediately. */
void ultrasonic_core_reset(ultrasonic_core_t *s);

/**
 * Turn the three raw bytes into centimetres.
 *
 * A zero reading, or one outside ULTRASONIC_MIN_RANGE_CM..ULTRASONIC_MAX_RANGE_CM,
 * is ULTRASONIC_ERROR with *distance_cm = ULTRASONIC_DIST_ERROR — never a
 * clamped distance, which would read as a real obstacle (or a real clear path).
 */
ultrasonic_result_t ultrasonic_core_decode(const uint8_t raw[RCWL1601_READ_LEN],
                                           uint16_t *distance_cm);

/**
 * Advance one sensor by at most one bus operation.
 *
 * - Idle, and at least ULTRASONIC_CONVERSION_MS since the last trigger attempt
 *   (or never triggered): trigger. PENDING on success, ERROR if the write fails.
 * - Converting, and the conversion time has passed: read and decode.
 * - Otherwise: PENDING, with no bus traffic.
 *
 * Triggers are paced by ULTRASONIC_CONVERSION_MS whether the last cycle
 * succeeded or failed, so an absent sensor costs the bus no more than a working
 * one. @p now_ms may wrap; intervals are computed modulo 2^32.
 *
 * *distance_cm is written on READY and ERROR only.
 */
ultrasonic_result_t ultrasonic_core_step(ultrasonic_core_t *s, uint8_t channel, uint32_t now_ms,
                                         const ultrasonic_bus_t *bus, uint16_t *distance_cm);

#ifdef __cplusplus
}
#endif
