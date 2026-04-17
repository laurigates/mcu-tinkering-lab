/**
 * @file goal_state.h
 * @brief Shared goal state — mutex-protected, last-write-wins with TTL.
 *
 * The planner task (Core 1, ~1 Hz) writes structured goals via
 * goal_state_write().  The reactive executor (Core 0, ~30 Hz) reads them via
 * goal_state_read().  When the planner goes silent the goal ages out and the
 * executor falls back to GOAL_KIND_STOP.
 *
 * Host-test build
 * ---------------
 * Compile with -DGOAL_STATE_HOST_TEST=1 to replace the FreeRTOS mutex and
 * esp_timer with thin POSIX shims so the module can be exercised on the host
 * without any ESP-IDF dependency.  The clock can be further overridden via
 * goal_state_set_clock_override() to drive TTL scenarios deterministically.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef GOAL_STATE_HOST_TEST
#include "esp_err.h"
#else
/* ---- Host-test compatibility shims ---- */
#include <errno.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG (-2)
#define ESP_ERR_INVALID_STATE (-3)
/* ---------------------------------------- */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Goal types
 * ========================================================================= */

/**
 * @brief Discriminator for the goal union below.
 */
typedef enum {
    GOAL_KIND_NONE = 0, /**< No goal has been set yet (initial state). */
    GOAL_KIND_STOP,     /**< Halt all motion immediately.              */
    GOAL_KIND_DRIVE,    /**< Drive toward a heading for a distance.    */
    GOAL_KIND_TRACK,    /**< Visual-servo toward a bounding box.       */
    GOAL_KIND_ROTATE,   /**< Rotate in place by an angle.              */
} goal_kind_t;

/**
 * @brief A single structured goal emitted by the planner.
 *
 * Bounding-box coordinates use Gemini Robotics-ER 1.6 normalised units
 * (0..1000 for both axes, origin top-left).
 */
typedef struct {
    goal_kind_t kind;
    union {
        /** GOAL_KIND_DRIVE params */
        struct {
            int16_t heading_deg;  /**< Target heading, -180..180 deg    */
            uint16_t distance_cm; /**< Distance to travel in cm         */
            uint8_t speed_pct;    /**< Speed 0-100 %                    */
        } drive;

        /** GOAL_KIND_TRACK params */
        struct {
            uint16_t ymin;         /**< Top edge, 0..1000               */
            uint16_t xmin;         /**< Left edge, 0..1000              */
            uint16_t ymax;         /**< Bottom edge, 0..1000            */
            uint16_t xmax;         /**< Right edge, 0..1000             */
            uint8_t max_speed_pct; /**< Speed cap, 0-100 %              */
        } track;

        /** GOAL_KIND_ROTATE params */
        struct {
            int16_t angle_deg; /**< +CW, -CCW                           */
        } rotate;
    } params;
} goal_t;

/* =========================================================================
 * Constants
 * ========================================================================= */

/** Default freshness window in milliseconds (1.5× the 1 Hz planner period). */
#define GOAL_STATE_DEFAULT_TTL_MS 1500U

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the module. Call once at boot before any other function.
 *
 * Idempotent — a second call returns ESP_OK without reinitialising.
 *
 * @return ESP_OK on success, ESP_FAIL if the mutex could not be created.
 */
esp_err_t goal_state_init(void);

/**
 * @brief Write a new goal, replacing whatever was there before (last-write-wins).
 *
 * Timestamps the write internally.  Safe to call from any task context.
 * Short critical section — no logging inside the lock.
 *
 * @param goal    Pointer to the goal to copy in.  Must not be NULL.
 * @param ttl_ms  Freshness window in ms.  Pass 0 to use
 *                GOAL_STATE_DEFAULT_TTL_MS.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL goal), or ESP_ERR_INVALID_STATE
 *         (not initialised).
 */
esp_err_t goal_state_write(const goal_t *goal, uint32_t ttl_ms);

/**
 * @brief Read the current goal.
 *
 * Copies the internal goal into *out.  If the goal has aged past its TTL,
 * *is_fresh is set to false AND out->kind is forced to GOAL_KIND_STOP so the
 * caller can act without extra branching.
 *
 * Safe to call from any task context.  Short critical section.
 *
 * @param out       Destination buffer.  Must not be NULL.
 * @param is_fresh  Set to true if the goal is within its TTL, false otherwise.
 *                  Must not be NULL.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (NULL args), or ESP_ERR_INVALID_STATE
 *         (not initialised).
 */
esp_err_t goal_state_read(goal_t *out, bool *is_fresh);

/**
 * @brief Atomically set the goal to GOAL_KIND_STOP with the default TTL.
 *
 * Useful for obstacle reflexes that need to override the planner immediately,
 * and for unit tests that need a clean reset.
 *
 * @return ESP_OK or ESP_ERR_INVALID_STATE (not initialised).
 */
esp_err_t goal_state_force_stop(void);

/* =========================================================================
 * Test / instrumentation hooks (only meaningful with GOAL_STATE_HOST_TEST=1)
 * ========================================================================= */

/**
 * @brief Override the clock used for TTL calculations.
 *
 * Pass a function returning microseconds since an arbitrary epoch.  Pass NULL
 * to restore the default (esp_timer_get_time on target, a monotonic fallback
 * on host).
 *
 * This hook is compiled in on all builds so the linker dependency is
 * consistent; on target it is simply never called.
 */
void goal_state_set_clock_override(int64_t (*now_us_fn)(void));

#ifdef __cplusplus
}
#endif
