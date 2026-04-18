/**
 * @file reactive_controller.h
 * @brief 30 Hz reactive motor executor — the fast control layer.
 *
 * Runs as a FreeRTOS task pinned to Core 0 (motor-critical core, matching
 * motor_task and peripheral_task).  Every 33 ms the executor:
 *
 *   1. Reads the ultrasonic rangefinder.
 *   2. If obstacle < STOP_THRESHOLD_CM, latches a safety stop (overrides any
 *      planner goal until the obstacle clears).
 *   3. Otherwise reads the current goal from goal_state and drives the motors
 *      toward it using a simple skid-steer mix (GOAL_KIND_DRIVE) or a
 *      proportional visual-servo (GOAL_KIND_TRACK).
 *
 * The planner task (Core 1) writes goals via goal_state_write(); this executor
 * only reads.  Motor commands are issued via the existing motor_controller.h
 * API — no direct GPIO writes.
 *
 * Host-test build
 * ---------------
 * Compile with -DREACTIVE_CONTROLLER_HOST_TEST=1 to replace FreeRTOS and
 * ESP-IDF dependencies with thin POSIX shims so the decision logic (goal
 * dispatch, skid-steer mix, visual-servo P controller, reflex latch) can be
 * unit-tested on the host without hardware.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef REACTIVE_CONTROLLER_HOST_TEST
#include "esp_err.h"
#else
#include <errno.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_INVALID_ARG (-2)
#define ESP_ERR_INVALID_STATE (-3)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Tuning constants (override via compiler flags if desired)
 * ========================================================================= */

/** Distance threshold (cm) below which the obstacle reflex fires. */
#ifndef STOP_THRESHOLD_CM
#define STOP_THRESHOLD_CM 15U
#endif

/**
 * Visual-servo proportional gain.
 *
 * Input: bounding-box centre error in normalised units (0..1000, centre=500).
 * Output: turn bias added/subtracted from base forward speed (0..255).
 *
 * With the camera field of view typically spanning ~500 units side-to-side,
 * a max error of 500 units at Kp=0.15 produces a turn bias of 75 out of 255
 * — roughly 30 % differential.  Raise Kp to tighten tracking; lower it to
 * reduce oscillation on a slippery surface.
 */
#ifndef TRACK_KP
#define TRACK_KP 0.15f
#endif

/**
 * Smoothing window for the running-mean distance filter.
 * 3 samples at 30 Hz gives ~100 ms of lag — enough to reject single-spike
 * glitches without delaying the obstacle reflex meaningfully.
 */
#define ULTRASONIC_SMOOTH_N 3U

/** Reactive controller task stack size in bytes. */
#define REACTIVE_TASK_STACK_SIZE 4096U

/** Reactive controller task FreeRTOS priority (between motor_task=6 and ai_task=3). */
#define REACTIVE_TASK_PRIORITY 5U

/** Core affinity: Core 0 matches motor_task for deterministic scheduling. */
#define REACTIVE_TASK_CORE 0

/** Loop period in ms (target 30 Hz). */
#define REACTIVE_LOOP_PERIOD_MS 33U

/* =========================================================================
 * Telemetry
 * ========================================================================= */

/**
 * @brief Snapshot of reactive controller runtime metrics.
 *
 * Retrieved via reactive_controller_get_telemetry().  Safe to call from any
 * task; the values are copied under a brief critical section.
 */
typedef struct {
    uint16_t distance_cm;      /**< Latest smoothed ultrasonic reading.         */
    bool reflex_active;        /**< True when obstacle reflex has latched stop.  */
    uint32_t loop_hz;          /**< Measured loop rate over the last second.     */
    uint32_t stack_high_water; /**< Stack headroom from uxTaskGetStackHighWaterMark. */
} reactive_telemetry_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the reactive controller and spawn the executor task.
 *
 * Calls ultrasonic_init() internally.  goal_state_init() must have been called
 * before this function.
 *
 * Idempotent — a second call returns ESP_OK without creating a second task.
 *
 * @return ESP_OK on success, ESP_FAIL if task creation fails.
 */
esp_err_t reactive_controller_init(void);

/**
 * @brief Stop and delete the executor task.
 *
 * Calls motor_stop() before deleting the task so the robot does not coast.
 * Safe to call from any context.  After this call, reactive_controller_init()
 * may be called again.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the controller was not running.
 */
esp_err_t reactive_controller_stop(void);

/**
 * @brief Retrieve a snapshot of the latest telemetry.
 *
 * Thread-safe.  Does not block.
 *
 * @param[out] out  Destination buffer.  Must not be NULL.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if out is NULL.
 */
esp_err_t reactive_controller_get_telemetry(reactive_telemetry_t *out);

#ifdef __cplusplus
}
#endif
