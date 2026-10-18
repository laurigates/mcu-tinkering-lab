/**
 * @file reactive_controller.h
 * @brief 30 Hz reactive motor executor — the fast control layer.
 *
 * Runs as a FreeRTOS task pinned to Core 0 (motor-critical core, matching
 * motor_task and peripheral_task).  Every 33 ms the executor:
 *
 *   1. Polls the front and rear RCWL-1601 rangefinders (non-blocking: a read
 *      completes over several ticks, see ultrasonic.h).
 *   2. If a front reading < STOP_THRESHOLD_CM, latches a safety stop
 *      (overrides any planner goal until the obstacle clears). The rear
 *      sensor is reported in telemetry only.
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

#include "ultrasonic_core.h"

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
 * Smoothing window for the front sensor's running-mean distance filter, in
 * readings (not ticks — only a completed read enters the filter).
 *
 * The mean governs only the reflex's *release* and the distance in telemetry;
 * it does not delay the trip. See "Obstacle reflex timing" below for why.
 */
#define ULTRASONIC_SMOOTH_N 3U

/**
 * Obstacle reflex timing (issue #791)
 * -----------------------------------
 * The reflex trips on the first front reading below STOP_THRESHOLD_CM, and
 * releases only once that latest reading AND the running mean are both at or
 * above it. Tripping on the mean instead cost ULTRASONIC_SMOOTH_N readings: in
 * I2C mode a reading completes only every ~165 ms, so a sudden obstacle took
 * ~500 ms to pull a 3-reading mean under 15 cm.
 *
 * The cost is a false trip: one spurious short reading brakes for one reading
 * period (~165 ms) until the next clear one, because the mean never fell. That
 * errs toward stopping, which is the side a reflex should err on. A failed read
 * still enters the filter as max range and never trips it.
 *
 * Worst case from an obstacle appearing to the brake, with
 * C = ceil(ULTRASONIC_CONVERSION_MS / REACTIVE_LOOP_PERIOD_MS):
 *
 *     REACTIVE_REFLEX_WORST_CASE_MS = (2*C + 1) * REACTIVE_LOOP_PERIOD_MS
 *
 * The obstacle appears just after a trigger ranged the clear path; that
 * cycle's read, C ticks later, is still clear; the next trigger is one tick
 * after the read, and its read C ticks later trips the reflex on the same tick.
 * With the unmeasured 120 ms wait, C = 4 and the bound is 9 * 33 = 297 ms —
 * still well above the GPIO driver's ~100 ms. Shortening
 * ULTRASONIC_CONVERSION_MS once the bench measurement in
 * docs/reference/datasheets/sensor--rcwl-1601.md settles it brings the bound
 * down with it (a 33 ms wait gives C = 1 and 99 ms). This is the nominal 30 Hz
 * loop; a late tick adds its lateness. test_reflex_latency.c measures the bound
 * over the real ultrasonic_core.c read cycle and fails if it drifts from this.
 */
#define REACTIVE_REFLEX_CONVERSION_TICKS \
    ((ULTRASONIC_CONVERSION_MS + REACTIVE_LOOP_PERIOD_MS - 1U) / REACTIVE_LOOP_PERIOD_MS)
#define REACTIVE_REFLEX_WORST_CASE_MS \
    ((2U * REACTIVE_REFLEX_CONVERSION_TICKS + 1U) * REACTIVE_LOOP_PERIOD_MS)

/**
 * Consecutive failed rangefinder reads before a sensor is declared down.
 * Counted per sensor, over completed reads only: 5 reads is ~0.6–0.8 s, long
 * enough that a single bad reading (absorbed by the smoothing filter) does not
 * raise a false alarm.
 */
#define ULTRASONIC_FAIL_STREAK 5U

/** Reactive controller task stack size in bytes. */
#define REACTIVE_TASK_STACK_SIZE 4096U

/** Reactive controller task FreeRTOS priority (between motor_task=6 and ai_task=3). */
#define REACTIVE_TASK_PRIORITY 5U

/** Core affinity: Core 0 matches motor_task for deterministic scheduling. */
#define REACTIVE_TASK_CORE 0

/** Loop period in ms (target 30 Hz). */
#define REACTIVE_LOOP_PERIOD_MS 33U

/**
 * Lifetime of a manual override, in ms. A manual command is a *lease*, not a
 * mode: the console operator holding "forward" must not leave the robot driving
 * after the serial link drops, and the planner must be able to take back over
 * on its own. Matches the old COMMAND_TIMEOUT_MS watchdog this replaced.
 */
#define REACTIVE_MANUAL_TTL_MS 1000U

/* =========================================================================
 * Manual override
 * ========================================================================= */

/**
 * @brief Manual movement commands from the serial console / bench testing.
 *
 * These exist because the goal vocabulary cannot express them: goals describe
 * *intent* ("drive toward this heading", "track this box"), while bench testing
 * needs raw actuation including reverse, which no goal kind encodes.
 */
typedef enum {
    REACTIVE_MANUAL_STOP = 0,
    REACTIVE_MANUAL_FORWARD,
    REACTIVE_MANUAL_BACKWARD,
    REACTIVE_MANUAL_LEFT,
    REACTIVE_MANUAL_RIGHT,
    REACTIVE_MANUAL_ROTATE_CW,
    REACTIVE_MANUAL_ROTATE_CCW,
} reactive_manual_cmd_t;

/**
 * @brief Hand a manual movement command to the executor.
 *
 * The executor remains the only writer of motor output (see the project
 * CLAUDE.md "Don't" list) — callers do not touch motor_controller.h. Two
 * properties follow from routing through here rather than driving the motors
 * directly, and both are the point of this function:
 *
 *   - the obstacle reflex still wins, so a manual command cannot drive the
 *     robot into a wall;
 *   - the command expires after @p ttl_ms, after which the planner's goal
 *     resumes without any explicit hand-back.
 *
 * Safe to call from any task. Does not block.
 *
 * @param cmd     Movement to apply.  REACTIVE_MANUAL_STOP clears the override.
 * @param speed   Motor speed 0..255 (ignored for STOP and the rotate commands,
 *                which use their own fixed rate).
 * @param ttl_ms  Lease duration; pass 0 for REACTIVE_MANUAL_TTL_MS.
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the executor is not running.
 */
esp_err_t reactive_controller_manual(reactive_manual_cmd_t cmd, uint8_t speed, uint32_t ttl_ms);

/* =========================================================================
 * Pan/tilt head (issue #511)
 * =========================================================================
 *
 * The executor owns the head the same way it owns the wheels: it aims a track
 * goal with it and re-centres it otherwise (see head_aim.h). A console command
 * that moves the head therefore takes a *head lease*, the counterpart of
 * reactive_controller_manual(), instead of writing the servos itself — a second
 * writer would be overwritten within one refresh interval and would fight the
 * aiming loop meanwhile.
 *
 * While any head lease is live, a track goal is steered with the wheels alone,
 * exactly as on a board with no servos.
 */

/** Head axis for reactive_controller_head_hold(). */
typedef enum {
    REACTIVE_HEAD_PAN = 0,
    REACTIVE_HEAD_TILT,
} reactive_head_axis_t;

/**
 * Lifetime of a console head lease, in ms. Long, unlike the 1 s motor lease:
 * a held head angle is a bench diagnostic somebody wants to look at, and a head
 * left pointing somewhere is not a hazard the way a driving robot is.
 */
#define REACTIVE_HEAD_HOLD_TTL_MS 30000U

/**
 * @brief Hold one head axis at an angle for @p ttl_ms, overriding aiming.
 *
 * The other axis holds where it is. The executor clamps the held angle to the
 * live travel limits on every write, so a limit narrowed during the lease still
 * applies. Safe from any task; does not block.
 *
 * @param ttl_ms  Lease duration; 0 for REACTIVE_HEAD_HOLD_TTL_MS.
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the executor is not running.
 */
esp_err_t reactive_controller_head_hold(reactive_head_axis_t axis, int16_t angle_deg,
                                        uint32_t ttl_ms);

/**
 * @brief Keep the executor's hands off the head for @p ttl_ms.
 *
 * For a caller that drives the servos itself over several seconds — the only
 * one is servo_exercise() on peripheral_task. The executor writes nothing while
 * this is live. End it with reactive_controller_head_release() when done; the
 * TTL is a backstop, not the normal end.
 *
 * @param ttl_ms  Lease duration; 0 for REACTIVE_HEAD_HOLD_TTL_MS.
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the executor is not running.
 */
esp_err_t reactive_controller_head_external(uint32_t ttl_ms);

/** End any head lease now. The executor resumes from wherever the head is. */
esp_err_t reactive_controller_head_release(void);

/** What the executor did with the head on its last tick. */
typedef enum {
    REACTIVE_HEAD_UNAVAILABLE = 0, /**< Servos not initialised or released.   */
    REACTIVE_HEAD_CENTRE,          /**< No fresh track goal: re-centring.     */
    REACTIVE_HEAD_AIM,             /**< Head covers the track target.         */
    REACTIVE_HEAD_TURN,            /**< Body turning toward the head.         */
    REACTIVE_HEAD_LEASE,           /**< A console lease owns the head.        */
} reactive_head_mode_t;

/** Short name for a head mode, for the console and logs. */
const char *reactive_head_mode_name(reactive_head_mode_t mode);

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
    /** Front sensor, running mean of ULTRASONIC_SMOOTH_N readings. Gates the
     *  reflex release only: the trip uses the latest reading, so this can read
     *  >= STOP_THRESHOLD_CM while reflex_active is true (issue #791). */
    uint16_t distance_cm;
    bool reflex_active;        /**< True when obstacle reflex has latched stop.  */
    uint32_t loop_hz;          /**< Measured loop rate over the last second.     */
    uint32_t stack_high_water; /**< Stack headroom from uxTaskGetStackHighWaterMark. */
    bool manual_active;        /**< True while a manual override lease is live.  */
    /** True when the rangefinder has failed for ULTRASONIC_FAIL_STREAK readings
     *  in a row. Distinguishes "nothing ahead" from "the sensor stopped
     *  answering" — the filter maps both to max range, so without this the two
     *  are indistinguishable from outside. */
    bool sensor_failed;
    /** What the executor did with the pan/tilt head on the last tick. */
    reactive_head_mode_t head_mode;
    /** Rear sensor's latest reading in cm, unsmoothed; ULTRASONIC_DIST_ERROR
     *  until the first reading and after any failed one. Not part of the
     *  reflex. */
    uint16_t distance_rear_cm;
    /** The rear sensor's counterpart of sensor_failed. */
    bool rear_failed;
} reactive_telemetry_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the reactive controller and spawn the executor task.
 *
 * Calls ultrasonic_init() internally. A rangefinder failure is not fatal: the
 * executor runs without them, and telemetry reports both sensors as failed
 * once ULTRASONIC_FAIL_STREAK reads have failed. goal_state_init() must have
 * been called before this function.
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
