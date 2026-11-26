/**
 * @file motion_bound_core.h
 * @brief Executor-side time bound for `drive` and `rotate` goals (ADR-029 §4).
 *
 * The robot has no odometry or IMU, so a goal's distance or angle cannot be
 * closed in a loop. Instead each goal runs open loop for the time its distance
 * or angle takes at a speed estimate, never longer than MOTION_BOUND_CAP_MS,
 * and then the wheels coast until the planner writes another goal (#1057).
 * Without it a `rotate` spun in place until the next goal, 15 s at best and
 * 300 s when the planner backed off.
 *
 * Shared-core pattern (.claude/rules/testing.md): the firmware, the behaviour
 * twin and test/test_motion_bound_core.c compile this file unmodified.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest any one drive or rotate goal runs, in ms (ADR-029 §4: "a few
 *  seconds"; 3 s chosen 2026-11-26). */
#define MOTION_BOUND_CAP_MS 3000U

/** UNMEASURED. Straight-line speed at 100 % duty, in cm/s. From the twin's
 *  plant estimate (about 22 cm/s at 40 %). An overestimate stops short, which
 *  the next plan corrects; measure a timed run on the bench and pin it. */
#define MOTION_DRIVE_CM_PER_S_FULL 55U

/** UNMEASURED. Yaw rate of a rotate goal (~60 % duty), in deg/s. The same
 *  speed and estimate as the head-led body turn (head_aim.h,
 *  HEAD_BODY_YAW_DEG_PER_S). */
#define MOTION_ROTATE_DEG_PER_S 180U

/** No bound: the goal runs until the next one (track, stop). */
#define MOTION_BOUND_NONE UINT32_MAX

/** How long a drive of @p distance_cm at @p speed_pct runs, capped. 0 for a
 *  zero speed or distance. */
uint32_t motion_bound_drive_ms(uint16_t distance_cm, uint8_t speed_pct);

/** How long a rotate of @p angle_deg (either sign) runs, capped. 0 for 0°. */
uint32_t motion_bound_rotate_ms(int16_t angle_deg);

/** Elapsed time of the current goal. */
typedef struct {
    bool started;
    uint32_t seq;      /**< goal_state write the timer belongs to */
    uint32_t start_ms; /**< first tick that saw it                */
} motion_bound_t;

/** Forget any goal: the next call starts timing afresh. */
void motion_bound_reset(motion_bound_t *b);

/**
 * @brief One executor tick: has the goal outrun its bound?
 *
 * A @p goal_seq different from the last call's starts the timer at @p now_ms.
 * @p bound_ms comes from the two functions above, or MOTION_BOUND_NONE.
 * Wrap-safe across the day-49 millisecond rollover.
 */
bool motion_bound_expired(motion_bound_t *b, uint32_t goal_seq, uint32_t bound_ms, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
