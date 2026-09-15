/**
 * @file head_aim.h
 * @brief Pan/tilt head aiming for the reactive executor — pure decision logic.
 *
 * Issue #511, option C: the planner has no head tool. `track(box_2d)` stays its
 * only aiming primitive, and the 30 Hz executor decides how to satisfy it:
 *
 *   - The head leads. A track box is converted once, when a new goal arrives,
 *     into a body-relative bearing: the pan angle the head held when the
 *     planner's frame was captured, plus the box centre's offset in that frame.
 *     The head slews toward that bearing, inside the live travel limits.
 *   - The wheels follow only when needed. Once the bearing passes
 *     HEAD_PAN_ENGAGE_PCT of the pan limit on that side, the body turns in place
 *     toward it. Each tick of turning subtracts the estimated yaw from the
 *     bearing, so the head re-centres as the heading catches up, and the turn
 *     ends once the bearing is inside HEAD_RECENTRE_DEADBAND_DEG.
 *   - Any other goal, or a stale one, re-centres the head.
 *
 * The bearing is latched per goal rather than recomputed from the box every
 * tick. The box describes one frame; there is no newer frame on the device
 * between plans, so re-reading it every tick would integrate the same offset
 * forever and wind the head into its end stop.
 *
 * Nothing here touches hardware. reactive_controller.c feeds in the live servo
 * position and limits, writes what this returns, and turns the wheels. That
 * split is what lets the 49-day millisecond wrap, a narrowed limit mid-track,
 * and a whole body turn run as host tests.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Tuning constants
 * ========================================================================= */

/** Period the step functions assume, in ms. Must equal REACTIVE_LOOP_PERIOD_MS;
 *  reactive_controller.c asserts it. */
#define HEAD_AIM_TICK_MS 33U

/**
 * Camera field of view, in degrees, used to turn a box offset into an angle.
 *
 * UNMEASURED. The OV3660 module fitted to this XIAO has no published lens
 * figure in this repo; these are a typical value for the stock lens at 4:3.
 * An error here scales where the head points, not whether it points the right
 * way — the next plan's frame corrects it. Measure (a target at a known angle,
 * read its box) and pin the result.
 */
#define HEAD_CAMERA_HFOV_DEG 60
#define HEAD_CAMERA_VFOV_DEG 45

/** Maximum head travel per executor tick, in degrees (~180 deg/s). An SG90 is
 *  rated ~600 deg/s unloaded; the slew exists so a new goal swings the head
 *  instead of snapping it, which also bounds the stall current on each step. */
#define HEAD_SLEW_DEG_PER_TICK 6

/** The body starts turning once the bearing passes this percentage of the pan
 *  limit on that side. Below 100 so the head keeps some travel in reserve for
 *  the next plan's correction instead of arriving pinned at its limit. */
#define HEAD_PAN_ENGAGE_PCT 80

/** The body stops turning once the bearing is within this many degrees of
 *  straight ahead. Also the minimum engage threshold, so a limit set at or near
 *  zero cannot make the turn start and stop on alternate ticks. */
#define HEAD_RECENTRE_DEADBAND_DEG 5

/**
 * Estimated body yaw rate while turning in place at HEAD_BODY_TURN_SPEED, in
 * degrees per second.
 *
 * UNMEASURED — there is no IMU or odometry, so this is the only thing that says
 * when the heading has "caught up". Chosen high on purpose: an overestimate ends
 * the turn early and leaves a residual the next plan's frame corrects, while an
 * underestimate spins the robot past its target. Measure a timed in-place spin
 * and pin the result.
 */
#define HEAD_BODY_YAW_DEG_PER_S 180

/** Wheel speed (0..255) for the in-place body turn. Fixed, and equal to the
 *  rotate goal's ~60 %, because HEAD_BODY_YAW_DEG_PER_S is only meaningful at
 *  one speed. */
#define HEAD_BODY_TURN_SPEED 153U

/**
 * An unchanged head command is re-sent at most this often, in ms.
 *
 * Same reasoning as MOTOR_REFRESH_INTERVAL_MS: the 30 Hz loop would otherwise
 * restate two identical PCA9685 counts every tick, and suppression without an
 * expiry would never re-assert a PCA9685 that browned out or was re-seated.
 */
#define HEAD_REFRESH_INTERVAL_MS 1000U

/* =========================================================================
 * Types
 * ========================================================================= */

/** Live travel limits, in degrees about centre, as servo_get_limits() reports. */
typedef struct {
    int16_t pan_min;
    int16_t pan_max;
    int16_t tilt_min;
    int16_t tilt_max;
} head_limits_t;

/** Where the head is (the last angle the servo driver accepted). */
typedef struct {
    int16_t pan_deg;
    int16_t tilt_deg;
} head_pose_t;

/** A track goal's box, Gemini-normalised 0..1000, origin top-left. */
typedef struct {
    uint16_t ymin;
    uint16_t xmin;
    uint16_t ymax;
    uint16_t xmax;
} head_box_t;

/** What the wheels should do this tick while a track goal is being aimed. */
typedef enum {
    HEAD_BODY_HOLD = 0, /**< The head covers the target; wheels creep straight. */
    HEAD_BODY_TURN_CW,  /**< Turn right in place toward the head's direction.   */
    HEAD_BODY_TURN_CCW, /**< Turn left in place toward the head's direction.    */
} head_body_t;

/** Result of one aiming step. */
typedef struct {
    head_pose_t cmd;  /**< Head command for this tick, always inside the limits. */
    head_body_t body; /**< Wheel instruction (HOLD for a centring step).         */
} head_aim_out_t;

/** Module state. Zero-initialise or call head_aim_reset(). */
typedef struct {
    bool latched;        /**< A bearing has been latched for goal_seq.          */
    uint32_t goal_seq;   /**< goal_state write sequence the bearing belongs to. */
    float bearing_deg;   /**< Body-relative bearing, + = right.                 */
    float tilt_deg;      /**< Target tilt, + = up.                              */
    bool body_turning;   /**< Hysteresis latch for the body turn.               */
    bool written;        /**< false until the first write, and after a failure. */
    head_pose_t last;    /**< The command last written successfully.            */
    uint32_t written_ms; /**< When `last` was written.                          */
} head_aim_t;

/* =========================================================================
 * API
 * ========================================================================= */

/** Forget the latched goal, the body-turn latch and the write shadow. */
void head_aim_reset(head_aim_t *s);

/**
 * @brief One aiming step for a fresh track goal.
 *
 * @param goal_seq       goal_state's write sequence for this goal. A value
 *                       different from the latched one re-latches the bearing.
 * @param box            The goal's box.
 * @param capture        Head pose when the goal's frame was captured.
 * @param now            Current head pose.
 * @param lim            Live travel limits.
 * @param body_can_turn  false while the wheels are not the executor's (obstacle
 *                       reflex, manual lease). The turn is then neither
 *                       commanded nor counted, so the yaw estimate does not
 *                       advance while the robot is held still.
 */
head_aim_out_t head_aim_track(head_aim_t *s, uint32_t goal_seq, const head_box_t *box,
                              head_pose_t capture, head_pose_t now, const head_limits_t *lim,
                              bool body_can_turn);

/**
 * @brief One step back toward centre — stale goal or any non-track goal.
 *
 * Also drops the latched bearing and the body-turn latch, so a later track goal
 * starts clean.
 */
head_aim_out_t head_aim_centre(head_aim_t *s, head_pose_t now, const head_limits_t *lim);

/**
 * @brief Whether @p cmd needs writing: it differs from the pose, differs from
 *        the last write, or the last write is older than HEAD_REFRESH_INTERVAL_MS.
 *
 * Unsigned millisecond arithmetic, so the uint32 wrap at day 49 is a correct
 * elapsed time rather than a refresh that never falls due again.
 */
bool head_aim_write_due(const head_aim_t *s, head_pose_t cmd, head_pose_t now, uint32_t now_ms);

/** Record a write attempt. A failed write invalidates the shadow so the next
 *  tick retries rather than remembering a command the chip never received. */
void head_aim_note_write(head_aim_t *s, head_pose_t cmd, uint32_t now_ms, bool ok);

/**
 * @brief Fold the head's pan into a body-relative drive heading.
 *
 * `drive(heading_deg)` is stated relative to what the planner saw, and the
 * planner saw a frame taken with the head turned by @p capture_pan_deg. The
 * body heading is their sum, wrapped into -180..180.
 */
int16_t head_aim_body_heading(int16_t heading_deg, int16_t capture_pan_deg);

#ifdef __cplusplus
}
#endif
