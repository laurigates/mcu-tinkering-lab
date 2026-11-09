/**
 * @file reactive_controller.c
 * @brief 30 Hz reactive motor executor.
 *
 * See reactive_controller.h for architecture overview.
 *
 * Skid-steer mix (GOAL_KIND_DRIVE)
 * ---------------------------------
 * heading_deg is treated as a turn bias, not an absolute compass heading (we
 * have no IMU or odometry).  Positive heading_deg turns right; negative turns
 * left.  The heading is stated in the frame the planner saw, so the head's pan
 * at capture is folded in first (head_aim_body_heading()).  The mix is:
 *
 *   base   = speed_pct * 255 / 100
 *   bias   = |heading_deg| * base / 180   (linear, max 100 % differential)
 *   right  = (heading_deg > 0) ? base - bias : base
 *   left   = (heading_deg < 0) ? base - bias : base
 *   dir    = 1 (forward)
 *
 * This is intentionally simple.  A full heading-hold loop requires odometry
 * or a compass — neither is available in this phase.  The planner is expected
 * to keep re-issuing updated heading_deg goals as the scene changes.
 *
 * Track (GOAL_KIND_TRACK) — head leads, wheels follow
 * ---------------------------------------------------
 * With the pan/tilt head available the head aims at the box and the wheels
 * creep straight at max_speed_pct; once the target's bearing nears the pan
 * limit the body turns in place toward it and the head re-centres as it goes.
 * See head_aim.h. Without the head (servos not initialised, released, or under
 * a console lease) the original wheel-only P controller runs:
 *
 *   cx    = (xmin + xmax) / 2          centre of bounding box, 0..1000
 *   error = cx - 500                   signed error, -500..500
 *   turn  = TRACK_KP * error           turn bias, ±75 at max error with Kp=0.15
 *   base  = max_speed_pct * 255 / 100
 *   right = base - turn   (clamped 0..255)
 *   left  = base + turn   (clamped 0..255)
 *   dir   = 1 (always creep forward while tracking)
 *
 * Obstacle reflex
 * ---------------
 * If a single front reading < STOP_THRESHOLD_CM, motors are braked and
 * reflex_active is latched. The rear sensor never trips it. The latch clears
 * automatically once the latest reading and the running mean are both back at
 * or above STOP_THRESHOLD_CM; no planner intervention is required. Tripping on
 * one reading rather than on the mean is what keeps the I2C read's ~165 ms
 * cadence from multiplying into the reaction time (issue #791); the bound is
 * REACTIVE_REFLEX_WORST_CASE_MS in reactive_controller.h.
 * The reflex is logged once on entry and once on exit to avoid log spam.
 * The head keeps aiming through a reflex — it cannot hit anything.
 *
 * Cliff reflex
 * ------------
 * Two downward floor sensors, front and rear (cliff_sensor.c, decisions in
 * cliff_core.c). A sensor that loses the floor brakes the robot, then backs it
 * away from the edge if the other side has floor. While a sensor sees no floor,
 * any wheel command moving toward it (manual or planner) is replaced by a
 * brake. It ranks above the obstacle reflex: a front edge reverses even with an
 * obstacle ahead, and a rear edge never pulls forward into one. Without an
 * MCP23017, or after `cliff off`, it is disarmed and nothing is filtered.
 * docs/firmware/cliff-sensors.md has the wiring and the bench checks.
 *
 * Stack high-water-mark
 * ---------------------
 * Printed every 5 s via uxTaskGetStackHighWaterMark for gate G2 verification.
 */

#include "reactive_controller.h"

#include <stdlib.h>
#include <string.h>

#include "cliff_core.h"
#include "cliff_sensor.h"
#include "goal_state.h"
#include "head_aim.h"
#include "idle_gaze.h"
#include "motor_controller.h"
#include "servo_controller.h"
#include "ultrasonic.h"

_Static_assert(HEAD_AIM_TICK_MS == REACTIVE_LOOP_PERIOD_MS,
               "head_aim's yaw estimate assumes the executor's loop period");

/* =========================================================================
 * Shared by the target and host-test builds
 * =========================================================================
 *
 * Everything that decides what the actuators do lives here, compiled once, so
 * the host tests exercise the shipped decision code rather than a copy of it.
 * Only timing, locking and the task itself differ between the two builds.
 */

/** Which kind of head lease is live. */
typedef enum {
    HEAD_LEASE_NONE = 0,
    HEAD_LEASE_HOLD,     /**< Console: hold these angles.              */
    HEAD_LEASE_EXTERNAL, /**< Another task drives the servos itself.   */
} head_lease_kind_t;

typedef struct {
    head_lease_kind_t kind;
    int16_t pan_deg;
    int16_t tilt_deg;
} head_lease_t;

/** Executor-private aiming state. Touched only by the executor tick. */
static head_aim_t s_head_aim;

/** Idle gaze (ADR-029 §1). The state is the executor tick's alone; the switch
 *  is a bool any task may write, read once per tick. */
static idle_gaze_t s_gaze;
static volatile bool s_gaze_enabled = true;

/** Whether the head has held still for HEAD_SETTLE_MS, from the last tick. */
static bool s_head_settled;

/* Supplied by each build below: whether a voice turn is listening (the head
 * faces forward), and the gaze's random source. */
static bool head_face_forward(void);
static uint32_t gaze_rng(void *ctx);

void reactive_controller_set_gaze(bool on)
{
    s_gaze_enabled = on;
}

bool reactive_controller_gaze_enabled(void)
{
    return s_gaze_enabled;
}

/** Pre-OTA hold: the wheels coast every tick and nothing else moves them. A
 *  bool any task may write, read once per tick, like the gaze switch. */
static volatile bool s_hold = false;

void reactive_controller_set_hold(bool on)
{
    s_hold = on;
}

bool reactive_controller_hold_active(void)
{
    return s_hold;
}

const char *reactive_head_mode_name(reactive_head_mode_t mode)
{
    switch (mode) {
        case REACTIVE_HEAD_CENTRE:
            return "centre";
        case REACTIVE_HEAD_AIM:
            return "aim";
        case REACTIVE_HEAD_TURN:
            return "turn";
        case REACTIVE_HEAD_LEASE:
            return "lease";
        case REACTIVE_HEAD_GAZE:
            return "gaze";
        case REACTIVE_HEAD_UNAVAILABLE:
        default:
            return "unavailable";
    }
}

/** Clamp an integer to [0, 255]. */
static inline uint8_t clamp_u8(int32_t v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return (uint8_t)v;
}

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return (int16_t)v;
}

/**
 * @brief Apply skid-steer mix for a drive goal.
 *
 * NOTE: motor_set_individual() takes (left_speed, right_speed, left_dir,
 * right_dir).  Direction 1 = forward.
 *
 * Shortcut noted: we ignore drive.distance_cm — the robot drives until the
 * planner emits a new goal.  Closed-loop distance requires odometry, which is
 * not available in this phase.
 */
static void execute_drive(const goal_t *goal)
{
    int16_t heading = head_aim_body_heading(goal->params.drive.heading_deg, goal->head_pan_deg);
    uint8_t speed_pct = goal->params.drive.speed_pct;

    int32_t base = (int32_t)speed_pct * 255 / 100;
    int32_t bias = (int32_t)abs(heading) * base / 180;

    int32_t left, right;
    if (heading > 0) {
        /* Turn right: slow down right wheel */
        right = base - bias;
        left = base;
    } else if (heading < 0) {
        /* Turn left: slow down left wheel */
        left = base - bias;
        right = base;
    } else {
        left = base;
        right = base;
    }

    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

/**
 * @brief Wheel-only proportional visual servo for a track goal.
 *
 * Used when the head is not the executor's to aim. Kp = TRACK_KP (default
 * 0.15).  With a 500-unit half-range and base speed of 128 (≈50 %), the
 * maximum turn bias is ±75 counts — a 59 % differential that produces a visible
 * arc without spinning in place.
 */
static void execute_track_wheels_only(const goal_t *goal)
{
    uint16_t cx = (goal->params.track.xmin + goal->params.track.xmax) / 2u;
    int32_t error = (int32_t)cx - 500;

    float turn = TRACK_KP * (float)error;
    int32_t base = (int32_t)goal->params.track.max_speed_pct * 255 / 100;

    /* Positive error (target right of centre) → right wheel slower */
    int32_t left = base + (int32_t)turn;
    int32_t right = base - (int32_t)turn;

    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

/**
 * @brief Track goal with the head leading.
 *
 * The head points at the target, so the wheels do not steer: they creep
 * straight at max_speed_pct (0 holds position and only looks), or turn in place
 * when head_aim says the bearing has outrun the head's travel. The turn runs at
 * a fixed speed because the yaw estimate that ends it is only valid at one.
 */
static void execute_track_head_led(const goal_t *goal, head_body_t body)
{
    switch (body) {
        case HEAD_BODY_TURN_CW:
            motor_rotate_cw(HEAD_BODY_TURN_SPEED);
            break;
        case HEAD_BODY_TURN_CCW:
            motor_rotate_ccw(HEAD_BODY_TURN_SPEED);
            break;
        case HEAD_BODY_HOLD:
        default: {
            const uint8_t base = clamp_u8((int32_t)goal->params.track.max_speed_pct * 255 / 100);
            if (base == 0) {
                motor_stop();
            } else {
                motor_set_individual(base, base, 1, 1);
            }
            break;
        }
    }
}

/**
 * @brief Apply in-place rotation.
 *
 * Direction is determined by the sign of angle_deg.  Magnitude is ignored —
 * we have no IMU to close the angle loop.  The planner must emit a stop/drive
 * goal when the desired heading is reached (e.g. by inspecting the next camera
 * frame).
 *
 * Rotation speed is fixed at ~60 % to give the planner time to react.
 */
static void execute_rotate(const goal_t *goal)
{
    const uint8_t rot_speed = 153; /* ~60 % of 255 */
    if (goal->params.rotate.angle_deg >= 0) {
        motor_rotate_cw(rot_speed);
    } else {
        motor_rotate_ccw(rot_speed);
    }
}

/** Apply a manual override command. */
static void execute_manual(reactive_manual_cmd_t cmd, uint8_t speed)
{
    const uint8_t rot_speed = 153; /* ~60 % of 255, matching execute_rotate */
    switch (cmd) {
        case REACTIVE_MANUAL_FORWARD:
            motor_move_forward(speed);
            break;
        case REACTIVE_MANUAL_BACKWARD:
            motor_move_backward(speed);
            break;
        case REACTIVE_MANUAL_LEFT:
            motor_turn_left(speed);
            break;
        case REACTIVE_MANUAL_RIGHT:
            motor_turn_right(speed);
            break;
        case REACTIVE_MANUAL_ROTATE_CW:
            motor_rotate_cw(rot_speed);
            break;
        case REACTIVE_MANUAL_ROTATE_CCW:
            motor_rotate_ccw(rot_speed);
            break;
        case REACTIVE_MANUAL_STOP:
        default:
            motor_stop();
            break;
    }
}

/**
 * @brief Write a head command, one axis at a time, skipping a released axis.
 *
 * @return true when every write attempted succeeded.
 */
static bool write_head(head_pose_t cmd, head_pose_t now, bool full)
{
    bool ok = true;
    if (full || cmd.pan_deg != now.pan_deg) {
        ok = (servo_set_angle(SERVO_PAN, cmd.pan_deg) == ESP_OK) && ok;
    }
    if (servo_is_enabled(SERVO_TILT) && (full || cmd.tilt_deg != now.tilt_deg)) {
        ok = (servo_set_angle(SERVO_TILT, cmd.tilt_deg) == ESP_OK) && ok;
    }
    return ok;
}

/**
 * @brief One head step: aim, gaze, centre, hold a lease, or keep hands off.
 *
 * @param body_can_turn  false when the wheels are held by the reflex or a
 *                       manual lease, so the yaw estimate does not advance.
 * @param[out] body      Wheel instruction for a head-led track goal.
 * @return What was done with the head. Anything but AIM/TURN means a track goal
 *         must be steered with the wheels alone.
 */
static reactive_head_mode_t execute_head(const head_lease_t *lease, const goal_t *goal, bool fresh,
                                         uint32_t goal_seq, bool body_can_turn, uint32_t now_ms,
                                         head_body_t *body)
{
    *body = HEAD_BODY_HOLD;

    if (lease->kind == HEAD_LEASE_EXTERNAL) {
        /* Forget the write shadow too: the other writer moved the servos, so
         * the first write after the lease must not be suppressed. Not settled:
         * that writer is moving the head right now. */
        head_aim_reset(&s_head_aim);
        idle_gaze_restart(&s_gaze);
        s_head_settled = false;
        return REACTIVE_HEAD_LEASE;
    }

    /* Pan is the axis the aiming depends on; `servo off` releases both.
     * Settled: nothing commands the head, so nothing moves it. */
    if (!servo_is_initialized() || !servo_is_enabled(SERVO_PAN)) {
        head_aim_reset(&s_head_aim);
        idle_gaze_restart(&s_gaze);
        s_head_settled = true;
        return REACTIVE_HEAD_UNAVAILABLE;
    }

    servo_position_t pos = {0};
    head_limits_t lim = {0};
    if (servo_get_position(&pos) != ESP_OK ||
        servo_get_limits(SERVO_PAN, &lim.pan_min, &lim.pan_max) != ESP_OK ||
        servo_get_limits(SERVO_TILT, &lim.tilt_min, &lim.tilt_max) != ESP_OK) {
        head_aim_reset(&s_head_aim);
        idle_gaze_restart(&s_gaze);
        s_head_settled = false;
        return REACTIVE_HEAD_UNAVAILABLE;
    }
    const head_pose_t now = {.pan_deg = pos.pan_angle, .tilt_deg = pos.tilt_angle};

    reactive_head_mode_t mode;
    head_pose_t cmd;
    if (lease->kind == HEAD_LEASE_HOLD) {
        s_head_aim.latched = false;
        s_head_aim.body_turning = false;
        cmd.pan_deg = clamp_i16(lease->pan_deg, lim.pan_min, lim.pan_max);
        cmd.tilt_deg = clamp_i16(lease->tilt_deg, lim.tilt_min, lim.tilt_max);
        mode = REACTIVE_HEAD_LEASE;
        idle_gaze_restart(&s_gaze);
    } else if (fresh && goal->kind == GOAL_KIND_TRACK) {
        const head_box_t box = {
            .ymin = goal->params.track.ymin,
            .xmin = goal->params.track.xmin,
            .ymax = goal->params.track.ymax,
            .xmax = goal->params.track.xmax,
        };
        const head_pose_t capture = {.pan_deg = goal->head_pan_deg,
                                     .tilt_deg = goal->head_tilt_deg};
        const head_aim_out_t out =
            head_aim_track(&s_head_aim, goal_seq, &box, capture, now, &lim, body_can_turn);
        cmd = out.cmd;
        *body = out.body;
        mode = (out.body == HEAD_BODY_HOLD) ? REACTIVE_HEAD_AIM : REACTIVE_HEAD_TURN;
        /* The goal pre-empts the gaze; when it goes stale the gaze resumes
         * from centre (ADR-029 §1). */
        idle_gaze_restart(&s_gaze);
    } else if (s_gaze_enabled) {
        const head_pose_t target = idle_gaze_step(&s_gaze, now, &lim, head_face_forward(), now_ms);
        cmd = head_aim_gaze(&s_head_aim, now, target, &lim).cmd;
        mode = REACTIVE_HEAD_GAZE;
    } else {
        cmd = head_aim_centre(&s_head_aim, now, &lim).cmd;
        mode = REACTIVE_HEAD_CENTRE;
        idle_gaze_restart(&s_gaze);
    }

    /* A released tilt keeps its last angle, so an unwritable difference cannot
     * make every tick look like a change. */
    if (!servo_is_enabled(SERVO_TILT)) {
        cmd.tilt_deg = now.tilt_deg;
    }

    head_aim_note_motion(&s_head_aim, cmd, now, now_ms);
    s_head_settled = head_aim_settled(&s_head_aim, now_ms);

    if (head_aim_write_due(&s_head_aim, cmd, now, now_ms)) {
        const bool full =
            !s_head_aim.written || (cmd.pan_deg == now.pan_deg && cmd.tilt_deg == now.tilt_deg);
        head_aim_note_write(&s_head_aim, cmd, now_ms, write_head(cmd, now, full));
    }
    return mode;
}

/** Motors for the planner's goal, once the reflex and manual lease have passed. */
static void execute_goal(const goal_t *goal, bool fresh, reactive_head_mode_t head_mode,
                         head_body_t body)
{
    if (!fresh || goal->kind == GOAL_KIND_STOP || goal->kind == GOAL_KIND_NONE) {
        motor_stop();
        return;
    }
    switch (goal->kind) {
        case GOAL_KIND_DRIVE:
            execute_drive(goal);
            break;
        case GOAL_KIND_TRACK:
            if (head_mode == REACTIVE_HEAD_AIM || head_mode == REACTIVE_HEAD_TURN) {
                execute_track_head_led(goal, body);
            } else {
                execute_track_wheels_only(goal);
            }
            break;
        case GOAL_KIND_ROTATE:
            execute_rotate(goal);
            break;
        default:
            motor_stop();
            break;
    }
}

/* -------------------------------------------------------------------------
 * Rangefinders — front (reflex) and rear (telemetry)
 * -------------------------------------------------------------------------
 * A read completes over several ticks, so only READY and ERROR results move
 * anything here; PENDING ticks leave every field as it was.
 */

typedef struct {
    uint16_t buf[ULTRASONIC_SMOOTH_N]; /**< Front readings, running mean.   */
    uint8_t idx;
    bool full;
    uint16_t front_cm;      /**< Smoothed front distance; releases the reflex. */
    uint16_t front_last_cm; /**< Latest front reading (error = max); trips it. */
    uint16_t rear_cm;       /**< Latest rear reading, or ULTRASONIC_DIST_ERROR.  */
    uint32_t streak[ULTRASONIC_COUNT];
    bool failed[ULTRASONIC_COUNT];
} rangers_t;

/** Executor-private. Touched only by the executor tick and init. */
static rangers_t s_rangers;

static void rangers_reset(rangers_t *r)
{
    memset(r, 0, sizeof(*r));
    /* Before the first reading the path reads as clear, the same value a
     * failed reading feeds the filter. */
    r->front_cm = ULTRASONIC_MAX_RANGE_CM;
    r->front_last_cm = ULTRASONIC_MAX_RANGE_CM;
    r->rear_cm = ULTRASONIC_DIST_ERROR;
}

static uint16_t rangers_smooth(rangers_t *r, uint16_t raw)
{
    /* Treat error values as max range so a single bad reading does not
     * erroneously trigger the reflex. */
    const uint16_t clamped = (raw == ULTRASONIC_DIST_ERROR) ? ULTRASONIC_MAX_RANGE_CM : raw;

    r->buf[r->idx] = clamped;
    r->idx = (uint8_t)((r->idx + 1u) % ULTRASONIC_SMOOTH_N);
    if (r->idx == 0u) {
        r->full = true;
    }
    const uint8_t n = r->full ? (uint8_t)ULTRASONIC_SMOOTH_N : r->idx;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += r->buf[i];
    }
    return (uint16_t)(sum / n);
}

/**
 * Whether the obstacle reflex holds the wheels.
 *
 * Trips on the latest front reading alone, so the reflex fires on the first
 * reading that sees the obstacle instead of waiting ULTRASONIC_SMOOTH_N
 * readings for the mean to follow. Releases only when the mean agrees, so a
 * close reading still in the filter keeps the brake on. A failed read counts as
 * max range in both, so it can neither trip nor hold the reflex.
 */
static bool rangers_reflex(const rangers_t *r)
{
    return r->front_last_cm < STOP_THRESHOLD_CM || r->front_cm < STOP_THRESHOLD_CM;
}

/**
 * Poll both sensors once.
 *
 * A single failed read is deliberately treated as "max range" by the filter,
 * so one bad reading cannot trip the reflex. That hides *sustained* failure,
 * though: a dead sensor then reads as a permanently clear path. The streak
 * makes it visible in telemetry.
 *
 * @return Bitmask, bit (1 << id) set for each sensor whose failed flag changed
 *         on this call, so the caller can log the transition once.
 */
static uint8_t rangers_tick(rangers_t *r, uint32_t now_ms)
{
    uint8_t changed = 0;
    for (int i = 0; i < (int)ULTRASONIC_COUNT; i++) {
        uint16_t cm = ULTRASONIC_DIST_ERROR;
        const ultrasonic_result_t res = ultrasonic_poll((ultrasonic_id_t)i, now_ms, &cm);
        if (res == ULTRASONIC_PENDING) {
            continue;
        }
        const bool ok = (res == ULTRASONIC_READY);
        if (i == ULTRASONIC_FRONT) {
            r->front_cm = rangers_smooth(r, ok ? cm : ULTRASONIC_DIST_ERROR);
            r->front_last_cm = ok ? cm : ULTRASONIC_MAX_RANGE_CM;
        } else {
            r->rear_cm = ok ? cm : ULTRASONIC_DIST_ERROR;
        }

        const bool was_failed = r->failed[i];
        if (ok) {
            r->streak[i] = 0;
            r->failed[i] = false;
        } else if (r->streak[i] < ULTRASONIC_FAIL_STREAK) {
            r->streak[i]++;
            if (r->streak[i] == ULTRASONIC_FAIL_STREAK) {
                r->failed[i] = true;
            }
        }
        if (r->failed[i] != was_failed) {
            changed |= (uint8_t)(1u << i);
        }
    }
    return changed;
}

/* -------------------------------------------------------------------------
 * Cliff reflex — front and rear floor sensors (cliff_core.h)
 * ---------------------------------------------------------------------- */

/** Executor-private. Touched only by the executor tick and init. */
static cliff_t s_cliff;
/** Whether the last tick ran the reflex: enabled and the sensors present. */
static bool s_cliff_armed = false;
/** `cliff on|off`; a bool any task may write, read once per tick. On at boot. */
static volatile bool s_cliff_enabled = true;

void reactive_controller_set_cliff(bool on)
{
    s_cliff_enabled = on;
}

bool reactive_controller_cliff_enabled(void)
{
    return s_cliff_enabled;
}

/**
 * Poll the floor sensors and run one reflex step.
 *
 * Arming (enable, or sensors appearing) resets the state, which starts as "no
 * floor" on both sides: the robot holds still until the floor is confirmed, and
 * arming is never mistaken for an edge.
 */
static cliff_action_t cliff_tick(uint32_t now_ms, bool obstacle_ahead)
{
    const bool armed = s_cliff_enabled && cliff_sensor_available();
    if (!armed) {
        s_cliff_armed = false;
        return CLIFF_ACT_NONE;
    }
    if (!s_cliff_armed) {
        const uint32_t trips = s_cliff.trips; /* a count since boot, kept over re-arming */
        cliff_reset(&s_cliff);
        s_cliff.trips = trips;
        s_cliff_armed = true;
    }
    cliff_sample_t sample;
    cliff_sensor_read(&sample);
    return cliff_step(&s_cliff, &sample, now_ms, obstacle_ahead);
}

/** Whether the floor sensors allow a motion; always true while disarmed. */
static bool cliff_allows(cliff_motion_t m)
{
    return !s_cliff_armed || cliff_permits(&s_cliff, m);
}

/** The direction a manual command moves the robot. Turns are forward arcs. */
static cliff_motion_t manual_motion(reactive_manual_cmd_t cmd)
{
    switch (cmd) {
        case REACTIVE_MANUAL_FORWARD:
        case REACTIVE_MANUAL_LEFT:
        case REACTIVE_MANUAL_RIGHT:
            return CLIFF_MOTION_FORWARD;
        case REACTIVE_MANUAL_BACKWARD:
            return CLIFF_MOTION_BACKWARD;
        case REACTIVE_MANUAL_ROTATE_CW:
        case REACTIVE_MANUAL_ROTATE_CCW:
            return CLIFF_MOTION_ROTATE;
        case REACTIVE_MANUAL_STOP:
        default:
            return CLIFF_MOTION_NONE;
    }
}

/** The direction execute_goal() would move the robot, mirroring its branches. */
static cliff_motion_t goal_motion(const goal_t *goal, bool fresh, reactive_head_mode_t head_mode,
                                  head_body_t body)
{
    if (!fresh) {
        return CLIFF_MOTION_NONE;
    }
    switch (goal->kind) {
        case GOAL_KIND_DRIVE:
            return (goal->params.drive.speed_pct == 0) ? CLIFF_MOTION_NONE : CLIFF_MOTION_FORWARD;
        case GOAL_KIND_ROTATE:
            return CLIFF_MOTION_ROTATE;
        case GOAL_KIND_TRACK:
            if (head_mode == REACTIVE_HEAD_AIM || head_mode == REACTIVE_HEAD_TURN) {
                if (body == HEAD_BODY_TURN_CW || body == HEAD_BODY_TURN_CCW) {
                    return CLIFF_MOTION_ROTATE;
                }
                return (goal->params.track.max_speed_pct * 255 / 100 == 0) ? CLIFF_MOTION_NONE
                                                                           : CLIFF_MOTION_FORWARD;
            }
            return CLIFF_MOTION_FORWARD;
        default:
            return CLIFF_MOTION_NONE;
    }
}

/** What the wheel layers above the planner asked for this tick. */
typedef struct {
    bool held;            /**< Pre-OTA hold.                       */
    cliff_action_t cliff; /**< The cliff reflex's own action.      */
    bool reflex;          /**< Obstacle reflex.                    */
    bool manual_active;   /**< A manual lease is live.             */
    reactive_manual_cmd_t mcmd;
    uint8_t mspeed;
} wheel_layers_t;

/**
 * The wheels for one tick, highest layer first: the OTA hold, the cliff
 * reflex, the obstacle reflex, the manual lease, the planner's goal. A manual
 * or goal motion the floor sensors refuse is replaced by a brake.
 *
 * @return true when the floor sensors refused a requested motion.
 */
static bool drive_wheels(const wheel_layers_t *w, const goal_t *goal, bool fresh,
                         reactive_head_mode_t head_mode, head_body_t body)
{
    if (w->held) {
        /* Before everything else, the reflexes included: the robot is parked
         * for an OTA install and must not move again until the hold is
         * released or the board reboots. Coast, so the outputs are low when
         * the PCA9685 rides through the reset. */
        motor_stop();
        return false;
    }
    switch (w->cliff) {
        case CLIFF_ACT_BRAKE:
            motor_brake();
            return false;
        case CLIFF_ACT_REVERSE:
            motor_move_backward(CLIFF_BACKOFF_SPEED);
            return false;
        case CLIFF_ACT_FORWARD:
            motor_move_forward(CLIFF_BACKOFF_SPEED);
            return false;
        case CLIFF_ACT_NONE:
        default:
            break;
    }
    if (w->reflex) {
        /* Brake, not coast. Stopping distance matters here, and motor_stop()
         * leaves the outputs high-impedance so the robot rolls into whatever
         * tripped the reflex. */
        motor_brake();
        return false;
    }
    const cliff_motion_t wanted =
        w->manual_active ? manual_motion(w->mcmd) : goal_motion(goal, fresh, head_mode, body);
    if (!cliff_allows(wanted)) {
        /* Toward an edge: brake, for the same reason as the reflexes. */
        motor_brake();
        return true;
    }
    if (w->manual_active) {
        execute_manual(w->mcmd, w->mspeed);
    } else {
        execute_goal(goal, fresh, head_mode, body);
    }
    return false;
}

/** Copy the cliff state into telemetry; caller holds whatever lock it needs. */
static void cliff_telemetry(reactive_telemetry_t *t, cliff_action_t act, bool refused)
{
    t->cliff_armed = s_cliff_armed;
    t->cliff_front = s_cliff_armed && s_cliff.cliff[CLIFF_FRONT];
    t->cliff_rear = s_cliff_armed && s_cliff.cliff[CLIFF_REAR];
    t->cliff_active = s_cliff_armed && (act != CLIFF_ACT_NONE || refused);
    t->cliff_trips = s_cliff.trips;
}

#ifndef REACTIVE_CONTROLLER_HOST_TEST

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sound_clips.h"
#include "voice_turn.h"

static const char *TAG = "reactive_ctrl";

/* The reflex horn's cooldown state; only the executor task touches it. */
static uint32_t s_reflex_honk_ms = 0;
static bool s_reflex_honked = false;

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;
/* volatile: written by reactive_controller_stop() from another task and polled
 * as the executor loop condition, so the compiler must not cache it. */
static volatile bool s_running = false;

/* Telemetry — written by executor task, read by any task under spinlock. */
static portMUX_TYPE s_telem_mux = portMUX_INITIALIZER_UNLOCKED;
static reactive_telemetry_t s_telemetry = {
    .distance_cm = ULTRASONIC_DIST_ERROR,
    .reflex_active = false,
    .loop_hz = 0,
    .stack_high_water = 0,
    .manual_active = false,
    .sensor_failed = false,
    .head_mode = REACTIVE_HEAD_UNAVAILABLE,
    .distance_rear_cm = ULTRASONIC_DIST_ERROR,
    .rear_failed = false,
};

/* Manual override lease — written by any task, read by the executor. */
static portMUX_TYPE s_manual_mux = portMUX_INITIALIZER_UNLOCKED;
static reactive_manual_cmd_t s_manual_cmd = REACTIVE_MANUAL_STOP;
static uint8_t s_manual_speed = 0;
static int64_t s_manual_expiry_us = 0;

/* Head lease — written by any task, read by the executor. */
static portMUX_TYPE s_head_mux = portMUX_INITIALIZER_UNLOCKED;
static head_lease_t s_head_lease = {.kind = HEAD_LEASE_NONE};
static int64_t s_head_expiry_us = 0;

/* While a voice turn listens the head faces forward: one microphone gives no
 * direction of arrival, and forward is the best guess at where the speaker is
 * (ADR-029 §1). voice_turn_is_busy() reads one bool. */
static bool head_face_forward(void)
{
    return voice_turn_is_busy();
}

static uint32_t gaze_rng(void *ctx)
{
    (void)ctx;
    return esp_random();
}

/* -------------------------------------------------------------------------
 * Executor task
 * ---------------------------------------------------------------------- */

static void reactive_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Reactive executor started (Core %d, %u Hz target)", xPortGetCoreID(),
             1000U / REACTIVE_LOOP_PERIOD_MS);

    TickType_t last_wake = xTaskGetTickCount();

    bool reflex_was_active = false;
    reactive_head_mode_t head_mode_was = REACTIVE_HEAD_UNAVAILABLE;

    /* Hz measurement state */
    uint32_t loop_count = 0;
    int64_t hz_window_start = esp_timer_get_time();

    /* Stack HWM report interval */
    int64_t hwm_last_report = esp_timer_get_time();

    while (s_running) {
        /* ---- 1. Rangefinders ----
         * Non-blocking: each poll is at most one short I2C transaction, and a
         * reading completes over several ticks (ultrasonic_core.h). */
        const uint8_t changed = rangers_tick(&s_rangers, (uint32_t)(esp_timer_get_time() / 1000));
        for (int i = 0; i < (int)ULTRASONIC_COUNT; i++) {
            if ((changed & (1u << i)) == 0u) {
                continue;
            }
            if (s_rangers.failed[i]) {
                ESP_LOGW(TAG, "%s rangefinder has failed %u reads in a row%s",
                         ultrasonic_name((ultrasonic_id_t)i), (unsigned)ULTRASONIC_FAIL_STREAK,
                         (i == ULTRASONIC_FRONT) ? " — obstacle reflex is blind" : "");
            } else {
                ESP_LOGI(TAG, "%s rangefinder recovered", ultrasonic_name((ultrasonic_id_t)i));
            }
        }
        const uint16_t dist = s_rangers.front_cm;

        /* ---- 2. Obstacle reflex ---- */
        const bool reflex = rangers_reflex(&s_rangers);
        const bool reflex_tripped = reflex && !reflex_was_active;

        if (reflex_tripped) {
            ESP_LOGW(TAG, "Obstacle reflex ACTIVE — reading %u cm, mean %u cm (threshold %u cm)",
                     s_rangers.front_last_cm, dist, STOP_THRESHOLD_CM);
        } else if (!reflex && reflex_was_active) {
            ESP_LOGI(TAG, "Obstacle reflex CLEARED — reading %u cm, mean %u cm",
                     s_rangers.front_last_cm, dist);
        }
        reflex_was_active = reflex;

        const int64_t now_us = esp_timer_get_time();

        /* ---- 2b. Cliff reflex ---- */
        const uint32_t cliff_trips_before = s_cliff.trips;
        const bool cliff_was_armed = s_cliff_armed;
        const cliff_action_t cliff_act = cliff_tick((uint32_t)(now_us / 1000), reflex);
        const bool cliff_tripped = s_cliff.trips != cliff_trips_before;
        if (cliff_tripped) {
            ESP_LOGW(TAG, "Cliff reflex ACTIVE — %s sensor lost the floor (front %s, rear %s)",
                     cliff_name(s_cliff.trigger), s_cliff.cliff[CLIFF_FRONT] ? "no floor" : "floor",
                     s_cliff.cliff[CLIFF_REAR] ? "no floor" : "floor");
        }
        if (s_cliff_armed != cliff_was_armed) {
            ESP_LOGI(TAG, "Cliff reflex %s", s_cliff_armed ? "armed" : "disarmed");
        }

        /* ---- 3. Manual override lease (console / bench) ---- */
        reactive_manual_cmd_t mcmd;
        uint8_t mspeed;
        portENTER_CRITICAL(&s_manual_mux);
        const bool manual_active = (s_manual_expiry_us > now_us);
        mcmd = s_manual_cmd;
        mspeed = s_manual_speed;
        portEXIT_CRITICAL(&s_manual_mux);

        /* ---- 4. Goal ---- */
        goal_t goal = {0};
        bool fresh = false;
        uint32_t goal_seq = 0;
        if (goal_state_read_seq(&goal, &fresh, &goal_seq) != ESP_OK) {
            goal.kind = GOAL_KIND_STOP;
            fresh = false;
        }

        /* ---- 4b. Polite horn when the reflex stops a drive ----
         * Only when the reflex is holding back motion somebody asked for, and
         * not more than once per SOUND_REFLEX_COOLDOWN_MS. A request flag, not
         * speech_queue_post(): this task has under 1 kB of stack headroom. */
        if (reflex_tripped || cliff_tripped) {
            const bool blocking = manual_active || (fresh && goal.kind != GOAL_KIND_STOP);
            const uint32_t now_ms = (uint32_t)(now_us / 1000);
            if (sound_clips_reflex_due(now_ms, s_reflex_honk_ms, s_reflex_honked, blocking)) {
                sound_clips_request(SOUND_CLIP_HORN);
                s_reflex_honk_ms = now_ms;
                s_reflex_honked = true;
            }
        }

        /* ---- 5. Head ----
         * Before the wheels, because a head-led track goal decides whether the
         * body turns. Runs through the reflex and manual lease, which only hold
         * the wheels; body_can_turn tells it so. */
        head_lease_t lease;
        portENTER_CRITICAL(&s_head_mux);
        if (s_head_lease.kind != HEAD_LEASE_NONE && s_head_expiry_us <= now_us) {
            s_head_lease.kind = HEAD_LEASE_NONE;
        }
        lease = s_head_lease;
        portEXIT_CRITICAL(&s_head_mux);

        head_body_t body = HEAD_BODY_HOLD;
        const reactive_head_mode_t head_mode =
            execute_head(&lease, &goal, fresh, goal_seq,
                         !reflex && !manual_active && cliff_act == CLIFF_ACT_NONE &&
                             cliff_allows(CLIFF_MOTION_ROTATE),
                         (uint32_t)(now_us / 1000), &body);
        if (head_mode != head_mode_was) {
            ESP_LOGD(TAG, "Head: %s -> %s", reactive_head_mode_name(head_mode_was),
                     reactive_head_mode_name(head_mode));
            head_mode_was = head_mode;
        }

        /* ---- 6. Wheels ----
         * Reflex first, then the manual lease: a manual command must not be able
         * to drive into an obstacle, and the lease expires on its own so the
         * planner resumes without an explicit hand-back. */
        const bool held = s_hold;
        const wheel_layers_t layers = {
            .held = held,
            .cliff = cliff_act,
            .reflex = reflex,
            .manual_active = manual_active,
            .mcmd = mcmd,
            .mspeed = mspeed,
        };
        const bool cliff_refused = drive_wheels(&layers, &goal, fresh, head_mode, body);

        /* ---- 7. Telemetry ---- */
        loop_count++;
        int64_t now = esp_timer_get_time();

        uint32_t measured_hz = 0;
        if (now - hz_window_start >= 1000000LL) {
            measured_hz = loop_count;
            loop_count = 0;
            hz_window_start = now;
        }

        uint32_t hwm = 0;
        if (now - hwm_last_report >= 5000000LL) {
            hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Stack high-water-mark: %lu bytes remaining", (unsigned long)hwm);
            hwm_last_report = now;
        } else {
            hwm = uxTaskGetStackHighWaterMark(NULL);
        }

        portENTER_CRITICAL(&s_telem_mux);
        s_telemetry.distance_cm = dist;
        if (reflex && !s_telemetry.reflex_active) {
            s_telemetry.reflex_trips++;
        }
        s_telemetry.reflex_active = reflex;
        if (measured_hz > 0) {
            s_telemetry.loop_hz = measured_hz;
        }
        s_telemetry.stack_high_water = hwm;
        s_telemetry.manual_active = manual_active && !reflex;
        s_telemetry.sensor_failed = s_rangers.failed[ULTRASONIC_FRONT];
        s_telemetry.head_mode = head_mode;
        s_telemetry.distance_rear_cm = s_rangers.rear_cm;
        s_telemetry.rear_failed = s_rangers.failed[ULTRASONIC_REAR];
        s_telemetry.head_settled = s_head_settled;
        s_telemetry.held = held;
        cliff_telemetry(&s_telemetry, cliff_act, cliff_refused);
        portEXIT_CRITICAL(&s_telem_mux);

        /* ---- 8. Wait for next period ----
         * xTaskDelayUntil() returns pdFALSE without blocking when the loop body
         * already overran the period — e.g. an I2C transaction that runs to its
         * timeout on a stuck bus (the guard was written for the GPIO-mode
         * rangefinder's 40 ms busy-wait, which is gone). A
         * non-blocking return means this Core-0 task never yields, starving
         * IDLE0 and tripping the task watchdog. Guarantee at least a one-tick
         * yield when we have fallen behind, and resync so we don't then fire a
         * catch-up burst of zero-delay iterations. The healthy path (body fits
         * the period) still blocks normally, so the 30 Hz cadence is unchanged. */
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REACTIVE_LOOP_PERIOD_MS)) == pdFALSE) {
            vTaskDelay(1);
            last_wake = xTaskGetTickCount();
        }
    }

    motor_stop();
    ESP_LOGI(TAG, "Reactive executor stopping");
    /* Publish our own exit before deleting: reactive_controller_stop() uses a
     * NULL handle to mean "already gone", and clearing it here is what makes
     * that check race-free — otherwise stop() could vTaskDelete() a handle the
     * scheduler has already reclaimed. */
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t reactive_controller_init(void)
{
    if (s_running) {
        return ESP_OK; /* idempotent */
    }

    /* Not fatal: without the executor nothing drives the motors at all, so a
     * missing or unreachable rangefinder must cost the reflex, not the robot.
     * Every poll then fails, and telemetry reports both sensors as failed. */
    const esp_err_t ret = ultrasonic_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ultrasonic_init failed (%s) — running without rangefinders",
                 esp_err_to_name(ret));
    }

    rangers_reset(&s_rangers);
    head_aim_reset(&s_head_aim);
    idle_gaze_init(&s_gaze, gaze_rng, NULL);
    s_head_settled = false;
    s_hold = false;

    s_running = true;

    BaseType_t created =
        xTaskCreatePinnedToCore(reactive_task, "reactive_ctrl", REACTIVE_TASK_STACK_SIZE, NULL,
                                REACTIVE_TASK_PRIORITY, &s_task_handle, REACTIVE_TASK_CORE);

    if (created != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "Failed to create reactive controller task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Reactive controller started (stack=%u, prio=%u, core=%u)",
             REACTIVE_TASK_STACK_SIZE, REACTIVE_TASK_PRIORITY, REACTIVE_TASK_CORE);
    return ESP_OK;
}

esp_err_t reactive_controller_stop(void)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running = false;

    /* Give the task time to exit its loop and call vTaskDelete(NULL). */
    vTaskDelay(pdMS_TO_TICKS(REACTIVE_LOOP_PERIOD_MS * 3));

    /* If the task handle is still valid (task did not self-delete yet), force
     * delete it.  This is a last resort; normally the task exits cleanly.
     * Snapshot-and-clear in one step so we cannot delete a handle the task
     * already retired between the test and the call. */
    TaskHandle_t handle = s_task_handle;
    s_task_handle = NULL;
    if (handle != NULL) {
        vTaskDelete(handle);
    }

    motor_stop();
    ESP_LOGI(TAG, "Reactive controller stopped");
    return ESP_OK;
}

esp_err_t reactive_controller_manual(reactive_manual_cmd_t cmd, uint8_t speed, uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_MANUAL_TTL_MS;
    }

    portENTER_CRITICAL(&s_manual_mux);
    s_manual_cmd = cmd;
    s_manual_speed = speed;
    /* An explicit STOP ends the lease rather than holding the motors stopped
     * for a further TTL — otherwise the planner could not resume for a second
     * after every manual stop. */
    s_manual_expiry_us =
        (cmd == REACTIVE_MANUAL_STOP) ? 0 : esp_timer_get_time() + (int64_t)ttl_ms * 1000;
    portEXIT_CRITICAL(&s_manual_mux);

    if (cmd == REACTIVE_MANUAL_STOP) {
        /* Stop now instead of waiting up to one loop period; the executor will
         * keep it stopped because no goal outlives its TTL by then. */
        motor_stop();
    }
    return ESP_OK;
}

esp_err_t reactive_controller_head_hold(reactive_head_axis_t axis, int16_t angle_deg,
                                        uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_HEAD_HOLD_TTL_MS;
    }
    /* Read outside the spinlock: servo_get_position() is a plain copy, but it
     * is not ours to call with interrupts masked. */
    servo_position_t pos = {0};
    (void)servo_get_position(&pos);
    const int64_t expiry = esp_timer_get_time() + (int64_t)ttl_ms * 1000;

    portENTER_CRITICAL(&s_head_mux);
    if (s_head_lease.kind != HEAD_LEASE_HOLD || s_head_expiry_us <= esp_timer_get_time()) {
        /* A fresh hold starts from where the head is, so the other axis holds. */
        s_head_lease.pan_deg = pos.pan_angle;
        s_head_lease.tilt_deg = pos.tilt_angle;
    }
    s_head_lease.kind = HEAD_LEASE_HOLD;
    if (axis == REACTIVE_HEAD_PAN) {
        s_head_lease.pan_deg = angle_deg;
    } else {
        s_head_lease.tilt_deg = angle_deg;
    }
    s_head_expiry_us = expiry;
    portEXIT_CRITICAL(&s_head_mux);
    return ESP_OK;
}

esp_err_t reactive_controller_head_external(uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_HEAD_HOLD_TTL_MS;
    }
    const int64_t expiry = esp_timer_get_time() + (int64_t)ttl_ms * 1000;
    portENTER_CRITICAL(&s_head_mux);
    s_head_lease.kind = HEAD_LEASE_EXTERNAL;
    s_head_expiry_us = expiry;
    portEXIT_CRITICAL(&s_head_mux);
    return ESP_OK;
}

esp_err_t reactive_controller_head_release(void)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_head_mux);
    s_head_lease.kind = HEAD_LEASE_NONE;
    s_head_expiry_us = 0;
    portEXIT_CRITICAL(&s_head_mux);
    return ESP_OK;
}

esp_err_t reactive_controller_get_telemetry(reactive_telemetry_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_telem_mux);
    *out = s_telemetry;
    portEXIT_CRITICAL(&s_telem_mux);

    return ESP_OK;
}

/* =========================================================================
 * Host-test build
 * =========================================================================
 *
 * The host-test build links against stub versions of goal_state, ultrasonic,
 * motor_controller and servo_controller. Task creation becomes a no-op, and
 * reactive_controller_tick_for_test() runs one executor iteration so tests can
 * drive the shared decision code above one tick at a time. Leases count ticks
 * rather than microseconds so expiry is deterministic.
 * ========================================================================= */

#else /* REACTIVE_CONTROLLER_HOST_TEST */

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Stub telemetry */
static reactive_telemetry_t s_telemetry = {0};
static bool s_running = false;

/* Manual override lease, in ticks; reactive_controller_manual() converts the ms
 * TTL at the same 33 ms period the target executor runs at. */
static reactive_manual_cmd_t s_manual_cmd = REACTIVE_MANUAL_STOP;
static uint8_t s_manual_speed = 0;
static uint32_t s_manual_ticks_left = 0;

/* Head lease, in ticks. */
static head_lease_t s_head_lease = {.kind = HEAD_LEASE_NONE};
static uint32_t s_head_ticks_left = 0;

/* Host clock: one tick = one loop period. */
static uint32_t s_ticks = 0;

/* Voice-turn stand-in and a deterministic random source for the gaze. */
static bool s_test_face_forward = false;
static uint32_t s_test_rng_state = 0x12345678u;

static bool head_face_forward(void)
{
    return s_test_face_forward;
}

static uint32_t gaze_rng(void *ctx)
{
    (void)ctx;
    /* xorshift32: deterministic, and spread enough to visit every pose. */
    uint32_t x = s_test_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_test_rng_state = x;
    return x;
}

void reactive_controller_set_face_forward_for_test(bool face_forward)
{
    s_test_face_forward = face_forward;
}

static uint32_t ms_to_ticks(uint32_t ms)
{
    return (ms + REACTIVE_LOOP_PERIOD_MS - 1) / REACTIVE_LOOP_PERIOD_MS;
}

esp_err_t reactive_controller_manual(reactive_manual_cmd_t cmd, uint8_t speed, uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_MANUAL_TTL_MS;
    }
    s_manual_cmd = cmd;
    s_manual_speed = speed;
    s_manual_ticks_left = (cmd == REACTIVE_MANUAL_STOP) ? 0 : ms_to_ticks(ttl_ms);
    if (cmd == REACTIVE_MANUAL_STOP) {
        motor_stop();
    }
    return ESP_OK;
}

esp_err_t reactive_controller_head_hold(reactive_head_axis_t axis, int16_t angle_deg,
                                        uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_HEAD_HOLD_TTL_MS;
    }
    if (s_head_lease.kind != HEAD_LEASE_HOLD || s_head_ticks_left == 0) {
        servo_position_t pos = {0};
        (void)servo_get_position(&pos);
        s_head_lease.pan_deg = pos.pan_angle;
        s_head_lease.tilt_deg = pos.tilt_angle;
    }
    s_head_lease.kind = HEAD_LEASE_HOLD;
    if (axis == REACTIVE_HEAD_PAN) {
        s_head_lease.pan_deg = angle_deg;
    } else {
        s_head_lease.tilt_deg = angle_deg;
    }
    s_head_ticks_left = ms_to_ticks(ttl_ms);
    return ESP_OK;
}

esp_err_t reactive_controller_head_external(uint32_t ttl_ms)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ttl_ms == 0) {
        ttl_ms = REACTIVE_HEAD_HOLD_TTL_MS;
    }
    s_head_lease.kind = HEAD_LEASE_EXTERNAL;
    s_head_ticks_left = ms_to_ticks(ttl_ms);
    return ESP_OK;
}

esp_err_t reactive_controller_head_release(void)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    s_head_lease.kind = HEAD_LEASE_NONE;
    s_head_ticks_left = 0;
    return ESP_OK;
}

/**
 * @brief Single executor tick — exposed for host tests.
 *
 * Mirrors the loop body of reactive_task() without FreeRTOS timing.  Tests
 * call this directly to exercise one decision cycle with controlled inputs
 * injected via ultrasonic_test_set_distance(), goal_state_write() and the
 * servo stub.
 */
void reactive_controller_tick_for_test(void)
{
    (void)rangers_tick(&s_rangers, s_ticks * REACTIVE_LOOP_PERIOD_MS);
    const uint16_t dist = s_rangers.front_cm;

    const bool reflex = rangers_reflex(&s_rangers);

    const cliff_action_t cliff_act = cliff_tick(s_ticks * REACTIVE_LOOP_PERIOD_MS, reflex);

    const bool manual_active = (s_manual_ticks_left > 0);
    if (manual_active) {
        s_manual_ticks_left--;
    }

    goal_t goal = {0};
    bool fresh = false;
    uint32_t goal_seq = 0;
    if (goal_state_read_seq(&goal, &fresh, &goal_seq) != ESP_OK) {
        goal.kind = GOAL_KIND_STOP;
        fresh = false;
    }

    if (s_head_lease.kind != HEAD_LEASE_NONE) {
        if (s_head_ticks_left == 0) {
            s_head_lease.kind = HEAD_LEASE_NONE;
        } else {
            s_head_ticks_left--;
        }
    }
    const head_lease_t lease = s_head_lease;

    head_body_t body = HEAD_BODY_HOLD;
    const reactive_head_mode_t head_mode =
        execute_head(&lease, &goal, fresh, goal_seq,
                     !reflex && !manual_active && cliff_act == CLIFF_ACT_NONE &&
                         cliff_allows(CLIFF_MOTION_ROTATE),
                     s_ticks * REACTIVE_LOOP_PERIOD_MS, &body);

    const bool held = s_hold;
    const wheel_layers_t layers = {
        .held = held,
        .cliff = cliff_act,
        .reflex = reflex,
        .manual_active = manual_active,
        .mcmd = s_manual_cmd,
        .mspeed = s_manual_speed,
    };
    const bool cliff_refused = drive_wheels(&layers, &goal, fresh, head_mode, body);

    s_telemetry.distance_cm = dist;
    if (reflex && !s_telemetry.reflex_active) {
        s_telemetry.reflex_trips++;
    }
    s_telemetry.reflex_active = reflex;
    s_telemetry.manual_active = manual_active && !reflex;
    s_telemetry.head_mode = head_mode;
    s_telemetry.sensor_failed = s_rangers.failed[ULTRASONIC_FRONT];
    s_telemetry.distance_rear_cm = s_rangers.rear_cm;
    s_telemetry.rear_failed = s_rangers.failed[ULTRASONIC_REAR];
    s_telemetry.head_settled = s_head_settled;
    s_telemetry.held = held;
    cliff_telemetry(&s_telemetry, cliff_act, cliff_refused);
    s_ticks++;
}

esp_err_t reactive_controller_init(void)
{
    if (s_running)
        return ESP_OK;
    rangers_reset(&s_rangers);
    s_manual_ticks_left = 0;
    s_manual_cmd = REACTIVE_MANUAL_STOP;
    s_head_lease.kind = HEAD_LEASE_NONE;
    s_head_ticks_left = 0;
    s_ticks = 0;
    head_aim_reset(&s_head_aim);
    s_test_rng_state = 0x12345678u;
    idle_gaze_init(&s_gaze, gaze_rng, NULL);
    s_gaze_enabled = true; /* on at every boot */
    s_head_settled = false;
    s_hold = false;
    s_cliff_armed = false;
    s_cliff_enabled = true; /* on at every boot */
    memset(&s_cliff, 0, sizeof(s_cliff));
    s_running = true;
    ultrasonic_init();
    return ESP_OK;
}

esp_err_t reactive_controller_stop(void)
{
    if (!s_running)
        return ESP_ERR_INVALID_STATE;
    s_running = false;
    motor_stop();
    return ESP_OK;
}

esp_err_t reactive_controller_get_telemetry(reactive_telemetry_t *out)
{
    if (out == NULL)
        return ESP_ERR_INVALID_ARG;
    *out = s_telemetry;
    return ESP_OK;
}

#endif /* REACTIVE_CONTROLLER_HOST_TEST */
