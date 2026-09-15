/**
 * @file head_aim.c
 * @brief Pan/tilt head aiming — see head_aim.h.
 *
 * Pure C: no ESP-IDF, no FreeRTOS, no I/O. Compiled unmodified into the
 * firmware and the host tests.
 */

#include "head_aim.h"

#include <string.h>

/** Estimated yaw per tick of in-place turning, in degrees. */
static const float k_yaw_per_tick =
    (float)HEAD_BODY_YAW_DEG_PER_S * (float)HEAD_AIM_TICK_MS / 1000.0f;

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return (int16_t)v;
}

static int32_t round_to_int(float v)
{
    return (v >= 0.0f) ? (int32_t)(v + 0.5f) : -(int32_t)(-v + 0.5f);
}

/** Move @p from toward @p to by at most HEAD_SLEW_DEG_PER_TICK. */
static int32_t slew(int16_t from, int16_t to)
{
    int32_t step = (int32_t)to - (int32_t)from;
    if (step > HEAD_SLEW_DEG_PER_TICK)
        step = HEAD_SLEW_DEG_PER_TICK;
    if (step < -HEAD_SLEW_DEG_PER_TICK)
        step = -HEAD_SLEW_DEG_PER_TICK;
    return (int32_t)from + step;
}

/**
 * One slewed step toward @p target, clamped to the limits.
 *
 * The clamp is applied to the *result*, not only to the target: a pose left
 * outside a range that `servo limit` has since narrowed would otherwise slew
 * back in over several ticks, each of them a command outside the limits.
 */
static head_pose_t step_toward(head_pose_t now, int32_t pan_target, int32_t tilt_target,
                               const head_limits_t *lim)
{
    const int16_t pan_goal = clamp_i16(pan_target, lim->pan_min, lim->pan_max);
    const int16_t tilt_goal = clamp_i16(tilt_target, lim->tilt_min, lim->tilt_max);
    head_pose_t cmd = {
        .pan_deg = clamp_i16(slew(now.pan_deg, pan_goal), lim->pan_min, lim->pan_max),
        .tilt_deg = clamp_i16(slew(now.tilt_deg, tilt_goal), lim->tilt_min, lim->tilt_max),
    };
    return cmd;
}

void head_aim_reset(head_aim_t *s)
{
    memset(s, 0, sizeof(*s));
}

head_aim_out_t head_aim_track(head_aim_t *s, uint32_t goal_seq, const head_box_t *box,
                              head_pose_t capture, head_pose_t now, const head_limits_t *lim,
                              bool body_can_turn)
{
    if (!s->latched || s->goal_seq != goal_seq) {
        const int32_t cx = ((int32_t)box->xmin + (int32_t)box->xmax) / 2;
        const int32_t cy = ((int32_t)box->ymin + (int32_t)box->ymax) / 2;
        /* x grows to the right, like pan. y grows DOWN the image while positive
         * tilt is up (servo_exercise's "nod up" is tilt_max), hence 500 - cy. */
        s->bearing_deg =
            (float)capture.pan_deg + (float)(cx - 500) * (float)HEAD_CAMERA_HFOV_DEG / 1000.0f;
        s->tilt_deg =
            (float)capture.tilt_deg + (float)(500 - cy) * (float)HEAD_CAMERA_VFOV_DEG / 1000.0f;
        s->goal_seq = goal_seq;
        s->latched = true;
    }

    /* Engage edges: a fraction of each side's limit, but never inside the
     * deadband, or a limit at zero would start and stop the turn every tick. */
    float right_edge = (float)lim->pan_max * (float)HEAD_PAN_ENGAGE_PCT / 100.0f;
    float left_edge = (float)lim->pan_min * (float)HEAD_PAN_ENGAGE_PCT / 100.0f;
    if (right_edge < (float)HEAD_RECENTRE_DEADBAND_DEG)
        right_edge = (float)HEAD_RECENTRE_DEADBAND_DEG;
    if (left_edge > -(float)HEAD_RECENTRE_DEADBAND_DEG)
        left_edge = -(float)HEAD_RECENTRE_DEADBAND_DEG;

    if (s->bearing_deg > right_edge || s->bearing_deg < left_edge) {
        s->body_turning = true;
    } else if (s->bearing_deg <= (float)HEAD_RECENTRE_DEADBAND_DEG &&
               s->bearing_deg >= -(float)HEAD_RECENTRE_DEADBAND_DEG) {
        s->body_turning = false;
    }

    head_aim_out_t out = {.body = HEAD_BODY_HOLD};
    if (s->body_turning && body_can_turn) {
        if (s->bearing_deg > 0.0f) {
            out.body = HEAD_BODY_TURN_CW;
            s->bearing_deg -= k_yaw_per_tick;
            if (s->bearing_deg < 0.0f)
                s->bearing_deg = 0.0f;
        } else {
            out.body = HEAD_BODY_TURN_CCW;
            s->bearing_deg += k_yaw_per_tick;
            if (s->bearing_deg > 0.0f)
                s->bearing_deg = 0.0f;
        }
    }

    out.cmd = step_toward(now, round_to_int(s->bearing_deg), round_to_int(s->tilt_deg), lim);
    return out;
}

head_aim_out_t head_aim_centre(head_aim_t *s, head_pose_t now, const head_limits_t *lim)
{
    s->latched = false;
    s->body_turning = false;
    head_aim_out_t out = {
        .cmd = step_toward(now, 0, 0, lim),
        .body = HEAD_BODY_HOLD,
    };
    return out;
}

bool head_aim_write_due(const head_aim_t *s, head_pose_t cmd, head_pose_t now, uint32_t now_ms)
{
    if (!s->written)
        return true;
    if (cmd.pan_deg != now.pan_deg || cmd.tilt_deg != now.tilt_deg)
        return true;
    if (cmd.pan_deg != s->last.pan_deg || cmd.tilt_deg != s->last.tilt_deg)
        return true;
    return (uint32_t)(now_ms - s->written_ms) >= HEAD_REFRESH_INTERVAL_MS;
}

void head_aim_note_write(head_aim_t *s, head_pose_t cmd, uint32_t now_ms, bool ok)
{
    if (ok) {
        s->last = cmd;
        s->written_ms = now_ms;
        s->written = true;
    } else {
        s->written = false;
    }
}

int16_t head_aim_body_heading(int16_t heading_deg, int16_t capture_pan_deg)
{
    int32_t h = (int32_t)heading_deg + (int32_t)capture_pan_deg;
    while (h > 180)
        h -= 360;
    while (h < -180)
        h += 360;
    return (int16_t)h;
}
