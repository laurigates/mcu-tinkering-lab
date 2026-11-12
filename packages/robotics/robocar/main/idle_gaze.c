/**
 * @file idle_gaze.c
 * @brief Idle gaze target selection — see idle_gaze.h.
 *
 * Pure C: no ESP-IDF, no FreeRTOS, no I/O.
 */

#include "idle_gaze.h"

#include <string.h>

#include "scene_change.h"

/** The head angles of each dwell pose, from the constants the scene gates
 *  classify by, so the two cannot disagree about where a pose is. */
static head_pose_t pose_angles(uint8_t pose)
{
    head_pose_t p = {0, 0};
    switch (pose) {
        case SCENE_POSE_LEFT:
            p.pan_deg = -SCENE_POSE_SIDE_PAN_DEG;
            break;
        case SCENE_POSE_RIGHT:
            p.pan_deg = SCENE_POSE_SIDE_PAN_DEG;
            break;
        case SCENE_POSE_FLOOR:
            p.tilt_deg = SCENE_POSE_FLOOR_TILT_DEG;
            break;
        case SCENE_POSE_CENTRE:
        default:
            break;
    }
    return p;
}

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return (int16_t)v;
}

static head_pose_t clamp_pose(head_pose_t p, const head_limits_t *lim)
{
    head_pose_t c = {
        .pan_deg = clamp_i16(p.pan_deg, lim->pan_min, lim->pan_max),
        .tilt_deg = clamp_i16(p.tilt_deg, lim->tilt_min, lim->tilt_max),
    };
    return c;
}

/** Whether the limits leave @p pose where the scene gates still recognise it. */
static bool reachable(uint8_t pose, const head_limits_t *lim)
{
    const head_pose_t c = clamp_pose(pose_angles(pose), lim);
    return scene_pose_from_head(c.pan_deg, c.tilt_deg) == (scene_pose_t)pose;
}

static uint32_t draw_dwell_ms(idle_gaze_t *g)
{
    const uint32_t span = GAZE_DWELL_MAX_MS - GAZE_DWELL_MIN_MS + 1U;
    return GAZE_DWELL_MIN_MS + (g->rng(g->rng_ctx) % span);
}

/** A random reachable non-centre pose, or centre when none is reachable. */
static uint8_t draw_away_pose(idle_gaze_t *g, const head_limits_t *lim)
{
    uint8_t options[SCENE_POSES];
    uint32_t n = 0;
    for (uint8_t p = 0; p < SCENE_POSES; ++p) {
        if (p != SCENE_POSE_CENTRE && reachable(p, lim)) {
            options[n++] = p;
        }
    }
    if (n == 0U) {
        return SCENE_POSE_CENTRE;
    }
    return options[g->rng(g->rng_ctx) % n];
}

void idle_gaze_init(idle_gaze_t *g, idle_gaze_rng_t rng, void *rng_ctx)
{
    memset(g, 0, sizeof(*g));
    g->rng = rng;
    g->rng_ctx = rng_ctx;
    g->target = SCENE_POSE_CENTRE;
}

void idle_gaze_restart(idle_gaze_t *g)
{
    g->target = SCENE_POSE_CENTRE;
    g->arrived = false;
}

head_pose_t idle_gaze_step(idle_gaze_t *g, head_pose_t now, const head_limits_t *lim,
                           bool face_forward, uint32_t now_ms)
{
    if (face_forward) {
        /* Held, not dwelt: the dwell restarts when the turn ends, so the head
         * does not swing away the moment the person stops talking. */
        idle_gaze_restart(g);
        return clamp_pose(pose_angles(SCENE_POSE_CENTRE), lim);
    }

    /* The limits can narrow while the head dwells or travels. A target they
     * now cut off goes back to centre rather than parking at the clamp. */
    if (!reachable(g->target, lim)) {
        idle_gaze_restart(g);
    }

    head_pose_t target = clamp_pose(pose_angles(g->target), lim);

    if (!g->arrived) {
        if (now.pan_deg == target.pan_deg && now.tilt_deg == target.tilt_deg) {
            g->arrived = true;
            g->dwell_until_ms = now_ms + draw_dwell_ms(g);
        }
        return target;
    }

    /* Signed difference, so the day-49 wrap is an ordinary elapsed time. */
    if ((int32_t)(now_ms - g->dwell_until_ms) >= 0) {
        g->target =
            (g->target == SCENE_POSE_CENTRE) ? draw_away_pose(g, lim) : (uint8_t)SCENE_POSE_CENTRE;
        g->arrived = false;
        target = clamp_pose(pose_angles(g->target), lim);
        /* Nothing reachable but centre: start the next dwell here at once. */
        if (now.pan_deg == target.pan_deg && now.tilt_deg == target.tilt_deg) {
            g->arrived = true;
            g->dwell_until_ms = now_ms + draw_dwell_ms(g);
        }
    }
    return target;
}
