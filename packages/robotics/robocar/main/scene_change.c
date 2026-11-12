/**
 * @file scene_change.c
 * @brief Scene-change detection. See the header for the design and the reason
 *        the reference frame is "when the robot last spoke", not "last frame".
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_scene_change.c
 * builds it on the host with no shims.
 */

#include "scene_change.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Fingerprint construction                                                     */
/* -------------------------------------------------------------------------- */

void scene_fingerprint_from_luma(const uint8_t *luma, int w, int h, scene_fingerprint_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (luma == NULL || w <= 0 || h <= 0) {
        return; /* leaves valid = false */
    }

    /* Frame mean first: every block is stored relative to it, so a uniform
     * brightness shift (the sensor's AGC breathing on a static scene) cancels
     * out instead of reading as motion. */
    uint32_t frame_sum = 0;
    for (int i = 0; i < w * h; ++i) {
        frame_sum += luma[i];
    }
    const int frame_mean = (int)(frame_sum / (uint32_t)(w * h));

    for (int by = 0; by < SCENE_BLOCKS_Y; ++by) {
        /* Proportional bounds rather than a fixed block size: the grid need not
         * divide the plane evenly, and an off-by-one here would quietly compare
         * different regions of the two frames. */
        const int y0 = (by * h) / SCENE_BLOCKS_Y;
        const int y1 = ((by + 1) * h) / SCENE_BLOCKS_Y;

        for (int bx = 0; bx < SCENE_BLOCKS_X; ++bx) {
            const int x0 = (bx * w) / SCENE_BLOCKS_X;
            const int x1 = ((bx + 1) * w) / SCENE_BLOCKS_X;

            uint32_t sum = 0;
            uint32_t n = 0;
            for (int y = y0; y < y1; ++y) {
                for (int x = x0; x < x1; ++x) {
                    sum += luma[(y * w) + x];
                    ++n;
                }
            }

            int dev = 0;
            if (n > 0u) {
                dev = (int)(sum / n) - frame_mean;
            }
            if (dev > 127) {
                dev = 127;
            } else if (dev < -128) {
                dev = -128;
            }
            out->block[(by * SCENE_BLOCKS_X) + bx] = (int8_t)dev;
        }
    }

    out->valid = true;
}

unsigned scene_fingerprint_distance(const scene_fingerprint_t *a, const scene_fingerprint_t *b)
{
    if (a == NULL || b == NULL || !a->valid || !b->valid) {
        return 0;
    }

    uint32_t total = 0;
    for (int i = 0; i < SCENE_BLOCKS; ++i) {
        const int d = (int)a->block[i] - (int)b->block[i];
        total += (uint32_t)((d < 0) ? -d : d);
    }
    return (unsigned)(total / SCENE_BLOCKS);
}

/* -------------------------------------------------------------------------- */
/* Dwell poses                                                                  */
/* -------------------------------------------------------------------------- */

static bool near_deg(int value, int target)
{
    const int d = value - target;
    return (d >= -SCENE_POSE_TOLERANCE_DEG) && (d <= SCENE_POSE_TOLERANCE_DEG);
}

scene_pose_t scene_pose_from_head(int pan_deg, int tilt_deg)
{
    if (near_deg(tilt_deg, 0)) {
        if (near_deg(pan_deg, 0)) {
            return SCENE_POSE_CENTRE;
        }
        if (near_deg(pan_deg, -SCENE_POSE_SIDE_PAN_DEG)) {
            return SCENE_POSE_LEFT;
        }
        if (near_deg(pan_deg, SCENE_POSE_SIDE_PAN_DEG)) {
            return SCENE_POSE_RIGHT;
        }
        return SCENE_POSE_NONE;
    }
    if (near_deg(tilt_deg, SCENE_POSE_FLOOR_TILT_DEG) && near_deg(pan_deg, 0)) {
        return SCENE_POSE_FLOOR;
    }
    return SCENE_POSE_NONE;
}

char scene_pose_letter(uint8_t pose)
{
    static const char k_letters[SCENE_POSES] = {'C', 'L', 'R', 'F'};
    return (pose < SCENE_POSES) ? k_letters[pose] : '-';
}

/* -------------------------------------------------------------------------- */
/* Gate state                                                                   */
/* -------------------------------------------------------------------------- */

static scene_fingerprint_t s_current; /**< Most recent decodable frame. */
/** Per dwell pose, the frame the robot last spoke about at that pose. */
static scene_fingerprint_t s_reference[SCENE_POSES];
static uint8_t s_threshold = SCENE_CHANGE_THRESHOLD_DEFAULT;

/** The reference the current frame is measured against, or NULL when the
 *  current frame cannot be measured (undecoded, or off a dwell pose). */
static const scene_fingerprint_t *current_reference(void)
{
    if (!s_current.valid || s_current.pose >= SCENE_POSES) {
        return NULL;
    }
    return &s_reference[s_current.pose];
}

void scene_change_init(void)
{
    memset(&s_current, 0, sizeof(s_current));
    memset(s_reference, 0, sizeof(s_reference));
    s_threshold = SCENE_CHANGE_THRESHOLD_DEFAULT;
}

void scene_change_note(const scene_fingerprint_t *fp)
{
    if (fp == NULL || !fp->valid) {
        /* Keep the last frame we could actually see. An undecodable frame says
         * nothing about whether the scene moved, and treating it as a fresh
         * observation would let a run of corrupt captures gate speech on noise. */
        return;
    }
    s_current = *fp;
}

bool scene_change_has_measurement(void)
{
    /* note() only ever stores a valid fingerprint, so s_current.valid is set by
     * the first decodable frame and never cleared until init(). */
    return s_current.valid;
}

unsigned scene_change_score(void)
{
    const scene_fingerprint_t *ref = current_reference();
    return (ref != NULL) ? scene_fingerprint_distance(&s_current, ref) : 0u;
}

uint8_t scene_change_pose(void)
{
    return s_current.valid ? s_current.pose : (uint8_t)SCENE_POSE_NONE;
}

bool scene_change_novel(void)
{
    if (s_threshold == 0u) {
        return true; /* gate disabled */
    }
    /* Never seen a decodable frame -> fail CLOSED. This must outrank the
     * first-impression branch below: there is no first impression of a view
     * nobody has seen. In the other order this check was unreachable (s_current
     * never returns to invalid once valid, and s_reference only becomes valid by
     * copying it), so a camera whose frames never decoded opened the `speak`
     * tool on every cycle (issue #690). Same ordering as ambient_audio_novel();
     * see .claude/rules/stateless-model-gating.md §4. The boot greeting is
     * unaffected once one frame decodes. */
    if (!s_current.valid) {
        return false;
    }
    /* Off a dwell pose -> cannot measure -> fail closed, for the same reason
     * and in the same place as the undecoded frame above: a mid-slew picture
     * has no reference it could be compared with, and treating it as a first
     * impression would open `speak` on every frame the head takes while moving
     * (ADR-029 §2). */
    const scene_fingerprint_t *ref = current_reference();
    if (ref == NULL) {
        return false;
    }
    if (!ref->valid) {
        return true; /* nothing spoken about at this pose yet — its first view is new */
    }
    return scene_change_score() >= (unsigned)s_threshold;
}

bool scene_change_compared(void)
{
    /* Exactly the states in which novel() reaches its distance comparison. */
    const scene_fingerprint_t *ref = current_reference();
    return (s_threshold != 0u) && (ref != NULL) && ref->valid;
}

void scene_change_mark_spoken(void)
{
    /* Only an on-pose decoded frame becomes a reference, and only its own
     * pose's. Speaking over a mid-slew frame moves nothing (ADR-029 §2). */
    if (current_reference() == NULL) {
        return;
    }
    s_reference[s_current.pose] = s_current;
}

void scene_change_set_threshold(uint8_t threshold)
{
    s_threshold = threshold;
}

uint8_t scene_change_threshold(void)
{
    return s_threshold;
}
