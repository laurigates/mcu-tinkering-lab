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
/* Gate state                                                                   */
/* -------------------------------------------------------------------------- */

static scene_fingerprint_t s_current;   /**< Most recent decodable frame. */
static scene_fingerprint_t s_reference; /**< The frame the robot last spoke about. */
static uint8_t s_threshold = SCENE_CHANGE_THRESHOLD_DEFAULT;

void scene_change_init(void)
{
    memset(&s_current, 0, sizeof(s_current));
    memset(&s_reference, 0, sizeof(s_reference));
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

unsigned scene_change_score(void)
{
    return scene_fingerprint_distance(&s_current, &s_reference);
}

bool scene_change_novel(void)
{
    if (s_threshold == 0u) {
        return true; /* gate disabled */
    }
    if (!s_reference.valid) {
        return true; /* nothing spoken about yet — the first view is new */
    }
    if (!s_current.valid) {
        return false; /* never seen a decodable frame: nothing to remark on */
    }
    return scene_change_score() >= (unsigned)s_threshold;
}

void scene_change_mark_spoken(void)
{
    s_reference = s_current;
}

void scene_change_set_threshold(uint8_t threshold)
{
    s_threshold = threshold;
}

uint8_t scene_change_threshold(void)
{
    return s_threshold;
}
