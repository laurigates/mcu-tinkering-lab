/**
 * @file idle_gaze.h
 * @brief Where the head looks when nothing directs it (ADR-029 §1).
 *
 * Without this the head sits centred whenever the planner has not sent a
 * `track` goal, which is most of the time, and people watching read a still
 * head as an inattentive robot. The idle gaze moves the head between a small
 * fixed set of dwell poses, the same poses the scene gates keep one reference
 * each for (scene_pose_t in scene_change.h), and holds each for a few seconds:
 *
 *   - Dwell poses, not sweeps. A frame is only measurable at a dwell pose, so a
 *     sweep would blind the scene gates for its whole length.
 *   - Centre every other move, so the view the planner knows comes back often.
 *   - Order and dwell are drawn from an injected random source (esp_random() on
 *     the board, a script in the host test).
 *   - A pose the live travel limits would clamp is skipped: the head could not
 *     reach it, and a frame taken at the clamped angle is off every dwell pose.
 *   - While told to face forward (a voice turn is listening) it holds centre,
 *     and dwells there afresh once released.
 *
 * This module only picks the target. head_aim_gaze() slews the head toward it
 * at HEAD_GAZE_SLEW_DEG_PER_TICK, and reactive_controller.c decides when the
 * gaze has the head at all: never while a fresh `track` goal or a head lease
 * holds it (ADR-025 and the console leases keep priority).
 *
 * Pure C: no ESP-IDF, no FreeRTOS. Compiled unmodified into the firmware and
 * test/test_idle_gaze.c.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "head_aim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Shortest and longest dwell at a pose, ms. ADR-029's starting values; the
 *  bench sets the real ones. The planner captures every 15 s, so most dwells
 *  see at most one planner frame. */
#define GAZE_DWELL_MIN_MS 3000U
#define GAZE_DWELL_MAX_MS 8000U

/** Random source: any uniformly distributed 32-bit value. */
typedef uint32_t (*idle_gaze_rng_t)(void *ctx);

/** Gaze state. Initialise with idle_gaze_init(). */
typedef struct {
    uint8_t target;          /**< scene_pose_t the head is going to or dwelling at. */
    bool arrived;            /**< The head has reached `target`; dwell is running.  */
    uint32_t dwell_until_ms; /**< When the dwell at `target` ends.                  */
    idle_gaze_rng_t rng;
    void *rng_ctx;
} idle_gaze_t;

/** Start at centre with no dwell running yet. */
void idle_gaze_init(idle_gaze_t *g, idle_gaze_rng_t rng, void *rng_ctx);

/** Back to centre with a fresh dwell. Called when something else takes the
 *  head, so the gaze resumes from the view the planner knows best. */
void idle_gaze_restart(idle_gaze_t *g);

/**
 * @brief One gaze step: the pose the head should be heading for.
 *
 * @param now           Current head pose (the last commanded angle).
 * @param lim           Live travel limits.
 * @param face_forward  Hold centre (a voice turn is listening).
 * @param now_ms        Milliseconds since boot; wrap-safe.
 * @return The target pose, inside @p lim. Not slewed.
 */
head_pose_t idle_gaze_step(idle_gaze_t *g, head_pose_t now, const head_limits_t *lim,
                           bool face_forward, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
