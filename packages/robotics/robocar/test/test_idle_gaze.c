/**
 * @file test_idle_gaze.c
 * @brief Host tests for idle_gaze.c — where the head looks when nothing directs
 *        it (ADR-029 §1).
 *
 * idle_gaze.c is compiled unmodified. The executor integration (pre-emption by
 * a track goal or a lease, facing forward during a voice turn, the settled
 * flag the planner reads) is pinned in test_reactive_controller.c.
 */

#include "idle_gaze.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "scene_change.h"

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

static const head_limits_t k_boot_limits = {
    .pan_min = -60, .pan_max = 60, .tilt_min = -30, .tilt_max = 30};

/* A scripted random source: returns the values in order, then repeats. */
typedef struct {
    const uint32_t *values;
    size_t n;
    size_t i;
} script_t;

static uint32_t script_next(void *ctx)
{
    script_t *s = (script_t *)ctx;
    const uint32_t v = s->values[s->i % s->n];
    s->i++;
    return v;
}

/* A pseudo-random source, for tests about coverage rather than one sequence. */
static uint32_t xorshift_next(void *ctx)
{
    uint32_t *x = (uint32_t *)ctx;
    *x ^= *x << 13;
    *x ^= *x >> 17;
    *x ^= *x << 5;
    return *x;
}

/** Drive the gaze as the executor would: the head reaches each target at once
 *  (the slew is head_aim's business), one step per @p step_ms. Returns the
 *  final pose. */
static head_pose_t run(idle_gaze_t *g, head_pose_t pose, const head_limits_t *lim, uint32_t *t,
                       uint32_t until_ms, uint32_t step_ms)
{
    while ((int32_t)(until_ms - *t) > 0) {
        pose = idle_gaze_step(g, pose, lim, false, *t);
        *t += step_ms;
    }
    return pose;
}

static int pose_of(head_pose_t p)
{
    return (int)scene_pose_from_head(p.pan_deg, p.tilt_deg);
}

/* The gaze starts at centre and holds it for a drawn dwell, inside the bounds. */
static void test_starts_at_centre_and_dwells(void)
{
    static const uint32_t k_rng[] = {0u}; /* dwell = the minimum */
    script_t sc = {k_rng, 1, 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    uint32_t t = 1000u;
    head_pose_t pose = {0, 0};
    pose = idle_gaze_step(&g, pose, &k_boot_limits, false, t);
    ASSERT(pose.pan_deg == 0 && pose.tilt_deg == 0);

    pose = run(&g, pose, &k_boot_limits, &t, 1000u + GAZE_DWELL_MIN_MS - 33u, 33u);
    ASSERT(pose_of(pose) == SCENE_POSE_CENTRE);

    pose = run(&g, pose, &k_boot_limits, &t, 1000u + GAZE_DWELL_MIN_MS + 100u, 33u);
    ASSERT(pose_of(pose) != SCENE_POSE_CENTRE);
}

/* Every target is a dwell pose the scene gates can measure, and centre comes
 * back every other move. */
static void test_visits_dwell_poses_with_centre_every_other_move(void)
{
    uint32_t state = 0x12345678u;
    idle_gaze_t g;
    idle_gaze_init(&g, xorshift_next, &state);

    uint32_t t = 0u;
    head_pose_t pose = {0, 0};
    int last = -1;
    int moves = 0;
    int seen[SCENE_POSES] = {0};
    for (int i = 0; i < 20000; ++i) { /* about 11 minutes at 33 ms */
        pose = idle_gaze_step(&g, pose, &k_boot_limits, false, t);
        const int p = pose_of(pose);
        ASSERT(p != SCENE_POSE_NONE);
        if (p != last) {
            if (last != -1) {
                /* Every move is to or from centre. */
                ASSERT(p == SCENE_POSE_CENTRE || last == SCENE_POSE_CENTRE);
                moves++;
            }
            seen[p]++;
            last = p;
        }
        t += 33u;
    }
    ASSERT(moves > 40);
    for (int p = 0; p < SCENE_POSES; ++p) {
        ASSERT(seen[p] > 0);
    }
}

/* Dwells are drawn from the random source and stay inside the bounds. */
static void test_dwell_is_drawn_within_bounds(void)
{
    static const uint32_t k_rng[] = {GAZE_DWELL_MAX_MS - GAZE_DWELL_MIN_MS}; /* the maximum */
    script_t sc = {k_rng, 1, 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    uint32_t t = 0u;
    head_pose_t pose = {0, 0};
    pose = run(&g, pose, &k_boot_limits, &t, GAZE_DWELL_MAX_MS - 33u, 33u);
    ASSERT(pose_of(pose) == SCENE_POSE_CENTRE);
    pose = run(&g, pose, &k_boot_limits, &t, GAZE_DWELL_MAX_MS + 100u, 33u);
    ASSERT(pose_of(pose) != SCENE_POSE_CENTRE);
}

/* While told to face forward the gaze holds centre, and when released it dwells
 * there afresh before moving, so a turn's end is not followed by a snap away. */
static void test_face_forward_holds_centre_then_dwells_afresh(void)
{
    static const uint32_t k_rng[] = {0u};
    script_t sc = {k_rng, 1, 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    uint32_t t = 0u;
    head_pose_t pose = {0, 0};
    pose = run(&g, pose, &k_boot_limits, &t, GAZE_DWELL_MIN_MS + 100u, 33u);
    ASSERT(pose_of(pose) != SCENE_POSE_CENTRE);

    for (int i = 0; i < 600; ++i) { /* 20 s of a voice turn */
        pose = idle_gaze_step(&g, pose, &k_boot_limits, true, t);
        ASSERT(pose.pan_deg == 0 && pose.tilt_deg == 0);
        t += 33u;
    }

    const uint32_t released = t;
    pose = run(&g, pose, &k_boot_limits, &t, released + GAZE_DWELL_MIN_MS - 33u, 33u);
    ASSERT(pose_of(pose) == SCENE_POSE_CENTRE);
}

/* A pose the live limits would clamp is not a dwell pose any more (the scene
 * gates could never measure it), so the gaze skips it. */
static void test_poses_the_limits_cut_off_are_skipped(void)
{
    static const uint32_t k_rng[] = {0u, 1u, 2u, 3u, 4u, 5u};
    script_t sc = {k_rng, sizeof(k_rng) / sizeof(k_rng[0]), 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    const head_limits_t narrow = {.pan_min = -30, .pan_max = 30, .tilt_min = -30, .tilt_max = 30};
    uint32_t t = 0u;
    head_pose_t pose = {0, 0};
    for (int i = 0; i < 10000; ++i) {
        pose = idle_gaze_step(&g, pose, &narrow, false, t);
        ASSERT(pose.pan_deg >= narrow.pan_min && pose.pan_deg <= narrow.pan_max);
        const int p = pose_of(pose);
        ASSERT(p == SCENE_POSE_CENTRE || p == SCENE_POSE_FLOOR);
        t += 33u;
    }

    /* No tilt below level either: nothing but centre is left. */
    const head_limits_t level = {.pan_min = -30, .pan_max = 30, .tilt_min = 0, .tilt_max = 30};
    idle_gaze_init(&g, script_next, &sc);
    for (int i = 0; i < 10000; ++i) {
        pose = idle_gaze_step(&g, pose, &level, false, t);
        ASSERT(pose.pan_deg == 0 && pose.tilt_deg == 0);
        t += 33u;
    }
}

/* The dwell timer survives the uint32 millisecond wrap at day 49. */
static void test_dwell_survives_the_millisecond_wrap(void)
{
    static const uint32_t k_rng[] = {0u};
    script_t sc = {k_rng, 1, 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    uint32_t t = 0xFFFFFFFFu - 1000u;
    head_pose_t pose = {0, 0};
    pose = idle_gaze_step(&g, pose, &k_boot_limits, false, t);
    t += 33u;
    pose = run(&g, pose, &k_boot_limits, &t,
               (uint32_t)(0xFFFFFFFFu - 1000u + GAZE_DWELL_MIN_MS + 100u), 33u);
    ASSERT(pose_of(pose) != SCENE_POSE_CENTRE);
}

/* restart() sends the gaze back to centre with a fresh dwell — what the
 * executor does when a track goal or a lease takes the head. */
static void test_restart_returns_to_centre(void)
{
    static const uint32_t k_rng[] = {0u};
    script_t sc = {k_rng, 1, 0};
    idle_gaze_t g;
    idle_gaze_init(&g, script_next, &sc);

    uint32_t t = 0u;
    head_pose_t pose = {0, 0};
    pose = run(&g, pose, &k_boot_limits, &t, GAZE_DWELL_MIN_MS + 100u, 33u);
    ASSERT(pose_of(pose) != SCENE_POSE_CENTRE);

    idle_gaze_restart(&g);
    pose = idle_gaze_step(&g, pose, &k_boot_limits, false, t);
    ASSERT(pose.pan_deg == 0 && pose.tilt_deg == 0);
}

int main(void)
{
    printf("=== idle_gaze host tests ===\n\n");
    test_run("starts at centre and dwells", test_starts_at_centre_and_dwells);
    test_run("visits dwell poses with centre every other move",
             test_visits_dwell_poses_with_centre_every_other_move);
    test_run("dwell is drawn within bounds", test_dwell_is_drawn_within_bounds);
    test_run("face forward holds centre then dwells afresh",
             test_face_forward_holds_centre_then_dwells_afresh);
    test_run("poses the limits cut off are skipped", test_poses_the_limits_cut_off_are_skipped);
    test_run("dwell survives the millisecond wrap", test_dwell_survives_the_millisecond_wrap);
    test_run("restart returns to centre", test_restart_returns_to_centre);
    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
