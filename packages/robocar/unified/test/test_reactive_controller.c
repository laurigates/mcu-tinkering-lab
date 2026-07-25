/**
 * @file test_reactive_controller.c
 * @brief Host-based unit tests for reactive_controller decision logic.
 *
 * Exercises the one-tick executor via reactive_controller_tick_for_test(),
 * which reactive_controller.c exposes under REACTIVE_CONTROLLER_HOST_TEST=1.
 * Motor commands are captured by motor_controller_stub.c; distance readings
 * are injected via ultrasonic_test_set_distance(); goals are written via
 * goal_state_write(). Each test primes the smoothing filter with three
 * "clear" readings before probing behaviour so the running mean stabilises.
 */

#include "goal_state.h"
#include "motor_controller.h"
#include "reactive_controller.h"
#include "ultrasonic.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* From motor_controller_stub.c */
void motor_stub_reset(void);
void motor_stub_get_last_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                               uint8_t *right_direction);

/* From reactive_controller.c (host-test build) */
void reactive_controller_tick_for_test(void);

/* =========================================================================
 * Test harness
 * ========================================================================= */

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

/* =========================================================================
 * Fixtures
 * ========================================================================= */

/**
 * Reset all module state so each test starts from a known baseline.
 *
 * Note: the smoothing buffer and the manual-override lease inside
 * reactive_controller.c are *process-local* static state, cleared only by
 * reactive_controller_init(). init() is idempotent and returns early when the
 * controller is already running, so stop() first — otherwise every setup() after
 * the first is a no-op and state leaks between tests. (That was previously
 * masked: prime_distance() happens to refill the whole 3-sample filter, so the
 * stale buffer never showed up.)
 */
static void setup(void)
{
    reactive_controller_stop(); /* no-op on the first call */
    goal_state_init();
    goal_state_force_stop(); /* clear any prior goal */
    motor_stub_reset();
    ultrasonic_test_set_distance(100); /* "clear" default */
    reactive_controller_init();        /* clears smoothing buffer + manual lease */
}

/** Prime the 3-sample running-mean distance filter at @p cm. */
static void prime_distance(uint16_t cm)
{
    ultrasonic_test_set_distance(cm);
    for (int i = 0; i < 3; i++) {
        reactive_controller_tick_for_test();
    }
}

static void get_motor_state(uint8_t *l_speed, uint8_t *r_speed, uint8_t *l_dir, uint8_t *r_dir)
{
    motor_stub_get_last_state(l_speed, r_speed, l_dir, r_dir);
}

/* =========================================================================
 * Tests
 * ========================================================================= */

/* Reflex latch: smoothed distance < 15 cm → motor_stop regardless of goal. */
static void test_reflex_latch_overrides_drive(void)
{
    setup();

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 80},
    };
    ASSERT(goal_state_write(&drive, 1500) == ESP_OK);

    prime_distance(10); /* 10 cm — well below the 15 cm threshold */

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls == 0);
    ASSERT(rs == 0);
}

/* Reflex release: distance back above threshold → goal takes effect. */
static void test_reflex_release(void)
{
    setup();

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 80},
    };
    ASSERT(goal_state_write(&drive, 1500) == ESP_OK);

    /* Block, then clear. 3 ticks at 30 cm fully flushes the 3-sample buffer. */
    prime_distance(10);
    prime_distance(30);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls > 0);
    ASSERT(rs > 0);
    ASSERT(ld == 1);
    ASSERT(rd == 1);
}

/* GOAL_KIND_TRACK centered box → both wheels within 5%. */
static void test_track_centered(void)
{
    setup();

    goal_t track = {
        .kind = GOAL_KIND_TRACK,
        .params.track = {.ymin = 400, .xmin = 400, .ymax = 600, .xmax = 600, .max_speed_pct = 60},
    };
    ASSERT(goal_state_write(&track, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* Centre of (400..600) = 500 → error 0 → left == right. */
    ASSERT(ls == rs);
    ASSERT(ld == 1);
    ASSERT(rd == 1);

    /* Within 5 % of expected 60 % of 255 = 153 */
    int32_t diff = (int32_t)ls - 153;
    if (diff < 0)
        diff = -diff;
    ASSERT(diff < 10);
}

/* GOAL_KIND_TRACK right-biased box → right wheel slower. */
static void test_track_right_biased(void)
{
    setup();

    goal_t track = {
        .kind = GOAL_KIND_TRACK,
        .params.track = {.ymin = 400, .xmin = 700, .ymax = 600, .xmax = 900, .max_speed_pct = 60},
    };
    ASSERT(goal_state_write(&track, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* Centre = 800 → error +300 → left faster, right slower. */
    ASSERT(ls > rs);
    ASSERT(ld == 1);
    ASSERT(rd == 1);
}

/* GOAL_KIND_DRIVE heading=0 → both wheels forward at commanded speed. */
static void test_drive_straight(void)
{
    setup();

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 100, .speed_pct = 50},
    };
    ASSERT(goal_state_write(&drive, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* 50 % of 255 = 127 */
    ASSERT(ls == rs);
    ASSERT(ls >= 126 && ls <= 128);
    ASSERT(ld == 1);
    ASSERT(rd == 1);
}

/* GOAL_KIND_DRIVE heading=+45 → right wheel slower than left. */
static void test_drive_turn_right(void)
{
    setup();

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 45, .distance_cm = 100, .speed_pct = 80},
    };
    ASSERT(goal_state_write(&drive, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* Positive heading → right wheel slowed by bias */
    ASSERT(ls > rs);
    ASSERT(ld == 1);
    ASSERT(rd == 1);
}

/* GOAL_KIND_ROTATE angle=+30 → CW (right back, left forward). */
static void test_rotate_cw(void)
{
    setup();

    goal_t rot = {
        .kind = GOAL_KIND_ROTATE,
        .params.rotate = {.angle_deg = 30},
    };
    ASSERT(goal_state_write(&rot, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* motor_rotate_cw stub: left=1 (fwd), right=0 (back) */
    ASSERT(ld == 1);
    ASSERT(rd == 0);
    ASSERT(ls > 0);
    ASSERT(rs > 0);
}

/* GOAL_KIND_ROTATE angle=-30 → CCW (left back, right forward). */
static void test_rotate_ccw(void)
{
    setup();

    goal_t rot = {
        .kind = GOAL_KIND_ROTATE,
        .params.rotate = {.angle_deg = -30},
    };
    ASSERT(goal_state_write(&rot, 1500) == ESP_OK);

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);

    /* motor_rotate_ccw stub: left=0 (back), right=1 (fwd) */
    ASSERT(ld == 0);
    ASSERT(rd == 1);
}

/* Stale goal (TTL expired) → motor_stop. */
static int64_t g_test_time_us;
static int64_t test_clock(void)
{
    return g_test_time_us;
}

static void test_stale_goal_stops(void)
{
    setup();

    g_test_time_us = 0;
    goal_state_set_clock_override(test_clock);

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 100, .speed_pct = 80},
    };
    ASSERT(goal_state_write(&drive, 1000) == ESP_OK);

    /* Advance past TTL */
    g_test_time_us = 2000 * 1000LL;

    prime_distance(100);

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls == 0);
    ASSERT(rs == 0);

    goal_state_set_clock_override(NULL);
}

/* =========================================================================
 * Manual override (console / bench driving)
 * =========================================================================
 *
 * The three properties that make routing console commands through the executor
 * safe, rather than letting a second task write the motors directly:
 * a manual command beats a live goal, the obstacle reflex still beats a manual
 * command, and the lease expires on its own so the planner resumes.
 */

/* A manual command takes precedence over a fresh planner goal. */
static void test_manual_overrides_goal(void)
{
    setup();
    prime_distance(100); /* clear path */

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 100},
    };
    ASSERT(goal_state_write(&drive, 5000) == ESP_OK);

    /* Reverse is deliberately something no goal kind can express — if the goal
     * were winning, both directions would read forward. */
    ASSERT(reactive_controller_manual(REACTIVE_MANUAL_BACKWARD, 120, 1000) == ESP_OK);
    reactive_controller_tick_for_test();

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls == 120);
    ASSERT(rs == 120);
    ASSERT(ld == 0); /* backward */
    ASSERT(rd == 0);
}

/* The obstacle reflex still wins — a manual command cannot drive into a wall. */
static void test_reflex_overrides_manual(void)
{
    setup();

    ASSERT(reactive_controller_manual(REACTIVE_MANUAL_FORWARD, 200, 5000) == ESP_OK);
    prime_distance(10); /* below the 15 cm threshold */

    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls == 0);
    ASSERT(rs == 0);
}

/* The lease expires by itself and the planner's goal resumes — no hand-back. */
static void test_manual_lease_expires_back_to_goal(void)
{
    setup();
    prime_distance(100);

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 40},
    };
    ASSERT(goal_state_write(&drive, 60000) == ESP_OK);

    /* One loop period of lease → exactly one manual tick. */
    ASSERT(reactive_controller_manual(REACTIVE_MANUAL_BACKWARD, 120, REACTIVE_LOOP_PERIOD_MS) ==
           ESP_OK);

    reactive_controller_tick_for_test();
    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ld == 0); /* still manual: reverse */

    reactive_controller_tick_for_test();
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ld == 1); /* lease gone: goal drives forward again */
    ASSERT(rd == 1);
    ASSERT(ls == (uint8_t)(40 * 255 / 100));
}

/* An explicit STOP ends the lease immediately rather than holding for a TTL. */
static void test_manual_stop_clears_lease(void)
{
    setup();
    prime_distance(100);

    goal_t drive = {
        .kind = GOAL_KIND_DRIVE,
        .params.drive = {.heading_deg = 0, .distance_cm = 200, .speed_pct = 40},
    };
    ASSERT(goal_state_write(&drive, 60000) == ESP_OK);

    ASSERT(reactive_controller_manual(REACTIVE_MANUAL_FORWARD, 200, 60000) == ESP_OK);
    reactive_controller_tick_for_test();
    uint8_t ls, rs, ld, rd;
    get_motor_state(&ls, &rs, &ld, &rd);
    ASSERT(ls == 200);

    ASSERT(reactive_controller_manual(REACTIVE_MANUAL_STOP, 0, 60000) == ESP_OK);
    reactive_controller_tick_for_test();
    get_motor_state(&ls, &rs, &ld, &rd);
    /* Back under planner control on the very next tick, not 60 s later. */
    ASSERT(ls == (uint8_t)(40 * 255 / 100));
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== reactive_controller host tests ===\n\n");

    test_run("reflex_latch_overrides_drive", test_reflex_latch_overrides_drive);
    test_run("reflex_release", test_reflex_release);
    test_run("track_centered", test_track_centered);
    test_run("track_right_biased", test_track_right_biased);
    test_run("drive_straight", test_drive_straight);
    test_run("drive_turn_right", test_drive_turn_right);
    test_run("rotate_cw", test_rotate_cw);
    test_run("rotate_ccw", test_rotate_ccw);
    test_run("stale_goal_stops", test_stale_goal_stops);
    test_run("manual_overrides_goal", test_manual_overrides_goal);
    test_run("reflex_overrides_manual", test_reflex_overrides_manual);
    test_run("manual_lease_expires_back_to_goal", test_manual_lease_expires_back_to_goal);
    test_run("manual_stop_clears_lease", test_manual_stop_clears_lease);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
