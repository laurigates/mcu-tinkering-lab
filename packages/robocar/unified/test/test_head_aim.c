/**
 * @file test_head_aim.c
 * @brief Host tests for head_aim.c — the pure pan/tilt aiming logic (#511).
 *
 * head_aim.c is compiled unmodified. The executor integration (servo writes,
 * wheel commands, fallback without servos) is pinned separately in
 * test_reactive_controller.c.
 */

#include "head_aim.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

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
static const head_pose_t k_centre = {0, 0};

/** A box whose centre sits at (cx, cy). */
static head_box_t box_at(uint16_t cx, uint16_t cy)
{
    head_box_t b = {.ymin = cy - 50, .xmin = cx - 50, .ymax = cy + 50, .xmax = cx + 50};
    return b;
}

/** Run @p ticks of track, feeding each command back in as the next pose. */
static head_pose_t run_track(head_aim_t *s, uint32_t seq, head_box_t box, head_pose_t capture,
                             head_pose_t pose, const head_limits_t *lim, int ticks,
                             head_aim_out_t *last)
{
    for (int i = 0; i < ticks; ++i) {
        *last = head_aim_track(s, seq, &box, capture, pose, lim, true);
        pose = last->cmd;
    }
    return pose;
}

/* A box in the frame's right third, well inside the head's travel: the head
 * turns toward it and the wheels are not asked to do anything. */
static void test_head_leads_without_turning_the_body(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    head_aim_out_t out;
    /* cx 800 -> +300 of 1000 * 60 deg = +18 deg. */
    const head_pose_t pose =
        run_track(&s, 1, box_at(800, 500), k_centre, k_centre, &k_boot_limits, 20, &out);

    ASSERT(pose.pan_deg == 18);
    ASSERT(pose.tilt_deg == 0);
    ASSERT(out.body == HEAD_BODY_HOLD);
}

/* The first step is slewed, not a snap to the bearing. */
static void test_head_slews(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_box_t box = box_at(1000, 500); /* +30 deg */
    const head_aim_out_t out =
        head_aim_track(&s, 1, &box, k_centre, k_centre, &k_boot_limits, true);
    ASSERT(out.cmd.pan_deg == HEAD_SLEW_DEG_PER_TICK);
}

/* The bearing is latched per goal. Re-reading the same box every tick from a
 * turned head must NOT add the offset again — that would wind the head into
 * its limit on a target it already faces. */
static void test_bearing_is_latched_per_goal(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    head_aim_out_t out;
    head_pose_t pose =
        run_track(&s, 7, box_at(800, 500), k_centre, k_centre, &k_boot_limits, 50, &out);
    ASSERT(pose.pan_deg == 18);

    /* A new plan from a frame taken with the head at +18 that shows the target
     * dead centre: same bearing, head stays put. */
    pose = run_track(&s, 8, box_at(500, 500), pose, pose, &k_boot_limits, 50, &out);
    ASSERT(pose.pan_deg == 18);
}

/* Tilt follows the box's vertical offset; image y grows downward, tilt up. */
static void test_tilt_follows_box_upward(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    head_aim_out_t out;
    /* cy 300 -> 200 of 1000 above centre * 45 deg = +9 deg. */
    const head_pose_t pose =
        run_track(&s, 1, box_at(500, 300), k_centre, k_centre, &k_boot_limits, 20, &out);
    ASSERT(pose.tilt_deg == 9);
    ASSERT(pose.pan_deg == 0);
}

/* Past the engage edge the body turns toward the target, and the head
 * re-centres as the estimated heading catches up. The turn ends by itself. */
static void test_wheels_engage_near_pan_limit_and_head_recentres(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    /* Head already at +40 when the frame was captured; target at the right
     * edge of that frame: bearing 40 + 30 = +70, past the 60 deg limit. */
    const head_pose_t capture = {40, 0};
    const head_box_t box = box_at(1000, 500);

    head_aim_out_t out = head_aim_track(&s, 1, &box, capture, capture, &k_boot_limits, true);
    ASSERT(out.body == HEAD_BODY_TURN_CW);

    head_pose_t pose = out.cmd;
    int turning_ticks = 1;
    int max_pan = pose.pan_deg;
    for (int i = 0; i < 200; ++i) {
        out = head_aim_track(&s, 1, &box, capture, pose, &k_boot_limits, true);
        pose = out.cmd;
        if (pose.pan_deg > max_pan)
            max_pan = pose.pan_deg;
        if (out.body == HEAD_BODY_TURN_CW)
            turning_ticks++;
        ASSERT(out.body != HEAD_BODY_TURN_CCW);
    }
    ASSERT(max_pan <= 60);              /* never past the limit */
    ASSERT(out.body == HEAD_BODY_HOLD); /* the turn ended */
    /* Head back within the deadband of centre. */
    ASSERT(abs(pose.pan_deg) <= HEAD_RECENTRE_DEADBAND_DEG);
    /* 70 deg at ~5.9 deg/tick is ~11-12 ticks: bounded, not a spin. */
    ASSERT(turning_ticks > 5 && turning_ticks < 20);
}

/* Left side mirrors the right. */
static void test_wheels_engage_to_the_left(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_pose_t capture = {-50, 0};
    const head_box_t box = box_at(100, 500); /* -24 deg -> bearing -74 */
    const head_aim_out_t out = head_aim_track(&s, 1, &box, capture, capture, &k_boot_limits, true);
    ASSERT(out.body == HEAD_BODY_TURN_CCW);
}

/* Inside the engage edge the wheels hold, even close to it. */
static void test_no_body_turn_inside_engage_edge(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    /* Engage edge is 80 % of 60 = 48; bearing 46 stays with the head. */
    const head_pose_t capture = {40, 0};
    const head_box_t box = box_at(600, 500); /* +6 deg */
    const head_aim_out_t out = head_aim_track(&s, 1, &box, capture, capture, &k_boot_limits, true);
    ASSERT(out.body == HEAD_BODY_HOLD);
}

/* While the wheels are held (reflex, manual lease) the turn is neither
 * commanded nor counted, so the estimate does not claim a heading change that
 * never happened. */
static void test_held_wheels_do_not_advance_the_yaw_estimate(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_pose_t capture = {40, 0};
    const head_box_t box = box_at(1000, 500); /* bearing +70 */
    head_pose_t pose = capture;
    for (int i = 0; i < 100; ++i) {
        const head_aim_out_t out =
            head_aim_track(&s, 1, &box, capture, pose, &k_boot_limits, false);
        ASSERT(out.body == HEAD_BODY_HOLD);
        pose = out.cmd;
    }
    ASSERT(pose.pan_deg == 60); /* holding at the limit, still looking */
    const head_aim_out_t out = head_aim_track(&s, 1, &box, capture, pose, &k_boot_limits, true);
    ASSERT(out.body == HEAD_BODY_TURN_CW); /* resumes the full turn */
}

/* No command ever leaves the live limits — including a far target, and a limit
 * narrowed while the head sits outside the new range. */
static void test_commands_never_exceed_limits(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    head_pose_t pose = k_centre;
    const head_pose_t capture = {60, 30};
    /* Top-right corner: bearing 60 + 27, tilt 30 + 20 — both past the limits. */
    const head_box_t far = {.ymin = 0, .xmin = 900, .ymax = 100, .xmax = 1000};
    for (int i = 0; i < 100; ++i) {
        const head_aim_out_t out =
            head_aim_track(&s, 1, &far, capture, pose, &k_boot_limits, false);
        ASSERT(out.cmd.pan_deg >= -60 && out.cmd.pan_deg <= 60);
        ASSERT(out.cmd.tilt_deg >= -30 && out.cmd.tilt_deg <= 30);
        pose = out.cmd;
    }
    ASSERT(pose.pan_deg == 60 && pose.tilt_deg == 30);

    /* `servo limit pan -20 20` with the head at +60: the very next command is
     * inside, not six degrees nearer. */
    const head_limits_t narrow = {.pan_min = -20, .pan_max = 20, .tilt_min = -10, .tilt_max = 10};
    head_aim_out_t out = head_aim_track(&s, 1, &far, capture, pose, &narrow, false);
    ASSERT(out.cmd.pan_deg == 20);
    ASSERT(out.cmd.tilt_deg == 10);

    out = head_aim_centre(&s, pose, &narrow);
    ASSERT(out.cmd.pan_deg >= -20 && out.cmd.pan_deg <= 20);
    ASSERT(out.cmd.tilt_deg >= -10 && out.cmd.tilt_deg <= 10);
}

/* Centring slews home and drops the latch and any turn in progress. */
static void test_centre_returns_home_and_forgets_goal(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_pose_t capture = {40, 0};
    const head_box_t box = box_at(1000, 500);
    (void)head_aim_track(&s, 1, &box, capture, capture, &k_boot_limits, true);
    ASSERT(s.body_turning);

    head_pose_t pose = {30, -12};
    head_aim_out_t out = {0};
    for (int i = 0; i < 20; ++i) {
        out = head_aim_centre(&s, pose, &k_boot_limits);
        pose = out.cmd;
    }
    ASSERT(pose.pan_deg == 0 && pose.tilt_deg == 0);
    ASSERT(out.body == HEAD_BODY_HOLD);
    ASSERT(!s.latched);
    ASSERT(!s.body_turning);
}

/* A new goal sequence re-latches even when the box is identical. */
static void test_new_goal_relatches(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_box_t box = box_at(500, 500);
    (void)head_aim_track(&s, 1, &box, (head_pose_t){20, 0}, k_centre, &k_boot_limits, true);
    ASSERT(s.bearing_deg > 19.0f && s.bearing_deg < 21.0f);
    (void)head_aim_track(&s, 2, &box, (head_pose_t){-20, 0}, k_centre, &k_boot_limits, true);
    ASSERT(s.bearing_deg < -19.0f && s.bearing_deg > -21.0f);
}

/* Write suppression: an unchanged command is not re-sent until the refresh
 * falls due, a changed one is sent at once, and a failure retries. */
static void test_write_suppression_and_refresh(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_pose_t c = {10, 5};

    ASSERT(head_aim_write_due(&s, c, c, 0)); /* never written */
    head_aim_note_write(&s, c, 0, true);
    ASSERT(!head_aim_write_due(&s, c, c, 1));
    ASSERT(!head_aim_write_due(&s, c, c, HEAD_REFRESH_INTERVAL_MS - 1));
    ASSERT(head_aim_write_due(&s, c, c, HEAD_REFRESH_INTERVAL_MS));

    const head_pose_t moved = {11, 5};
    ASSERT(head_aim_write_due(&s, moved, c, 2));

    head_aim_note_write(&s, c, 5, false);
    ASSERT(head_aim_write_due(&s, c, c, 6)); /* failed write is not remembered */
}

/* The uint32 millisecond wrap at day 49 neither refreshes every tick nor
 * stops refreshing. */
static void test_refresh_survives_millisecond_wrap(void)
{
    head_aim_t s;
    head_aim_reset(&s);
    const head_pose_t c = {0, 0};
    const uint32_t before_wrap = 0xFFFFFF00u;
    head_aim_note_write(&s, c, before_wrap, true);

    ASSERT(!head_aim_write_due(&s, c, c, before_wrap + 100u)); /* 100 ms, no wrap */
    ASSERT(!head_aim_write_due(&s, c, c, 0x00000010u));        /* 272 ms, wrapped */
    ASSERT(head_aim_write_due(&s, c, c, before_wrap + HEAD_REFRESH_INTERVAL_MS)); /* wrapped */
}

/* drive(heading) is stated in the frame the planner saw. */
static void test_drive_heading_folds_capture_pan(void)
{
    ASSERT(head_aim_body_heading(0, 0) == 0);
    ASSERT(head_aim_body_heading(0, 40) == 40);
    ASSERT(head_aim_body_heading(-30, 40) == 10);
    ASSERT(head_aim_body_heading(170, 40) == -150);
    ASSERT(head_aim_body_heading(-170, -40) == 150);
}

int main(void)
{
    printf("=== head_aim host tests ===\n\n");

    test_run("head_leads_without_turning_the_body", test_head_leads_without_turning_the_body);
    test_run("head_slews", test_head_slews);
    test_run("bearing_is_latched_per_goal", test_bearing_is_latched_per_goal);
    test_run("tilt_follows_box_upward", test_tilt_follows_box_upward);
    test_run("wheels_engage_near_pan_limit_and_head_recentres",
             test_wheels_engage_near_pan_limit_and_head_recentres);
    test_run("wheels_engage_to_the_left", test_wheels_engage_to_the_left);
    test_run("no_body_turn_inside_engage_edge", test_no_body_turn_inside_engage_edge);
    test_run("held_wheels_do_not_advance_the_yaw_estimate",
             test_held_wheels_do_not_advance_the_yaw_estimate);
    test_run("commands_never_exceed_limits", test_commands_never_exceed_limits);
    test_run("centre_returns_home_and_forgets_goal", test_centre_returns_home_and_forgets_goal);
    test_run("new_goal_relatches", test_new_goal_relatches);
    test_run("write_suppression_and_refresh", test_write_suppression_and_refresh);
    test_run("refresh_survives_millisecond_wrap", test_refresh_survives_millisecond_wrap);
    test_run("drive_heading_folds_capture_pan", test_drive_heading_folds_capture_pan);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
