/**
 * @file test_plan_activity.c
 * @brief Host tests for the gate that decides whether a planner cycle is worth
 *        a Gemini request at all.
 *
 * Every claim this module makes is about something that takes minutes or days
 * to observe on hardware and cannot be staged reliably even then:
 *
 *   - a board left on a still desk makes NO requests (a bench room is never
 *     quite still, and the failure is a bill rather than a symptom);
 *   - the sensor's own AGC breathing on a motionless scene does not count as a
 *     change (the exact false positive that would put the spending back);
 *   - the ladder climbs 15 -> 30 -> 60 -> 120 -> 300 s and only THEN stops;
 *   - the uint32 millisecond counter wrapping at day 49 does not leave the
 *     robot dormant for another 49;
 *   - a camera and a microphone that have never produced a reading do not
 *     between them manufacture evidence of anything.
 *
 * Real luma planes are used rather than hand-built fingerprints, so the
 * brightness-shift test exercises the actual code path the robot runs — see
 * ~/.claude/rules/never-fabricate-test-identifiers.md on why a retyped
 * equivalent would not be the same test.
 */

#include "plan_activity.h"

#include <stdio.h>
#include <string.h>

#include "scene_change.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                            \
        }                                                            \
    } while (0)

#define BASE_MS 15000u

#define TEST(name)                   \
    static void name(void);          \
    static void run_##name(void)     \
    {                                \
        printf("- %s\n", #name);     \
        plan_activity_init(BASE_MS); \
        name();                      \
    }                                \
    static void name(void)

/* =========================================================================
 * Synthetic frames — the real thumbnail geometry (320x240 at 1/8 scale)
 * ========================================================================= */

#define W 40
#define H 30

static uint8_t g_frame[W * H];

static void fill(uint8_t level)
{
    memset(g_frame, level, sizeof(g_frame));
}

static void rect(int x0, int y0, int x1, int y1, uint8_t level)
{
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            g_frame[(y * W) + x] = level;
        }
    }
}

static scene_fingerprint_t fingerprint(void)
{
    scene_fingerprint_t fp;
    scene_fingerprint_from_luma(g_frame, W, H, &fp);
    return fp;
}

/** A plain, decodable, unremarkable view. */
static scene_fingerprint_t view_a(void)
{
    fill(100);
    rect(0, 0, 20, 30, 160);
    return fingerprint();
}

/** A clearly different view — the bright half moves to the other side. */
static scene_fingerprint_t view_b(void)
{
    fill(100);
    rect(20, 0, 40, 30, 160);
    return fingerprint();
}

/** view_a with every pixel lifted, i.e. what the AGC does to a motionless
 *  scene between one frame and the next. */
static scene_fingerprint_t view_a_brighter(void)
{
    fill(140);
    rect(0, 0, 20, 30, 200);
    return fingerprint();
}

/** A frame nobody could decode. */
static scene_fingerprint_t view_undecodable(void)
{
    scene_fingerprint_t fp;
    scene_fingerprint_from_luma(NULL, 0, 0, &fp);
    return fp;
}

/** Evidence carrying only a frame — no sound, no rangefinder, not moving. */
static plan_evidence_t quiet(scene_fingerprint_t fp)
{
    plan_evidence_t ev = {0};
    ev.frame = fp;
    return ev;
}

/* =========================================================================
 * Boot behaviour
 * ========================================================================= */

/**
 * The headline promise: a board powered on in a static room makes no request at
 * all. The first frame is adopted as the reference rather than treated as a
 * change — a first observation cannot be evidence of a change, because there is
 * nothing it could have changed from.
 */
TEST(test_boots_dormant_and_a_still_room_never_calls)
{
    ASSERT(plan_activity_dormant());

    uint32_t t = 0u;
    for (int i = 0; i < 400; i++) { /* 400 ticks = well over an hour */
        ASSERT(!plan_activity_should_call(&(plan_evidence_t){.frame = view_a()}, t));
        t += BASE_MS;
    }
    ASSERT(plan_activity_dormant());
}

TEST(test_a_changed_view_wakes_it)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_evidence_t b = quiet(view_b());
    ASSERT(plan_activity_should_call(&b, BASE_MS));
    ASSERT(!plan_activity_dormant());
    ASSERT(strchr(plan_activity_verdict(), 'V') != NULL);
}

/**
 * THE load-bearing case. The OV3660's AGC/AEC rewrites gain and exposure every
 * frame, so a motionless scene drifts in absolute brightness continuously. An
 * absolute comparison here would wake the robot every time a cloud passed —
 * putting back exactly the spending this module removes, while looking like a
 * threshold that needed tuning rather than a representation that was wrong.
 */
TEST(test_a_uniform_brightness_shift_is_not_a_change)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_evidence_t brighter = quiet(view_a_brighter());
    ASSERT(!plan_activity_should_call(&brighter, BASE_MS));
    ASSERT(plan_activity_scene_score() == 0u);
    ASSERT(plan_activity_dormant());
}

/* =========================================================================
 * The other three senses
 * ========================================================================= */

TEST(test_an_audio_event_wakes_it)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_evidence_t heard = quiet(view_a());
    heard.audio_event = true;
    ASSERT(plan_activity_should_call(&heard, BASE_MS));
    ASSERT(strchr(plan_activity_verdict(), 'A') != NULL);
}

TEST(test_a_range_change_wakes_it)
{
    plan_evidence_t a = quiet(view_a());
    a.distance_cm = 150u;
    a.range_valid = true;
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_evidence_t near = quiet(view_a());
    near.distance_cm = 100u; /* 50 cm closer, over the 20 cm default */
    near.range_valid = true;
    ASSERT(plan_activity_should_call(&near, BASE_MS));
    ASSERT(plan_activity_range_score() == 50u);
    ASSERT(strchr(plan_activity_verdict(), 'R') != NULL);
}

/**
 * A dead rangefinder reads as max range through the executor's filter, so an
 * unguarded reading would show one large step and then sit perfectly still —
 * a broken sensor that appears to agree nothing is happening.
 */
TEST(test_an_invalid_range_reading_contributes_nothing)
{
    plan_evidence_t a = quiet(view_a());
    a.distance_cm = 150u;
    a.range_valid = true;
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_evidence_t failed = quiet(view_a());
    failed.distance_cm = 400u; /* the filter's max-range fallback */
    failed.range_valid = false;
    ASSERT(!plan_activity_should_call(&failed, BASE_MS));
    ASSERT(plan_activity_range_score() == 0u);
}

TEST(test_a_moving_robot_stays_at_full_rate)
{
    uint32_t t = 0u;
    for (int i = 0; i < 20; i++) {
        plan_evidence_t moving = quiet(view_a());
        moving.robot_moving = true;
        ASSERT(plan_activity_should_call(&moving, t));
        plan_activity_note_call(t);
        ASSERT(plan_activity_step() == 0u);
        ASSERT(!plan_activity_dormant());
        t += BASE_MS;
    }
}

TEST(test_an_external_wake_is_one_shot)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_activity_wake();
    ASSERT(plan_activity_should_call(&a, BASE_MS));
    ASSERT(strchr(plan_activity_verdict(), 'W') != NULL);
    plan_activity_note_call(BASE_MS);

    /* The wake is spent. Without a real change the ladder starts climbing again
     * rather than the console command latching the robot awake forever. */
    ASSERT(!plan_activity_should_call(&a, BASE_MS + 1u));
}

/* =========================================================================
 * The ladder
 * ========================================================================= */

/**
 * The full climb, which is the whole reason this is not an on/off switch: a
 * wake detector that has gone deaf costs a few minutes of reduced cadence and
 * five requests, not a robot that never comes back.
 */
TEST(test_the_ladder_climbs_then_stops)
{
    /* Wake it once so there is something to back off from. */
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));
    plan_activity_wake();
    ASSERT(plan_activity_should_call(&a, 0u));
    plan_activity_note_call(0u);
    ASSERT(plan_activity_step() == 0u);

    /* Rungs, in base-period multiples: 1, 2, 4, 8, 20. */
    static const uint32_t k_expect[PLAN_LADDER_STEPS] = {1u, 2u, 4u, 8u, 20u};

    uint32_t last_call = 0u;
    for (int rung = 0; rung < PLAN_LADDER_STEPS; rung++) {
        const uint32_t due = k_expect[rung] * BASE_MS;
        ASSERT(plan_activity_period_ms() == due);

        /* One tick short of due: still holding. */
        ASSERT(!plan_activity_should_call(&a, last_call + due - BASE_MS));
        /* Due: one keep-alive request. */
        ASSERT(plan_activity_should_call(&a, last_call + due));
        last_call += due;
        plan_activity_note_call(last_call);
    }

    /* Ladder spent — no more requests at all, however long we wait. */
    ASSERT(plan_activity_dormant());
    ASSERT(plan_activity_period_ms() == 0u);
    for (int i = 0; i < 1000; i++) {
        last_call += BASE_MS;
        ASSERT(!plan_activity_should_call(&a, last_call));
    }
}

TEST(test_evidence_resets_the_ladder_to_the_base_period)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));
    plan_activity_wake();
    ASSERT(plan_activity_should_call(&a, 0u));
    plan_activity_note_call(0u);

    /* Climb two rungs. */
    ASSERT(plan_activity_should_call(&a, BASE_MS));
    plan_activity_note_call(BASE_MS);
    ASSERT(plan_activity_step() == 1u);
    ASSERT(plan_activity_should_call(&a, BASE_MS + (2u * BASE_MS)));
    plan_activity_note_call(BASE_MS + (2u * BASE_MS));
    ASSERT(plan_activity_step() == 2u);

    /* Something happens. */
    plan_evidence_t b = quiet(view_b());
    ASSERT(plan_activity_should_call(&b, 100000u));
    ASSERT(plan_activity_step() == 0u);
    ASSERT(plan_activity_period_ms() == BASE_MS);
}

TEST(test_evidence_wakes_a_dormant_planner)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));
    ASSERT(plan_activity_dormant());

    plan_evidence_t b = quiet(view_b());
    ASSERT(plan_activity_should_call(&b, 999999u));
    ASSERT(!plan_activity_dormant());
    ASSERT(plan_activity_step() == 0u);
}

/**
 * esp_timer's millisecond count wraps every 49.7 days. Unsigned subtraction
 * makes that a non-event; signed arithmetic or a naive `now > last + due` would
 * leave the robot holding for another 49 days, which is indistinguishable from
 * a broken wake detector and would take a month to reproduce.
 */
TEST(test_the_millisecond_wrap_at_day_49_does_not_stall_the_ladder)
{
    const uint32_t before_wrap = 0xFFFFFFFFu - (BASE_MS / 2u);

    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, before_wrap));
    plan_activity_wake();
    ASSERT(plan_activity_should_call(&a, before_wrap));
    plan_activity_note_call(before_wrap);

    /* One base period later, which is on the far side of the rollover. */
    const uint32_t after_wrap = before_wrap + BASE_MS; /* wraps to ~7500 */
    ASSERT(after_wrap < before_wrap);                  /* the wrap really happened */
    ASSERT(plan_activity_should_call(&a, after_wrap));
}

/* =========================================================================
 * Failing closed
 * ========================================================================= */

/**
 * A camera that has never returned a decodable frame and a microphone that has
 * never delivered one must not, between them, manufacture evidence. An all-zero
 * fingerprint is byte-identical to a flat grey frame, so an unguarded module
 * would either see a permanent change (fail open — spends forever, which is
 * this project's ambient_audio.c bug transplanted) or adopt garbage as its
 * reference and then report the first real frame as a whole new room.
 */
TEST(test_undecodable_frames_neither_wake_nor_become_the_reference)
{
    uint32_t t = 0u;
    for (int i = 0; i < 50; i++) {
        plan_evidence_t broken = quiet(view_undecodable());
        ASSERT(!plan_activity_should_call(&broken, t));
        ASSERT(plan_activity_scene_score() == 0u);
        t += BASE_MS;
    }
    ASSERT(plan_activity_dormant());

    /* The first frame that DOES decode is a first observation, not a change:
     * it becomes the reference and is not itself evidence. */
    plan_evidence_t first_good = quiet(view_a());
    ASSERT(!plan_activity_should_call(&first_good, t));

    /* And the gate works normally from there. */
    t += BASE_MS;
    plan_evidence_t changed = quiet(view_b());
    ASSERT(plan_activity_should_call(&changed, t));
}

TEST(test_an_undecodable_frame_is_not_adopted_by_note_call)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    plan_activity_wake();
    plan_evidence_t broken = quiet(view_undecodable());
    ASSERT(plan_activity_should_call(&broken, BASE_MS));
    plan_activity_note_call(BASE_MS);

    /* The reference must still be view_a, not the all-zero fingerprint — had it
     * been adopted, this identical frame would read as a completely new room.
     *
     * Sampled one millisecond after the call rather than a base period later:
     * at +1 base period the ladder's own keep-alive falls due and should_call()
     * returns true for a reason that has nothing to do with the reference,
     * which would make this assertion pass or fail for the wrong reason. */
    plan_evidence_t same = quiet(view_a());
    ASSERT(!plan_activity_should_call(&same, BASE_MS + 1u));
    ASSERT(plan_activity_scene_score() == 0u);
    ASSERT(strchr(plan_activity_verdict(), 'V') == NULL);
}

/* =========================================================================
 * Knobs
 * ========================================================================= */

TEST(test_a_zero_threshold_disables_that_sub_gate)
{
    plan_activity_configure(0u /* scene off */, PLAN_RANGE_THRESHOLD_CM_DEFAULT);

    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));

    /* A completely different view now contributes nothing. Note the polarity:
     * unlike scene_change's threshold, 0 here means "never fires", because
     * every term in this gate is ORed and the neutral element follows the
     * operator. */
    plan_evidence_t b = quiet(view_b());
    ASSERT(!plan_activity_should_call(&b, BASE_MS));
}

TEST(test_disabling_the_mechanism_restores_a_request_every_tick)
{
    plan_activity_set_enabled(false);
    ASSERT(!plan_activity_enabled());
    ASSERT(!plan_activity_dormant());

    uint32_t t = 0u;
    for (int i = 0; i < 100; i++) {
        plan_evidence_t a = quiet(view_a());
        ASSERT(plan_activity_should_call(&a, t));
        plan_activity_note_call(t);
        ASSERT(plan_activity_step() == 0u);
        t += BASE_MS;
    }
}

TEST(test_re_enabling_hands_back_a_fresh_ladder_not_a_latched_dormancy)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));
    ASSERT(plan_activity_dormant());

    plan_activity_set_enabled(false);
    ASSERT(plan_activity_should_call(&a, BASE_MS));
    plan_activity_note_call(BASE_MS);

    plan_activity_set_enabled(true);
    /* `plan off` then `plan on` must not resurrect the dormancy that was in
     * force before — the operator turned the mechanism off and back on, which
     * is a reset, not a pause. */
    ASSERT(!plan_activity_dormant());
    ASSERT(plan_activity_step() == 0u);
}

TEST(test_sleep_forces_dormancy_immediately)
{
    plan_evidence_t a = quiet(view_a());
    ASSERT(!plan_activity_should_call(&a, 0u));
    plan_activity_wake();
    ASSERT(plan_activity_should_call(&a, 0u));
    plan_activity_note_call(0u);
    ASSERT(!plan_activity_dormant());

    plan_activity_sleep();
    ASSERT(plan_activity_dormant());
    ASSERT(!plan_activity_should_call(&a, 10u * BASE_MS));

    /* Still wakes on real evidence — `plan sleep` is a nap, not a kill switch. */
    plan_evidence_t b = quiet(view_b());
    ASSERT(plan_activity_should_call(&b, 11u * BASE_MS));
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    printf("plan_activity host tests\n");

    run_test_boots_dormant_and_a_still_room_never_calls();
    run_test_a_changed_view_wakes_it();
    run_test_a_uniform_brightness_shift_is_not_a_change();
    run_test_an_audio_event_wakes_it();
    run_test_a_range_change_wakes_it();
    run_test_an_invalid_range_reading_contributes_nothing();
    run_test_a_moving_robot_stays_at_full_rate();
    run_test_an_external_wake_is_one_shot();
    run_test_the_ladder_climbs_then_stops();
    run_test_evidence_resets_the_ladder_to_the_base_period();
    run_test_evidence_wakes_a_dormant_planner();
    run_test_the_millisecond_wrap_at_day_49_does_not_stall_the_ladder();
    run_test_undecodable_frames_neither_wake_nor_become_the_reference();
    run_test_an_undecodable_frame_is_not_adopted_by_note_call();
    run_test_a_zero_threshold_disables_that_sub_gate();
    run_test_disabling_the_mechanism_restores_a_request_every_tick();
    run_test_re_enabling_hands_back_a_fresh_ladder_not_a_latched_dormancy();
    run_test_sleep_forces_dormancy_immediately();

    if (g_failures == 0) {
        printf("PASS\n");
        return 0;
    }
    printf("FAILED: %d assertion(s)\n", g_failures);
    return 1;
}
