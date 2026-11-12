/**
 * @file test_planner_cycle_core.c
 * @brief Host tests for the planner's per-cycle decisions.
 *
 * These are the decisions planner_task.c used to make inline, where a bench can
 * only ever show one of them at a time: the one-shot announce latches over a
 * night of dormancy, the backoff ladder across a run of 429s, and a planner line
 * dropped because a person spoke while the request was in flight. Each case
 * here is a few calls against the real pure modules the firmware links.
 */

#include <stdio.h>
#include <string.h>

#include "ambient_audio.h"
#include "dialogue_style.h"
#include "note_gate.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "planner_cycle_core.h"
#include "planner_task.h"
#include "scene_change.h"
#include "speech_budget.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                            \
        }                                                            \
    } while (0)

static void reset_modules(void)
{
    plan_activity_init(PLANNER_LOOP_PERIOD_MS);
    plan_budget_init();
    scene_change_init();
    ambient_audio_init();
    speech_budget_init();
    dialogue_style_reset_recent();
    note_gate_init();
}

#define TEST(name)               \
    static void name(void);      \
    static void run_##name(void) \
    {                            \
        printf("- %s\n", #name); \
        reset_modules();         \
        name();                  \
    }                            \
    static void name(void)

/* ---- request gate and announce latches ----------------------------------- */

TEST(test_dormant_announces_once_then_stays_quiet)
{
    planner_cycle_state_t st = {0};
    planner_cycle_decision_t d = planner_cycle_gate(&st, false, true, true);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE);
    ASSERT(st.dormant_announced);
    for (int i = 0; i < 5; i++) {
        d = planner_cycle_gate(&st, false, true, true);
        ASSERT(d.action == PLANNER_CYCLE_SKIP_DORMANT_QUIET);
        ASSERT(!d.announce_waking);
    }
}

TEST(test_climbing_the_ladder_logs_idle_and_sets_no_latch)
{
    planner_cycle_state_t st = {0};
    const planner_cycle_decision_t d = planner_cycle_gate(&st, false, true, false);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_IDLE);
    ASSERT(!st.dormant_announced && !st.budget_announced);
}

TEST(test_blown_fuse_announces_once_and_outranks_dormancy)
{
    planner_cycle_state_t st = {0};
    /* Warranted or not, dormant or not: an unaffordable cycle is the fuse's. */
    planner_cycle_decision_t d = planner_cycle_gate(&st, true, false, true);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_BUDGET_ANNOUNCE);
    ASSERT(st.budget_announced);
    ASSERT(!st.dormant_announced);
    for (int i = 0; i < 5; i++) {
        d = planner_cycle_gate(&st, i % 2 == 0, false, i % 3 == 0);
        ASSERT(d.action == PLANNER_CYCLE_SKIP_BUDGET_SILENT);
    }
}

TEST(test_waking_is_announced_once_after_dormancy)
{
    planner_cycle_state_t st = {0};
    (void)planner_cycle_gate(&st, false, true, true);
    planner_cycle_decision_t d = planner_cycle_gate(&st, true, true, false);
    ASSERT(d.action == PLANNER_CYCLE_REQUEST);
    ASSERT(d.announce_waking);
    ASSERT(!st.dormant_announced && !st.budget_announced);
    d = planner_cycle_gate(&st, true, true, false);
    ASSERT(d.action == PLANNER_CYCLE_REQUEST);
    ASSERT(!d.announce_waking);
    /* And dormancy can be announced again afterwards. */
    d = planner_cycle_gate(&st, false, true, true);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE);
}

TEST(test_waking_is_announced_after_the_fuse_is_resumed)
{
    planner_cycle_state_t st = {0};
    (void)planner_cycle_gate(&st, true, false, false);
    const planner_cycle_decision_t d = planner_cycle_gate(&st, true, true, false);
    ASSERT(d.action == PLANNER_CYCLE_REQUEST);
    ASSERT(d.announce_waking);
    ASSERT(!st.budget_announced);
}

TEST(test_no_waking_line_without_a_prior_announcement)
{
    planner_cycle_state_t st = {0};
    const planner_cycle_decision_t d = planner_cycle_gate(&st, true, true, false);
    ASSERT(d.action == PLANNER_CYCLE_REQUEST);
    ASSERT(!d.announce_waking);
}

/* decide() reads the real modules: a sleeping plan_activity is dormant, and a
 * tripped plan_budget is unaffordable. */
TEST(test_decide_makes_no_request_while_dormant)
{
    planner_cycle_state_t st = {0};
    plan_activity_sleep();
    const plan_evidence_t ev = {0}; /* invalid frame, no audio, no range: no evidence */
    planner_cycle_decision_t d = planner_cycle_decide(&st, &ev, 1000);
    ASSERT(!d.warranted);
    ASSERT(d.affordable);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE);
    d = planner_cycle_decide(&st, &ev, 16000);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_DORMANT_QUIET);

    /* A console command wakes it the way main.c does: the next cycle requests
     * and carries the waking line. */
    plan_activity_wake();
    d = planner_cycle_decide(&st, &ev, 31000);
    ASSERT(d.warranted);
    ASSERT(d.action == PLANNER_CYCLE_REQUEST);
    ASSERT(d.announce_waking);
}

TEST(test_decide_makes_no_request_with_the_fuse_tripped)
{
    planner_cycle_state_t st = {0};
    plan_budget_configure(1, 0);
    plan_budget_note(100, true);
    ASSERT(!plan_budget_allows());
    plan_activity_wake(); /* warranted, so only the fuse holds it */
    const plan_evidence_t ev = {0};
    planner_cycle_decision_t d = planner_cycle_decide(&st, &ev, 1000);
    ASSERT(d.warranted);
    ASSERT(!d.affordable);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_BUDGET_ANNOUNCE);
    d = planner_cycle_decide(&st, &ev, 16000);
    ASSERT(d.action == PLANNER_CYCLE_SKIP_BUDGET_SILENT);
}

/* ---- result handling and backoff ------------------------------------------ */

TEST(test_backoff_doubles_to_the_cap_and_resets_on_success)
{
    planner_cycle_state_t st = {0};
    ASSERT(planner_cycle_wait_periods(&st) == 1u);
    const uint32_t expected[] = {1u, 2u, 4u, PLANNER_MAX_BACKOFF_PERIODS,
                                 PLANNER_MAX_BACKOFF_PERIODS};
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        ASSERT(planner_cycle_result(&st, ESP_FAIL) == PLANNER_RESULT_STOP_BACKOFF);
        ASSERT(st.backoff_periods == expected[i]);
        ASSERT(planner_cycle_wait_periods(&st) == 1u + expected[i]);
    }
    ASSERT(planner_cycle_result(&st, ESP_OK) == PLANNER_RESULT_WRITE_GOAL);
    ASSERT(st.backoff_periods == 0u);
    ASSERT(planner_cycle_wait_periods(&st) == 1u);
    ASSERT(planner_cycle_result(&st, ESP_ERR_TIMEOUT) == PLANNER_RESULT_STOP_BACKOFF);
    ASSERT(st.backoff_periods == 1u);
}

TEST(test_no_link_or_key_forces_stop_without_backoff)
{
    planner_cycle_state_t st = {0};
    ASSERT(planner_cycle_result(&st, ESP_ERR_INVALID_STATE) == PLANNER_RESULT_STOP_IDLE);
    ASSERT(st.backoff_periods == 0u);
    /* Nor does it disturb a backoff already in progress. */
    (void)planner_cycle_result(&st, ESP_FAIL);
    (void)planner_cycle_result(&st, ESP_FAIL);
    ASSERT(planner_cycle_result(&st, ESP_ERR_INVALID_STATE) == PLANNER_RESULT_STOP_IDLE);
    ASSERT(st.backoff_periods == 2u);
}

/* ---- speech screening ------------------------------------------------------ */

static const char k_line[] = "A red ball has rolled under the chair.";

TEST(test_no_line_is_none)
{
    const planner_speech_snapshot_t s = {0};
    ASSERT(planner_cycle_screen_speech("", &s, &s) == PLANNER_SPEECH_NONE);
    ASSERT(planner_cycle_screen_speech(NULL, &s, &s) == PLANNER_SPEECH_NONE);
}

TEST(test_quiet_request_posts_the_line)
{
    const planner_speech_snapshot_t at = {.speech_notes = 3, .voice_turns = 7};
    const planner_speech_snapshot_t now = {.speech_notes = 3, .voice_turns = 7};
    ASSERT(planner_cycle_screen_speech(k_line, &at, &now) == PLANNER_SPEECH_POST);
}

TEST(test_line_dropped_when_a_voice_turn_is_busy)
{
    const planner_speech_snapshot_t at = {.speech_notes = 3, .voice_turns = 7};
    const planner_speech_snapshot_t now = {.speech_notes = 3, .voice_turns = 7, .voice_busy = true};
    ASSERT(planner_cycle_screen_speech(k_line, &at, &now) == PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE);
}

TEST(test_line_dropped_when_a_voice_turn_ran_during_the_request)
{
    const planner_speech_snapshot_t at = {.speech_notes = 3, .voice_turns = 7};
    const planner_speech_snapshot_t now = {.speech_notes = 3, .voice_turns = 8};
    ASSERT(planner_cycle_screen_speech(k_line, &at, &now) == PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE);
    /* The count wraps; equality is all that is compared. */
    const planner_speech_snapshot_t at_wrap = {.voice_turns = UINT32_MAX};
    const planner_speech_snapshot_t now_wrap = {.voice_turns = 0};
    ASSERT(planner_cycle_screen_speech(k_line, &at_wrap, &now_wrap) ==
           PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE);
}

TEST(test_line_dropped_when_something_was_spoken_during_the_request)
{
    const planner_speech_snapshot_t at = {.speech_notes = 3, .voice_turns = 7};
    const planner_speech_snapshot_t now = {.speech_notes = 4, .voice_turns = 7};
    ASSERT(planner_cycle_screen_speech(k_line, &at, &now) == PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE);
}

TEST(test_near_repeat_is_suppressed_before_the_person_check)
{
    dialogue_style_note_spoken(k_line);
    const planner_speech_snapshot_t at = {0};
    const planner_speech_snapshot_t quiet = {0};
    const planner_speech_snapshot_t busy = {.voice_busy = true};
    ASSERT(planner_cycle_screen_speech(k_line, &at, &quiet) == PLANNER_SPEECH_SUPPRESS_REPEAT);
    ASSERT(planner_cycle_screen_speech(k_line, &at, &busy) == PLANNER_SPEECH_SUPPRESS_REPEAT);
    ASSERT(planner_cycle_screen_speech("The hallway light just switched on.", &at, &quiet) ==
           PLANNER_SPEECH_POST);
}

/* A first impression of the room and the view: both detectors novel. */
static void stage_novel_view_and_sound(uint32_t now)
{
    uint8_t luma[32 * 24];
    for (size_t i = 0; i < sizeof(luma); i++) {
        luma[i] = (uint8_t)((i * 37u) % 251u);
    }
    scene_fingerprint_t fp;
    scene_fingerprint_from_luma(luma, 32, 24, &fp);
    scene_change_note(&fp);

    static int16_t pcm[AMBIENT_FRAME_SAMPLES];
    for (size_t i = 0; i < AMBIENT_FRAME_SAMPLES; i++) {
        pcm[i] = (int16_t)(((int)((i * 7919u) % 2001u) - 1000) * 8);
    }
    ambient_fingerprint_t afp;
    ambient_fingerprint_from_pcm(pcm, AMBIENT_FRAME_SAMPLES, &afp);
    ambient_audio_note(&afp, now);
}

TEST(test_a_posted_line_re_references_the_view_and_the_soundscape)
{
    const uint32_t now = 100000;
    stage_novel_view_and_sound(now);
    ASSERT(scene_change_novel());
    ASSERT(ambient_audio_novel(now));

    planner_cycle_note_spoken(k_line, now);

    /* Both detectors measure "since the robot last spoke", so the same view and
     * the same soundscape are no longer news to either (voice.md). */
    ASSERT(!scene_change_novel());
    ASSERT(scene_change_score() == 0u);
    ASSERT(!ambient_audio_novel(now + 1u));
}

TEST(test_a_posted_line_is_marked_spoken)
{
    const uint32_t now = 100000;
    ASSERT(speech_budget_allows(now));
    const uint32_t notes_before = speech_budget_notes();
    const planner_speech_snapshot_t at = {.speech_notes = notes_before};

    planner_cycle_note_spoken(k_line, now);

    /* The speaking budget is spent: notes moved and the minimum gap holds. */
    ASSERT(speech_budget_notes() == notes_before + 1u);
    ASSERT(!speech_budget_allows(now + 1u));
    ASSERT(strcmp(planner_cycle_gate_verdict(now + 1u), "-") == 0);
    /* It reached the recall ring, so saying it again is a repeat. */
    const planner_speech_snapshot_t same = at;
    ASSERT(planner_cycle_screen_speech(k_line, &at, &same) == PLANNER_SPEECH_SUPPRESS_REPEAT);
    /* And a later request built before this note would see it as speech
     * during the request. */
    const planner_speech_snapshot_t after = {.speech_notes = speech_budget_notes()};
    ASSERT(planner_cycle_screen_speech("Something else entirely, out the window.", &at, &after) ==
           PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE);
}

TEST(test_after_request_returns_the_gate_verdict_at_that_moment)
{
    plan_activity_wake();
    const plan_evidence_t ev = {0};
    ASSERT(plan_activity_should_call(&ev, 1000));
    planner_cycle_note_spoken(k_line, 1000);
    /* Within the speaking gap the budget holds, whatever the detectors say. */
    ASSERT(strcmp(planner_cycle_after_request(1000, 2000), "-") == 0);
    ASSERT(strcmp(planner_cycle_after_request(1000, 2000), planner_cycle_gate_verdict(2000)) == 0);
}

TEST(test_after_request_notes_the_call)
{
    const plan_evidence_t ev = {0}; /* no evidence beyond the console wake */
    plan_activity_wake();
    ASSERT(plan_activity_should_call(&ev, 1000));
    (void)planner_cycle_after_request(1000, 2000);
    /* Noted: the next keep-alive waits a full base period from the request.
     * Without the note the planner has never called, and asks again at once. */
    ASSERT(!plan_activity_should_call(&ev, 2000));
    ASSERT(!plan_activity_should_call(&ev, 1000 + PLANNER_LOOP_PERIOD_MS - 1u));

    /* A keep-alive request (no evidence) climbs one rung when it is noted —
     * whatever its outcome, since after_request runs before the result. */
    const uint32_t t1 = 1000 + PLANNER_LOOP_PERIOD_MS;
    ASSERT(plan_activity_should_call(&ev, t1));
    ASSERT(plan_activity_step() == 0u);
    (void)planner_cycle_after_request(t1, t1 + 300u);
    ASSERT(plan_activity_step() == 1u);
    ASSERT(plan_activity_period_ms() == 2u * PLANNER_LOOP_PERIOD_MS);
    ASSERT(!plan_activity_should_call(&ev, t1 + PLANNER_LOOP_PERIOD_MS));
    ASSERT(plan_activity_should_call(&ev, t1 + 2u * PLANNER_LOOP_PERIOD_MS));
}

/* ---- note gate triggers (ADR-033) ---------------------------------------- */

TEST(test_a_reflex_trip_offers_a_note_once_then_waits_for_the_window)
{
    planner_cycle_state_t st = {0};
    planner_note_inputs_t in = {.reflex_trips = 0, .blind = false};
    ASSERT(planner_cycle_note_offer(&st, &in, 1000) == NOTE_TRIGGER_NONE);

    in.reflex_trips = 1; /* braked and released between two requests */
    ASSERT(planner_cycle_note_offer(&st, &in, 16000) == NOTE_TRIGGER_REFLEX);
    ASSERT(planner_cycle_note_offer(&st, &in, 31000) == NOTE_TRIGGER_NONE);

    in.reflex_trips = 2; /* again, inside the five-minute window */
    ASSERT(planner_cycle_note_offer(&st, &in, 46000) == NOTE_TRIGGER_NONE);
    ASSERT(planner_cycle_note_offer(&st, &in, 16000 + NOTE_GATE_MIN_INTERVAL_MS) ==
           NOTE_TRIGGER_REFLEX);
}

TEST(test_blind_is_raised_once_per_spell)
{
    planner_cycle_state_t st = {0};
    planner_note_inputs_t in = {.reflex_trips = 0, .blind = true};
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_BLIND);
    uint32_t t = NOTE_GATE_MIN_INTERVAL_MS;
    ASSERT(planner_cycle_note_offer(&st, &in, t) == NOTE_TRIGGER_NONE); /* same spell */
    in.blind = false;
    (void)planner_cycle_note_offer(&st, &in, t += NOTE_GATE_MIN_INTERVAL_MS);
    in.blind = true; /* a new spell */
    ASSERT(planner_cycle_note_offer(&st, &in, t += NOTE_GATE_MIN_INTERVAL_MS) ==
           NOTE_TRIGGER_BLIND);
}

TEST(test_a_track_streak_raises_track_stuck)
{
    planner_cycle_state_t st = {0};
    const planner_note_inputs_t in = {0};
    for (uint32_t i = 0; i + 1 < NOTE_GATE_TRACK_STUCK_CYCLES; i++) {
        planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_TRACK,
                                   PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    }
    /* A different goal breaks the streak. */
    planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP, PLANNER_SPEECH_NONE,
                               NOTE_TRIGGER_NONE);
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_NONE);
    for (uint32_t i = 0; i < NOTE_GATE_TRACK_STUCK_CYCLES; i++) {
        planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_TRACK,
                                   PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    }
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_TRACK_STUCK);
}

TEST(test_a_failed_request_breaks_the_track_streak)
{
    planner_cycle_state_t st = {0};
    const planner_note_inputs_t in = {0};
    for (uint32_t i = 0; i < NOTE_GATE_TRACK_STUCK_CYCLES; i++) {
        planner_cycle_note_outcome(&st,
                                   i == 1 ? PLANNER_RESULT_STOP_BACKOFF : PLANNER_RESULT_WRITE_GOAL,
                                   GOAL_KIND_TRACK, PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    }
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_NONE);
}

TEST(test_repeated_repeats_raise_repeats_and_a_spoken_line_resets)
{
    planner_cycle_state_t st = {0};
    const planner_note_inputs_t in = {0};
    planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP,
                               PLANNER_SPEECH_SUPPRESS_REPEAT, NOTE_TRIGGER_NONE);
    planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP, PLANNER_SPEECH_POST,
                               NOTE_TRIGGER_NONE);
    for (uint32_t i = 0; i + 1 < NOTE_GATE_REPEAT_STREAK; i++) {
        planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP,
                                   PLANNER_SPEECH_SUPPRESS_REPEAT, NOTE_TRIGGER_NONE);
        /* A cycle with no line does not break it. */
        planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP,
                                   PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    }
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_NONE);
    planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP,
                               PLANNER_SPEECH_SUPPRESS_REPEAT, NOTE_TRIGGER_NONE);
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_REPEATS);
}

TEST(test_every_nth_request_raises_a_reflection)
{
    planner_cycle_state_t st = {0};
    const planner_note_inputs_t in = {0};
    for (uint32_t i = 0; i + 1 < NOTE_GATE_REFLECT_EVERY; i++) {
        planner_cycle_note_outcome(&st, PLANNER_RESULT_WRITE_GOAL, GOAL_KIND_STOP,
                                   PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    }
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_NONE);
    planner_cycle_note_outcome(&st, PLANNER_RESULT_STOP_BACKOFF, GOAL_KIND_STOP,
                               PLANNER_SPEECH_NONE, NOTE_TRIGGER_NONE);
    ASSERT(planner_cycle_note_offer(&st, &in, 0) == NOTE_TRIGGER_REFLECT);
}

TEST(test_an_offline_request_gives_the_offer_back)
{
    planner_cycle_state_t st = {0};
    planner_note_inputs_t in = {.reflex_trips = 1, .blind = false};
    const note_trigger_t offered = planner_cycle_note_offer(&st, &in, 0);
    ASSERT(offered == NOTE_TRIGGER_REFLEX);
    planner_cycle_note_outcome(&st, PLANNER_RESULT_STOP_IDLE, GOAL_KIND_STOP, PLANNER_SPEECH_NONE,
                               offered);
    ASSERT(note_gate_offers() == 0);
    /* The link is back on the next tick: the same reflex is offered. */
    ASSERT(planner_cycle_note_offer(&st, &in, 15000) == NOTE_TRIGGER_REFLEX);
}

int main(void)
{
    printf("planner_cycle_core host tests\n");
    run_test_dormant_announces_once_then_stays_quiet();
    run_test_climbing_the_ladder_logs_idle_and_sets_no_latch();
    run_test_blown_fuse_announces_once_and_outranks_dormancy();
    run_test_waking_is_announced_once_after_dormancy();
    run_test_waking_is_announced_after_the_fuse_is_resumed();
    run_test_no_waking_line_without_a_prior_announcement();
    run_test_decide_makes_no_request_while_dormant();
    run_test_decide_makes_no_request_with_the_fuse_tripped();
    run_test_backoff_doubles_to_the_cap_and_resets_on_success();
    run_test_no_link_or_key_forces_stop_without_backoff();
    run_test_no_line_is_none();
    run_test_quiet_request_posts_the_line();
    run_test_line_dropped_when_a_voice_turn_is_busy();
    run_test_line_dropped_when_a_voice_turn_ran_during_the_request();
    run_test_line_dropped_when_something_was_spoken_during_the_request();
    run_test_near_repeat_is_suppressed_before_the_person_check();
    run_test_a_posted_line_re_references_the_view_and_the_soundscape();
    run_test_a_posted_line_is_marked_spoken();
    run_test_after_request_returns_the_gate_verdict_at_that_moment();
    run_test_after_request_notes_the_call();
    run_test_a_reflex_trip_offers_a_note_once_then_waits_for_the_window();
    run_test_blind_is_raised_once_per_spell();
    run_test_a_track_streak_raises_track_stuck();
    run_test_a_failed_request_breaks_the_track_streak();
    run_test_repeated_repeats_raise_repeats_and_a_spoken_line_resets();
    run_test_every_nth_request_raises_a_reflection();
    run_test_an_offline_request_gives_the_offer_back();

    if (g_failures) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("All planner_cycle_core tests passed\n");
    return 0;
}
