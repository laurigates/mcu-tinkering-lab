/**
 * @file planner_cycle_core.c
 * @brief The planner's per-cycle decisions. See planner_cycle_core.h.
 *
 * No FreeRTOS, camera, HTTP or logging: everything here is decided from the
 * pure modules and from snapshots the caller passes in, so it compiles
 * unchanged for the host tests and the behaviour twin.
 */

#include "planner_cycle_core.h"

#include <stddef.h>

#include "ambient_audio.h"
#include "dialogue_style.h"
#include "plan_budget.h"
#include "planner_task.h" /* PLANNER_MAX_BACKOFF_PERIODS */
#include "scene_change.h"
#include "speech_budget.h"

planner_cycle_decision_t planner_cycle_gate(planner_cycle_state_t *st, bool warranted,
                                            bool affordable, bool dormant)
{
    planner_cycle_decision_t d = {
        .action = PLANNER_CYCLE_REQUEST,
        .warranted = warranted,
        .affordable = affordable,
        .announce_waking = false,
    };

    if (!warranted || !affordable) {
        /* No request. The goal is deliberately left to expire on its own TTL
         * rather than being force-stopped every skipped cycle: the executor
         * already treats a stale goal as a hold, and re-stopping a stopped
         * robot 240 times an hour would bury the one transition that matters
         * in the log. */
        if (!affordable) {
            /* The fuse outranks everything else in this branch: while it is
             * blown there is no "next request in N ms" to report, and saying
             * there is would describe a cadence that will never run. Reported
             * once, then silent — an exhausted budget does not become news
             * again every 15 s. */
            if (!st->budget_announced) {
                st->budget_announced = true;
                d.action = PLANNER_CYCLE_SKIP_BUDGET_ANNOUNCE;
            } else {
                d.action = PLANNER_CYCLE_SKIP_BUDGET_SILENT;
            }
        } else if (dormant && !st->dormant_announced) {
            st->dormant_announced = true;
            d.action = PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE;
        } else if (!dormant) {
            /* Climbing the ladder: the interesting phase, and the one whose
             * thresholds get tuned from a capture. Dormant cycles are NOT
             * logged at INFO — an overnight run would otherwise emit thousands
             * of identical lines. */
            d.action = PLANNER_CYCLE_SKIP_IDLE;
        } else {
            d.action = PLANNER_CYCLE_SKIP_DORMANT_QUIET;
        }
        return d;
    }

    if (st->dormant_announced || st->budget_announced) {
        d.announce_waking = true;
        st->dormant_announced = false;
        st->budget_announced = false;
    }
    return d;
}

planner_cycle_decision_t planner_cycle_decide(planner_cycle_state_t *st, const plan_evidence_t *ev,
                                              uint32_t now_ms)
{
    const bool warranted = plan_activity_should_call(ev, now_ms);
    const bool affordable = plan_budget_allows();
    /* Read after should_call(), which is what can enter dormancy. Only consulted
     * on a skipped cycle, as planner_task.c always did. */
    const bool dormant = (!warranted || !affordable) ? plan_activity_dormant() : false;
    return planner_cycle_gate(st, warranted, affordable, dormant);
}

/*
 * This exists so the OR in gemini_backend.c can be judged from a capture instead
 * of defended from an armchair. Count the "A"-only cycles in a 20-minute
 * `just robocar::monitor | tee` run: if they track the HVAC, the fix is
 * `voice loud` / `voice sound`; if they coincide with sentences worth hearing, the
 * OR is vindicated; if "A" never appears at all, the audio gate is dead weight and
 * either the thresholds or the mic gain are wrong.
 */
const char *planner_cycle_gate_verdict(uint32_t now_ms)
{
    if (!speech_budget_allows(now_ms)) {
        return "-";
    }
    const bool v = scene_change_novel();
    const bool a = ambient_audio_novel(now_ms);
    if (v && a) {
        return "VA";
    }
    if (v) {
        return "V";
    }
    if (a) {
        return "A";
    }
    return ".";
}

const char *planner_cycle_after_request(uint32_t request_ms, uint32_t now_ms)
{
    /* Noted whatever the outcome: the request went out, so it moves the
     * reference to the view just planned on and advances the ladder. Noting
     * only successes would let a run of failures replan the same unchanged
     * scene at full rate forever. */
    plan_activity_note_call(request_ms);

    /* Which gate opened (or held) THIS cycle, sampled here and not at the log
     * line, because mark_spoken() re-references both detectors in between.
     * Reading them afterwards would report the state the next cycle starts
     * from and make every successful utterance look like it came from a closed
     * gate. */
    return planner_cycle_gate_verdict(now_ms);
}

planner_speech_verdict_t planner_cycle_screen_speech(const char *speech,
                                                     const planner_speech_snapshot_t *at_request,
                                                     const planner_speech_snapshot_t *now)
{
    if (speech == NULL || speech[0] == '\0') {
        return PLANNER_SPEECH_NONE;
    }
    /* The prompt already carries what was recently said and asks for something
     * new; this re-checks the answer, because a request is not a guarantee.
     * See dialogue_style.h. */
    if (dialogue_style_is_repetitive(speech)) {
        return PLANNER_SPEECH_SUPPRESS_REPEAT;
    }
    /* A person spoke while this request was in flight: a voice turn is still
     * running, one began since the request, or something was spoken. The
     * request was built from the state before that, so its `speak` licence and
     * any "the sound changed" clause are about the person's own voice, and
     * speaking now is the "some strange noise" remark on top of the reply
     * (2026-11-05: turn recorded at 138.6 s, request at ~139.7 s, noise line at
     * 144.9 s, the turn's reply at 146.4 s). The goal still applies; only the
     * line is dropped. */
    if (now->voice_busy || now->voice_turns != at_request->voice_turns ||
        now->speech_notes != at_request->speech_notes) {
        return PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE;
    }
    return PLANNER_SPEECH_POST;
}

void planner_cycle_note_spoken(const char *speech, uint32_t now_ms)
{
    /* Feeds the next prompt's recall list and avoid-list, and spends the
     * speaking budget. Only on a successful post: a line dropped by a full
     * queue is never heard, so it is neither a repetition anyone can notice nor
     * an utterance worth rationing. */
    dialogue_style_note_spoken(speech);
    speech_budget_note(now_ms);
    /* This view is now the one the robot has remarked on, so it becomes the
     * reference the next frames are measured against. The soundscape is
     * re-referenced for the same reason and at the same moment: both detectors
     * measure "since the robot last spoke", so they must be marked together or
     * the surviving one keeps licensing remarks about evidence already used. */
    scene_change_mark_spoken();
    ambient_audio_mark_spoken();
}

planner_result_action_t planner_cycle_result(planner_cycle_state_t *st, esp_err_t ret)
{
    if (ret == ESP_OK) {
        st->backoff_periods = 0;
        return PLANNER_RESULT_WRITE_GOAL;
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        /* No WiFi link (issue #846) or no key yet (issue #845): nothing was
         * sent and nothing spent, so no backoff, and the first cycle after the
         * link or the key arrives plans at once. */
        return PLANNER_RESULT_STOP_IDLE;
    }
    st->backoff_periods = (st->backoff_periods == 0) ? 1
                          : (st->backoff_periods < PLANNER_MAX_BACKOFF_PERIODS)
                              ? st->backoff_periods * 2
                              : PLANNER_MAX_BACKOFF_PERIODS;
    return PLANNER_RESULT_STOP_BACKOFF;
}

uint32_t planner_cycle_wait_periods(const planner_cycle_state_t *st)
{
    return 1u + st->backoff_periods;
}

note_trigger_t planner_cycle_note_offer(planner_cycle_state_t *st, const planner_note_inputs_t *in,
                                        uint32_t now_ms)
{
    /* Compared by inequality, not "greater than": the count only grows, and
     * any difference means at least one trip since the last look. */
    if (in->reflex_trips != st->note_reflex_seen) {
        st->note_reflex_seen = in->reflex_trips;
        note_gate_raise(NOTE_TRIGGER_REFLEX);
    }
    if (in->blind && !st->note_blind_raised) {
        st->note_blind_raised = true;
        note_gate_raise(NOTE_TRIGGER_BLIND);
    } else if (!in->blind) {
        st->note_blind_raised = false;
    }
    return note_gate_take(now_ms);
}

void planner_cycle_note_outcome(planner_cycle_state_t *st, planner_result_action_t result,
                                goal_kind_t goal_kind, planner_speech_verdict_t speech,
                                note_trigger_t offered)
{
    if (result == PLANNER_RESULT_STOP_IDLE) {
        /* Nothing left the board, so the model never saw the offer, and a
         * robot offline for an hour would otherwise burn every trigger. */
        note_gate_untake(offered);
        return;
    }
    if (++st->note_requests_since_reflect >= NOTE_GATE_REFLECT_EVERY) {
        st->note_requests_since_reflect = 0;
        note_gate_raise(NOTE_TRIGGER_REFLECT);
    }

    if (result == PLANNER_RESULT_WRITE_GOAL && goal_kind == GOAL_KIND_TRACK) {
        if (++st->note_track_streak >= NOTE_GATE_TRACK_STUCK_CYCLES) {
            st->note_track_streak = 0;
            note_gate_raise(NOTE_TRIGGER_TRACK_STUCK);
        }
    } else {
        st->note_track_streak = 0;
    }

    if (speech == PLANNER_SPEECH_SUPPRESS_REPEAT) {
        if (++st->note_repeat_streak >= NOTE_GATE_REPEAT_STREAK) {
            st->note_repeat_streak = 0;
            note_gate_raise(NOTE_TRIGGER_REPEATS);
        }
    } else if (speech == PLANNER_SPEECH_POST) {
        st->note_repeat_streak = 0;
    }
}
