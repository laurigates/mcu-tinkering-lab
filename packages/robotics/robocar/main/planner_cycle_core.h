/**
 * @file planner_cycle_core.h
 * @brief The planner's per-cycle decisions, free of FreeRTOS, the camera, HTTP and logging.
 *
 * planner_task.c owns the I/O of one planner cycle: capture a frame, call
 * Gemini, post a line to the speech queue, write a goal, log, sleep. What it
 * decides in between — whether this cycle may make a request at all, which
 * one-shot announcement is due, whether the model's line may be spoken, what a
 * result code means for the goal and the backoff — lives here, so the same
 * compiled decisions run on the robot, in the host tests and in the behaviour
 * twin (ADR-030, PRD-017).
 *
 * The functions read and write the pure modules the decisions depend on
 * (plan_activity, plan_budget, scene_change, ambient_audio, speech_budget,
 * dialogue_style), which are host-compilable already. What is not — the voice
 * turn task and the speech queue — reaches this module as explicit snapshots
 * and return codes. Every log line stays in planner_task.c, driven by the
 * decision a function returns, so the log is unchanged by the extraction.
 *
 * One cycle, in the order planner_task.c calls it:
 *
 *   1. planner_cycle_decide()        request or skip, and what to announce
 *   2. (snapshot speech_budget_notes() and voice_turn_count(), make the request)
 *   3. planner_cycle_after_request() note the call, sample the speak gate
 *   4. planner_cycle_screen_speech() speak the line or say why not
 *   5. planner_cycle_note_spoken()   only after speech_queue_post() succeeded
 *   6. planner_cycle_result()        goal write vs force-stop, backoff
 *   7. planner_cycle_wait_periods()  how many periods to sleep
 *
 * The note gate (note_gate.h, ADR-033) hooks in on both sides of the request:
 * planner_cycle_note_offer() between 1 and 2 decides whether this request
 * offers the `note` tool, and planner_cycle_note_outcome() after 6 raises the
 * triggers the cycle's result supports.
 */

#ifndef PLANNER_CYCLE_CORE_H
#define PLANNER_CYCLE_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "goal_state.h"
#include "note_gate.h"
#include "plan_activity.h"

/** State the planner keeps from one cycle to the next. Zero-initialise it. */
typedef struct {
    /** Extra periods to wait after a failed plan. Gemini answers a quota
     *  overrun with HTTP 429 and a retryDelay that *grows* while the client
     *  keeps asking (observed climbing 19 s -> 31 s under a 1 Hz loop), so
     *  retrying at full rate makes the outage longer. Capped at
     *  PLANNER_MAX_BACKOFF_PERIODS so recovery stays bounded; reset on the
     *  first success. */
    uint32_t backoff_periods;

    /** One-shot latches so entering dormancy and tripping the fuse are
     *  announced once each rather than every 15 s for the rest of the night —
     *  and so the matching "waking" line has something to be the counterpart
     *  of. */
    bool dormant_announced;
    bool budget_announced;

    /** Note gate evidence (planner_cycle_note_offer/_outcome). The reflex count
     *  last compared, so a trip between two requests is seen even when the
     *  brake has already released. Zero-initialised to match the executor's
     *  count at boot. */
    uint32_t note_reflex_seen;
    /** BLIND was raised for the current blind spell; cleared when a frame
     *  decodes, so the next spell raises it again. */
    bool note_blind_raised;
    uint32_t note_track_streak;
    uint32_t note_repeat_streak;
    uint32_t note_requests_since_reflect;
} planner_cycle_state_t;

/** What a cycle does before (or instead of) the request. */
typedef enum {
    /** Make the request. */
    PLANNER_CYCLE_REQUEST = 0,
    /** No request: the fuse just blew. Announce it once and force a stop. */
    PLANNER_CYCLE_SKIP_BUDGET_ANNOUNCE,
    /** No request: the fuse is still blown and was announced. Say nothing. */
    PLANNER_CYCLE_SKIP_BUDGET_SILENT,
    /** No request: the planner just went dormant. Announce it once and force a
     *  stop. */
    PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE,
    /** No request: climbing the ladder. Log the scores beside their thresholds. */
    PLANNER_CYCLE_SKIP_IDLE,
    /** No request: dormant and already announced. Debug-level log only. */
    PLANNER_CYCLE_SKIP_DORMANT_QUIET,
} planner_cycle_action_t;

typedef struct {
    planner_cycle_action_t action;
    /** plan_activity_should_call() for this cycle's evidence. */
    bool warranted;
    /** plan_budget_allows(). */
    bool affordable;
    /** Only with PLANNER_CYCLE_REQUEST: the planner was dormant or out of
     *  budget and announced it, so this request is the counterpart "waking"
     *  line. The latches are already cleared when this is set. */
    bool announce_waking;
} planner_cycle_decision_t;

/**
 * @brief Decide from explicit gate results; no module is read.
 *
 * The decision table behind planner_cycle_decide(), exposed so the latches can
 * be tested and replayed without staging plan_activity and plan_budget.
 *
 * @param st          Cycle state; the announce latches are updated.
 * @param warranted   plan_activity_should_call() for this cycle.
 * @param affordable  plan_budget_allows().
 * @param dormant     plan_activity_dormant(), read after should_call().
 */
planner_cycle_decision_t planner_cycle_gate(planner_cycle_state_t *st, bool warranted,
                                            bool affordable, bool dormant);

/**
 * @brief Decide whether this cycle is worth a request at all.
 *
 * Calls plan_activity_should_call(ev, now_ms), then plan_budget_allows(), then
 * plan_activity_dormant(), and applies planner_cycle_gate(). The evidence loop
 * runs every tick regardless; only the network call is gated.
 */
planner_cycle_decision_t planner_cycle_decide(planner_cycle_state_t *st, const plan_evidence_t *ev,
                                              uint32_t now_ms);

/**
 * @brief Which gate decides whether to SPEAK, as a short literal for the log.
 *
 * "-" the budget held (rationed, regardless of evidence), "V" the view changed,
 * "A" the room sounded different, "VA" both, "." neither — nothing to say.
 *
 * Returns a STRING LITERAL, and callers must pass it as a %s ARGUMENT. A ternary
 * in the format position expands into a syntax error pointing inside
 * esp_log_color.h, far from the cause — see .claude/rules/esp-log-format-literal.md.
 */
const char *planner_cycle_gate_verdict(uint32_t now_ms);

/**
 * @brief Book-keeping right after the request returns, whatever its outcome.
 *
 * Notes the call with plan_activity_note_call(request_ms), then samples and
 * returns planner_cycle_gate_verdict(now_ms). note_call touches none of the
 * verdict's inputs, so their order here does not matter. What does: the verdict
 * must be sampled before planner_cycle_note_spoken(), which re-references both
 * detectors, and the caller keeps that order by calling this first.
 *
 * @param request_ms  The cycle's timestamp, taken before the request.
 * @param now_ms      The time now, after the request's latency.
 */
const char *planner_cycle_after_request(uint32_t request_ms, uint32_t now_ms);

/** The counters that tell whether a person spoke while a request was in flight. */
typedef struct {
    /** speech_budget_notes(). */
    uint32_t speech_notes;
    /** voice_turn_count(). */
    uint32_t voice_turns;
    /** voice_turn_is_busy(); ignored in the at-request snapshot. */
    bool voice_busy;
} planner_speech_snapshot_t;

/** What to do with the line the model asked to speak. */
typedef enum {
    /** No line. */
    PLANNER_SPEECH_NONE = 0,
    /** Post it to the speech queue. */
    PLANNER_SPEECH_POST,
    /** dialogue_style_is_repetitive(): a near-repeat of a recent line. */
    PLANNER_SPEECH_SUPPRESS_REPEAT,
    /** A voice turn is running, one began, or something was spoken since the
     *  request was built. */
    PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE,
} planner_speech_verdict_t;

/**
 * @brief Screen the model's line before it reaches the speech queue.
 *
 * Repetition is checked first, then whether a person spoke during the request.
 * Only generated lines are screened; console auditions and status reports go
 * straight to the queue.
 *
 * @param speech      The line; NULL or empty means none.
 * @param at_request  Snapshot taken before the request (voice_busy ignored).
 * @param now         Snapshot taken after the request returned.
 */
planner_speech_verdict_t planner_cycle_screen_speech(const char *speech,
                                                     const planner_speech_snapshot_t *at_request,
                                                     const planner_speech_snapshot_t *now);

/**
 * @brief Record a line that speech_queue_post() accepted.
 *
 * Feeds dialogue_style's recall list, spends the speaking budget at @p now_ms,
 * and re-references the scene and the soundscape together. Call only on a
 * successful post: a line dropped by a full queue was never heard.
 */
void planner_cycle_note_spoken(const char *speech, uint32_t now_ms);

/** What the request's result means for the goal. */
typedef enum {
    /** ESP_OK: write the parsed goal. Backoff reset. */
    PLANNER_RESULT_WRITE_GOAL = 0,
    /** ESP_ERR_INVALID_STATE: no WiFi link or no key. Force a stop, no backoff:
     *  nothing was sent and nothing spent. */
    PLANNER_RESULT_STOP_IDLE,
    /** Any other error: force a stop and back off (state.backoff_periods). */
    PLANNER_RESULT_STOP_BACKOFF,
} planner_result_action_t;

/**
 * @brief Map the request's result to a goal action and update the backoff.
 *
 * Backoff goes 0 -> 1 -> 2 -> 4 ... and holds at PLANNER_MAX_BACKOFF_PERIODS;
 * success resets it to 0; INVALID_STATE leaves it unchanged.
 */
planner_result_action_t planner_cycle_result(planner_cycle_state_t *st, esp_err_t ret);

/** @brief Periods to sleep after a cycle that made a request: 1 + backoff. */
uint32_t planner_cycle_wait_periods(const planner_cycle_state_t *st);

/** What the robot already measures that can trigger a note. */
typedef struct {
    /** reactive_telemetry_t.reflex_trips. */
    uint32_t reflex_trips;
    /** !scene_change_has_measurement(): no frame has decoded. */
    bool blind;
} planner_note_inputs_t;

/**
 * @brief Raise the triggers @p in supports, then ask the gate for an offer.
 *
 * Call on a cycle that will make a request, before it is built. A reflex trip
 * since the last call raises NOTE_TRIGGER_REFLEX; the start of a blind spell
 * raises NOTE_TRIGGER_BLIND once. Returns note_gate_take(now_ms): the trigger
 * this request offers the tool for, or NONE.
 */
note_trigger_t planner_cycle_note_offer(planner_cycle_state_t *st, const planner_note_inputs_t *in,
                                        uint32_t now_ms);

/**
 * @brief Raise the triggers this cycle's outcome supports, for a later request.
 *
 * Every NOTE_GATE_REFLECT_EVERY-th request raises NOTE_TRIGGER_REFLECT. A
 * `track` goal written NOTE_GATE_TRACK_STUCK_CYCLES times in a row raises
 * NOTE_TRIGGER_TRACK_STUCK (the streak then starts again, so a target chased
 * for twice as long raises it twice). NOTE_GATE_REPEAT_STREAK lines withheld as
 * repeats in a row raise NOTE_TRIGGER_REPEATS; a line that is spoken resets
 * that streak, and a cycle with no line leaves it.
 *
 * "Not converging" is approximated by the streak: the executor does not report
 * whether a track goal arrived, and a target that is reached stops being
 * tracked.
 *
 * A PLANNER_RESULT_STOP_IDLE request (no link or no key) was never sent: it
 * gives @p offered back to the gate (note_gate_untake()) and counts toward no
 * streak.
 *
 * @param offered  What planner_cycle_note_offer() returned for this request.
 */
void planner_cycle_note_outcome(planner_cycle_state_t *st, planner_result_action_t result,
                                goal_kind_t goal_kind, planner_speech_verdict_t speech,
                                note_trigger_t offered);

#endif /* PLANNER_CYCLE_CORE_H */
