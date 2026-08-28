/**
 * @file plan_activity.h
 * @brief Decides whether this planner cycle is worth a Gemini request at all.
 *
 * Before this module existed, planner_task.c called gemini_backend_plan() on
 * every one of its 15 s ticks, unconditionally and forever — 240 requests an
 * hour whether anything was happening or not. The speech gates
 * (speech_budget.h, scene_change.h, ambient_audio.h) were already computing
 * almost exactly the evidence needed to know better, but they were applied one
 * layer too late: they decide whether the `speak` *tool declaration* goes into
 * the request. On a quiet cycle the request got cheaper. It still went out, and
 * it still carried a JPEG through a vision model.
 *
 * So this module answers the question nothing was asking: *should this request
 * be made?*
 *
 * ## The ladder, and why it is not an on/off switch
 *
 * A verdict of "nothing is happening" comes from on-device sensors, and this
 * project has already been bitten once by trusting one: ambient_audio.c failed
 * OPEN when the microphone was absent and asserted novelty on every cycle for a
 * whole boot. The mirror of that failure — a wake detector that fails CLOSED —
 * is a robot that never wakes up again, which reads as bricked rather than as
 * frugal, and which no amount of standing in front of it fixes.
 *
 * A geometric ladder bounds both directions. While nothing changes the interval
 * between requests grows 15 s -> 30 -> 60 -> 120 -> 300 (PLAN_LADDER_STEPS
 * multipliers of PLANNER_LOOP_PERIOD_MS), and only after the whole ladder is
 * spent — five requests over about nine minutes — does the planner go dormant
 * and stop calling entirely. Any evidence at all resets it to the base period.
 * A detector that is merely *too quiet* therefore costs a few minutes of
 * reduced cadence, not a coma, and the failure is visible in the log as a
 * ladder that keeps climbing while a human is plainly in the room.
 *
 * ## The evidence loop keeps running while dormant
 *
 * Dormancy stops the *network request*, not the loop. Capturing a frame,
 * decoding it to a 40x30 thumbnail and fingerprinting it costs nothing but CPU,
 * so the planner keeps doing exactly that at PLANNER_LOOP_PERIOD_MS while
 * dormant. Wake latency is therefore one tick — a person walking up gets a
 * response in 15 s, not in however long the ladder had grown to.
 *
 * ## The reference is the view the robot last PLANNED on
 *
 * Not the previous frame. Frame-to-frame comparison is blind to slow drift: a
 * room can change completely in steps that each fall under the threshold, and
 * the gate never notices. scene_change.c makes the same argument for the frame
 * last *spoken about* and the project CLAUDE.md forbids re-pointing that one at
 * the previous frame, which is why this module keeps its own reference rather
 * than borrowing that gate's. The two ask different questions of the same
 * fingerprint: "is this worth remarking on?" and "is this worth thinking
 * about?"
 *
 * The comparison inherits scene_change.c's block-mean-minus-frame-mean
 * representation, so the sensor's AGC/AEC rewriting gain and exposure every
 * frame — which it does continuously, on a motionless scene — does not read as
 * a change. An absolute comparison here would wake the robot every time a cloud
 * passed.
 *
 * ## Sensitivity trades tokens against responsiveness, so it is a knob
 *
 * A false wake costs one request. A missed wake costs the robot's usefulness
 * until someone makes a noise or types at the console. The two are not
 * symmetric, so PLAN_SCENE_THRESHOLD_DEFAULT sits *below* scene_change.c's — a
 * view change too small to be worth talking about can still be worth thinking
 * about. But over-sensitivity is not free either: evidence resets the ladder to
 * full rate, so a detector that fires on sensor noise saves nothing at all.
 * Where the balance lies depends on the room and the lens, which is why every
 * threshold here is a `plan` console knob and every score is logged beside the
 * threshold it is being compared against.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_plan_activity.c
 * builds it on the host with no shims, and the cases a bench cannot stage (an
 * exact brightness shift with no motion, the uint32 millisecond wrap at day 49,
 * a camera that has never returned a decodable frame) are one memset each.
 */

#ifndef PLAN_ACTIVITY_H
#define PLAN_ACTIVITY_H

#include <stdbool.h>
#include <stdint.h>

#include "scene_change.h" /* scene_fingerprint_t, scene_fingerprint_distance() */

#ifdef __cplusplus
extern "C" {
#endif

/** Rungs on the backoff ladder. */
#define PLAN_LADDER_STEPS 5

/** Mean absolute block difference from the last-planned view at or above which
 *  the scene counts as worth planning again.
 *
 *  6 rather than scene_change.c's 8 for the asymmetry described in the header:
 *  a missed wake is far more expensive than a spurious one. Still a starting
 *  point and not a measured constant — watch the `still=` field in the planner
 *  log and set `plan scene <n>`. 0 disables this sub-gate (never contributes),
 *  matching the ambient thresholds' polarity rather than scene_change's,
 *  because every term here is ORed. */
#define PLAN_SCENE_THRESHOLD_DEFAULT 6u

/** Change in the ultrasonic reading, in cm, at or above which something has
 *  moved in front of the robot. 20 cm clears the filtered sensor's own jitter
 *  by a wide margin while still catching a person stepping into range.
 *  0 disables this sub-gate. */
#define PLAN_RANGE_THRESHOLD_CM_DEFAULT 20u

/**
 * @brief One cycle's worth of on-device evidence.
 *
 * A struct rather than six parameters because every field is read together and
 * a caller that gets the order wrong would compile silently. The fingerprint is
 * embedded by value for the reason scene_change.h gives for making it a value
 * type: the planner builds one on its stack per frame, and a stored pointer
 * would leave open the question of which frame it refers to.
 */
typedef struct {
    /** This cycle's frame. An invalid fingerprint contributes nothing — a frame
     *  nobody could decode is not evidence that the view did or did not change,
     *  and its all-zero block array is byte-identical to a flat grey frame. */
    scene_fingerprint_t frame;

    /** A latched, unexpired loudness or spectral-shape event from
     *  ambient_audio_event(). Deliberately NOT ambient_audio_novel(): that one
     *  additionally reports a room never spoken about as novel, which is right
     *  for speech and wrong here — a robot that never speaks would never see
     *  that branch clear, and would never go dormant. */
    bool audio_event;

    /** Latest smoothed rangefinder reading, and whether it means anything.
     *  reactive_telemetry_t.sensor_failed must map to range_valid = false: the
     *  filter maps a dead sensor to max range, so an unguarded reading would
     *  show a large step exactly once and then sit still forever. */
    uint16_t distance_cm;
    bool range_valid;

    /** True while the robot is under a manual lease or holding a fresh non-STOP
     *  goal. Forces the base period: a moving robot needs planning, and its own
     *  motion makes every other detector fire anyway. */
    bool robot_moving;
} plan_evidence_t;

/**
 * @brief Reset thresholds to defaults, forget the reference, boot dormant.
 *
 * @param base_period_ms  The planner's tick period (PLANNER_LOOP_PERIOD_MS).
 *                        The ladder is expressed as multipliers of it, so it is
 *                        supplied rather than #included: this module stays free
 *                        of planner_task.h, and a test can drive a whole ladder
 *                        in microseconds of simulated time.
 */
void plan_activity_init(uint32_t base_period_ms);

/**
 * @brief Decide whether to make a Gemini request this cycle.
 *
 * Call once per planner tick, before the request. Pure with respect to the
 * clock — @p now_ms is supplied — and wrap-safe across the uint32 millisecond
 * rollover at day 49.
 *
 * The first valid frame after init is adopted as the reference and reports
 * false: a first observation is not evidence, which is what makes "boot
 * dormant, wake on evidence" hold rather than firing one request at every boot.
 *
 * @param ev      This cycle's evidence. Must not be NULL.
 * @param now_ms  Milliseconds since boot.
 * @return true if the caller should make the request now.
 */
bool plan_activity_should_call(const plan_evidence_t *ev, uint32_t now_ms);

/**
 * @brief Record that a request was actually made.
 *
 * Adopts this cycle's frame and range as the reference — this is the view the
 * robot has now planned on — and, when the cycle produced no evidence, steps
 * the ladder up or (at the top) enters dormancy.
 *
 * Call only when the request really went out. A request refused by the budget
 * fuse must NOT be noted: doing so would move the reference to a view nothing
 * ever looked at, and the robot would then measure change against a frame it
 * never planned from.
 */
void plan_activity_note_call(uint32_t now_ms);

/**
 * @brief Wake from another task — a console byte, a manual drive command, a
 *        voice turn, an MQTT command.
 *
 * Sets a one-shot flag consumed by the next plan_activity_should_call(). A flag
 * rather than a direct state change because the planner task owns every other
 * field here and this is the only entry point another task uses; a torn read of
 * a bool cannot produce a state the planner cannot recover from, whereas a
 * concurrent write to the ladder index could.
 *
 * Idempotent: several wakes before the next tick are one wake.
 */
void plan_activity_wake(void);

/** @brief Enter dormancy immediately, as if the ladder had been spent. */
void plan_activity_sleep(void);

/** @brief Whether the planner is currently making no requests at all. */
bool plan_activity_dormant(void);

/** @brief Current ladder period in ms — how long until the next keep-alive
 *         request if nothing changes. 0 while dormant. */
uint32_t plan_activity_period_ms(void);

/** @brief Current rung, 0..PLAN_LADDER_STEPS-1. */
uint8_t plan_activity_step(void);

/** @brief Distance between this cycle's frame and the last-planned view. */
unsigned plan_activity_scene_score(void);

/** @brief Absolute change in the rangefinder reading since the last plan, cm. */
unsigned plan_activity_range_score(void);

/**
 * @brief Which evidence decided the most recent cycle, as a short string.
 *
 * Letters combine, newest evaluation wins: `W` woken externally, `M` moving,
 * `V` view changed, `A` audio event, `R` range changed; `+` a keep-alive
 * request fell due with no evidence; `.` holding between rungs; `Z` dormant.
 *
 * This exists so the ladder can be judged from a capture rather than argued
 * about. In a 20-minute `just robocar-unified::monitor | tee` run with someone
 * in the room, `Z` should be rare and `V` common; on an empty desk the sequence
 * should read `+ . . + . . . +` and end at `Z`. `Z` while a person is visibly
 * moving means `plan scene` is too high; a ladder that never leaves step 0 on
 * an empty desk means it is too low, or the camera is noisier than the
 * threshold.
 *
 * Returns a pointer to static storage, valid until the next
 * plan_activity_should_call(). Pass it as a %s ARGUMENT — never as the format
 * itself (see .claude/rules/esp-log-format-literal.md).
 */
const char *plan_activity_verdict(void);

/** @brief Set the wake thresholds. Either may be 0 to disable that sub-gate. */
void plan_activity_configure(uint8_t scene_threshold, uint8_t range_threshold_cm);

/** @brief Read the wake thresholds. Either pointer may be NULL. */
void plan_activity_get(uint8_t *scene_threshold, uint8_t *range_threshold_cm);

/**
 * @brief Enable or disable the whole mechanism.
 *
 * Disabled restores the original behaviour exactly — a request every tick,
 * forever — for a bench session that wants it. The budget fuse still applies;
 * this switch turns off the *optimisation*, never the backstop.
 */
void plan_activity_set_enabled(bool enabled);

/** @brief Whether the dormancy mechanism is enabled. */
bool plan_activity_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* PLAN_ACTIVITY_H */
