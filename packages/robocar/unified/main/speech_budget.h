/**
 * @file speech_budget.h
 * @brief How often the robot is allowed to say anything at all.
 *
 * The problem
 * -----------
 * The planner prompt asks the model to speak "only when the scene has changed in
 * a way worth remarking on" and reminds it that it is consulted every
 * PLANNER_LOOP_PERIOD_MS.  Neither instruction is *evaluable*: every planner
 * request is a fresh, stateless call with one image and no record of the
 * previous frame or of having spoken.  So the model behaves like a first-time
 * observer every time — which, pointed at an unchanging bench scene, means it
 * remarks on the same unchanging thing every cycle.
 *
 * The approach
 * ------------
 * Ration speech on-device and enforce it by *withholding the tool*: on a quiet
 * cycle gemini_backend.c omits the `speak` function declaration from the request
 * altogether, so the model cannot call it.  An instruction is advisory; a
 * missing tool is not.  Two independent limits, either of which can veto:
 *
 *   - a **minimum gap** between utterances (the "stop chattering" knob), and
 *   - a **cap per rolling window** (the "do not monologue for five minutes
 *     straight" knob), which a minimum gap alone cannot express.
 *
 * Both are runtime-tunable from the console rather than compile-time, for the
 * same reason the camera gain ceiling is: whether a robot is pleasantly laconic
 * or annoyingly mute is a judgement only somebody in the room can make, and a
 * reflash per trial is far too slow a loop.  They are deliberately *not*
 * persisted — a boot should come up at the documented default rather than at
 * whatever last night's experiment left behind.
 *
 * Time and concurrency
 * --------------------
 * The caller passes the clock in (`now_ms`, from esp_timer_get_time()/1000) so
 * this module stays free of ESP-IDF and FreeRTOS and can be unit-tested on the
 * host — same discipline as dialogue_style.c's externally seeded PRNG.  All
 * comparisons are unsigned differences, which are correct across the ~49.7-day
 * uint32 millisecond wrap as long as no single interval exceeds ~24.8 days.
 *
 * State is unlocked.  The planner is the only writer; the console reads it to
 * print status.  A torn read can only misreport a count by one for one line.
 */

#ifndef SPEECH_BUDGET_H
#define SPEECH_BUDGET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Minimum gap between two utterances, ms. Four planner periods at the 15 s
 *  default — enough that the robot reads as commenting rather than narrating. */
#define SPEECH_BUDGET_MIN_GAP_MS_DEFAULT 60000u

/** Rolling window for the cap, ms. */
#define SPEECH_BUDGET_WINDOW_MS_DEFAULT 300000u

/** Utterances allowed per window. */
#define SPEECH_BUDGET_MAX_PER_WINDOW_DEFAULT 3u

/** Timestamps retained. Only the newest SPEECH_BUDGET_MAX_PER_WINDOW_DEFAULT-ish
 *  matter, but the cap is runtime-tunable, so keep headroom for a raised one. */
#define SPEECH_BUDGET_HISTORY 12

/**
 * @brief Reset to the compiled-in defaults and forget all history.
 *
 * Call once at boot, before the planner starts.
 */
void speech_budget_init(void);

/**
 * @brief Set both limits.
 *
 * @param min_gap_ms     0 disables the gap check.
 * @param max_per_window 0 mutes the robot entirely (no cycle ever offers
 *                       `speak`); values above SPEECH_BUDGET_HISTORY are
 *                       clamped, since the window count cannot see further back.
 * @param window_ms      0 disables the window check.
 */
void speech_budget_configure(uint32_t min_gap_ms, uint8_t max_per_window, uint32_t window_ms);

/** @brief Read back the configured limits. Any pointer may be NULL. */
void speech_budget_get(uint32_t *min_gap_ms, uint8_t *max_per_window, uint32_t *window_ms);

/**
 * @brief Whether an utterance is permitted right now.
 *
 * @param now_ms Monotonic milliseconds (esp_timer_get_time() / 1000).
 */
bool speech_budget_allows(uint32_t now_ms);

/** @brief Record that the robot spoke at @p now_ms. Call only on lines that
 *         actually reached the speech queue — a dropped line was never heard. */
void speech_budget_note(uint32_t now_ms);

/**
 * @brief Milliseconds until speech is permitted again, 0 when it already is.
 *
 * For logging and the console. Returns UINT32_MAX when muted
 * (max_per_window == 0), i.e. "not on any schedule".
 */
uint32_t speech_budget_wait_ms(uint32_t now_ms);

/** @brief Utterances inside the current window, for the console status line. */
uint8_t speech_budget_used(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SPEECH_BUDGET_H */
