/**
 * @file voice_budget.h
 * @brief What voice turns may spend: a ration on hands-free triggers and a
 *        per-boot ceiling on every voice-turn request.
 *
 * ## The problem
 *
 * Hands-free listening is on at boot (issue #617). Before this module the only
 * pacing on a VAD-triggered turn was the listener's idle cooldown
 * (VAD_IDLE_COOLDOWN_MS, 10 s). Each trigger is a `gemini-flash-latest` request
 * carrying a WAV clip and a camera JPEG, so a room where people talk among
 * themselves could issue one every 10 s — around 360 an hour, unattended, most
 * of them answered `__IGNORE__`, and nothing counting them. The planner has had a
 * per-boot fuse since ADR-022 (plan_budget.h); voice turns had none.
 *
 * ## Why a separate ration, not speech_budget
 *
 * speech_budget.h rations *utterances*: it is charged only when a line reaches
 * the speech queue. That makes it the wrong instrument here, three ways over:
 *
 *   - **It cannot see the cost.** An `__IGNORE__` reply is never spoken, so it
 *     never charges speech_budget — and ignored replies are precisely the traffic
 *     that runs unbounded. A VAD gate on speech_budget_allows() would still let
 *     360 ignored requests an hour through.
 *   - **It would break the conversation.** Its 20 s minimum gap is longer than
 *     the 7 s follow-up window voice_history.h opens after a reply, so every
 *     follow-up would be refused exactly when somebody is talking to the robot.
 *   - **Sharing lets either side starve the other.** A chatty planner would use
 *     the turns a person needs to be answered, and a conversation would silence
 *     the planner's remarks.
 *
 * So this counts *requests*, which is what is billed, and keeps its own window.
 * A voice turn that does speak still charges speech_budget afterwards
 * (voice_turn.c), so the planner does not remark three seconds after an answer;
 * that ruling is unchanged.
 *
 * ## Two limits
 *
 *   - A **ration**: at most N hands-free (VAD) requests per rolling window. It
 *     bounds the *rate*. `listen` is exempt — a person typed it, and refusing an
 *     explicit request on a timer would read as broken.
 *   - A **ceiling**: at most M voice-turn requests per boot, `listen` included.
 *     It bounds the *total*, the same backstop plan_budget.h is for the planner:
 *     the dumbest thing in the chain, a counter that a detector failing open
 *     cannot argue with.
 *
 * Tripping the ceiling is terminal until `voice resume`, an MQTT command or a
 * reboot. There is no auto-reset, for plan_budget.h's reason: a fuse that resets
 * itself resumes spending on an unattended board the moment it rolls over.
 *
 * A request is charged when it is *attempted* (immediately before the HTTP
 * post), success or failure, so a ceiling bounds traffic even if every reply is
 * unparseable. Requests only — no token ceiling: a voice turn's size is bounded
 * by construction (one clip of at most 8 s, one JPEG, a fixed output cap), so
 * the request count already bounds the tokens to within a constant.
 *
 * ## The ignored count
 *
 * `__IGNORE__` replies to VAD turns are counted on their own. A trigger
 * threshold that is too permissive shows up as ignored ≈ VAD requests: the robot
 * is waking for conversations not addressed to it. That is a number to tune
 * `voice trigger` against, rather than a feeling.
 *
 * ## Time and concurrency
 *
 * Pure C — no FreeRTOS, no ESP-IDF — with the clock passed in, so
 * test/test_voice_budget.c builds it on the host with no shims. All window
 * comparisons are unsigned differences, correct across the uint32 millisecond
 * wrap at day 49.
 *
 * State is unlocked, like speech_budget.h. The voice-turn task charges, the
 * ambient listener asks, the console reads and configures. A torn read can at
 * worst let one extra request through the ration or misreport a count by one
 * for one line; the ceiling is checked again on the voice-turn task before each
 * request, so it cannot be overrun by more than the single turn already queued.
 */

#ifndef VOICE_BUDGET_H
#define VOICE_BUDGET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Hands-free requests allowed per ration window.
 *
 *  12 per 5 minutes caps unattended VAD traffic at 144 an hour, against 360 with
 *  the cooldown alone, while leaving room for a real exchange of a dozen
 *  questions. A starting point, not a measurement: tune it with `voice ration`
 *  against the `ignored` count. */
#define VOICE_BUDGET_RATION_MAX_DEFAULT 12u

/** The ration's rolling window, ms. */
#define VOICE_BUDGET_RATION_WINDOW_MS_DEFAULT 300000u

/** Timestamps retained for the window count; a raised ration is clamped here. */
#define VOICE_BUDGET_HISTORY 32u

/** Voice-turn requests allowed per boot, `listen` included.
 *
 *  200 is about 80 minutes of a saturated ration — enough for a long bench
 *  session, short of an unattended night. Raise it from the console
 *  (`voice turns <n>`) rather than quietly sizing the default for the longest
 *  thing anyone might do. */
#define VOICE_BUDGET_MAX_REQUESTS_DEFAULT 200u

/** Why a voice turn would be refused, or VOICE_BUDGET_OK. */
typedef enum {
    VOICE_BUDGET_OK = 0,      /**< Allowed.                                     */
    VOICE_BUDGET_RATIONED,    /**< VAD only: the window's ration is used up.    */
    VOICE_BUDGET_TRIP_CEILING /**< The per-boot ceiling has been reached.       */
} voice_budget_verdict_t;

/** @brief Reset limits to their defaults, zero every counter, clear the trip. */
void voice_budget_init(void);

/**
 * @brief Set the hands-free ration.
 *
 * @param max_per_window  VAD requests per window. 0 removes the ration (VAD turns
 *                        are then paced by the listener's cooldown alone; use
 *                        `voice vad off` to stop them). Clamped to
 *                        VOICE_BUDGET_HISTORY, since the count cannot see
 *                        further back.
 * @param window_ms       0 also removes the ration.
 */
void voice_budget_configure_ration(uint8_t max_per_window, uint32_t window_ms);

/** @brief Read the ration. Either pointer may be NULL. */
void voice_budget_get_ration(uint8_t *max_per_window, uint32_t *window_ms);

/** @brief Set the per-boot request ceiling. 0 disables it. */
void voice_budget_configure_ceiling(uint32_t max_requests);

/** @brief The per-boot request ceiling (0 = disabled). */
uint32_t voice_budget_ceiling(void);

/**
 * @brief Whether a voice turn may be made now.
 *
 * The ceiling is checked first and applies to both kinds; the ration applies to
 * hands-free turns only.
 *
 * @param now_ms  Monotonic milliseconds.
 * @param vad     true for a hands-free (VAD) turn, false for `listen`.
 */
voice_budget_verdict_t voice_budget_check(uint32_t now_ms, bool vad);

/**
 * @brief Charge one attempted voice-turn request.
 *
 * Call once per request actually sent, immediately before the HTTP post, from
 * the single place voice_turn.c makes it.
 */
void voice_budget_note_request(uint32_t now_ms, bool vad);

/** @brief Count an `__IGNORE__` reply. */
void voice_budget_note_ignored(bool vad);

/** @brief Count a hands-free trigger the ration refused. */
void voice_budget_note_refused(void);

/** @brief Clear the trip and zero the counters. The operator's `voice resume`.
 *         The ration window is cleared too, so a resumed board starts fresh. */
void voice_budget_resume(void);

/** @brief Hands-free requests inside the current ration window. */
uint8_t voice_budget_ration_used(uint32_t now_ms);

/**
 * @brief Milliseconds until the ration admits another hands-free request, 0
 *        when it already does (or there is no ration).
 */
uint32_t voice_budget_ration_wait_ms(uint32_t now_ms);

/** @brief Requests charged since init or the last resume, both kinds. */
uint32_t voice_budget_requests(void);

/** @brief Of those, how many were hands-free. */
uint32_t voice_budget_vad_requests(void);

/** @brief `__IGNORE__` replies to hands-free turns. */
uint32_t voice_budget_ignored_vad(void);

/** @brief `__IGNORE__` replies to `listen` turns. */
uint32_t voice_budget_ignored_listen(void);

/** @brief Hands-free triggers refused by the ration. */
uint32_t voice_budget_refused(void);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_BUDGET_H */
