/**
 * @file plan_activity.c
 * @brief Planner dormancy and backoff ladder. See the header for the design.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_plan_activity.c
 * builds it on the host with no shims.
 */

#include "plan_activity.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* Ladder                                                                      */
/* -------------------------------------------------------------------------- */

/** Multipliers of the planner's base period: 15 s, 30 s, 1 min, 2 min, 5 min.
 *  Spending the whole ladder from a standing start takes 1+2+4+8+20 = 35 base
 *  periods, about nine minutes, and costs five requests. */
static const uint8_t k_ladder[PLAN_LADDER_STEPS] = {1u, 2u, 4u, 8u, 20u};

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

static uint32_t s_base_period_ms = 15000u;

static uint8_t s_scene_threshold = PLAN_SCENE_THRESHOLD_DEFAULT;
static uint8_t s_range_threshold_cm = PLAN_RANGE_THRESHOLD_CM_DEFAULT;
static bool s_enabled = true;

/** The view and range the robot last planned on. */
static scene_fingerprint_t s_reference;
static uint16_t s_reference_cm;
static bool s_reference_range_valid;

/** This cycle's frame and range, held so note_call() can adopt them. */
static scene_fingerprint_t s_current;
static uint16_t s_current_cm;
static bool s_current_range_valid;

static uint8_t s_step;
static bool s_dormant;
static uint32_t s_last_call_ms;
static bool s_have_called;

/** Set by another task, consumed by the next evaluation. */
static volatile bool s_wake_pending;

/** Whether the most recent evaluation saw any evidence — read by note_call() to
 *  decide whether the ladder advances. Kept here rather than recomputed because
 *  note_call() has no evidence to recompute from. */
static bool s_last_evidence;

static unsigned s_scene_score;
static unsigned s_range_score;

/** Verdict letters for the most recent evaluation. Sized for every letter plus
 *  the terminator; see plan_activity_verdict(). */
static char s_verdict[8] = ".";

void plan_activity_init(uint32_t base_period_ms)
{
    /* A zero base period would make every keep-alive fall due immediately,
     * turning the ladder into no ladder at all — the exact failure this module
     * exists to prevent, arrived at through a caller's arithmetic rather than a
     * decision. Refuse it. */
    s_base_period_ms = (base_period_ms == 0u) ? 15000u : base_period_ms;

    s_scene_threshold = PLAN_SCENE_THRESHOLD_DEFAULT;
    s_range_threshold_cm = PLAN_RANGE_THRESHOLD_CM_DEFAULT;
    s_enabled = true;

    memset(&s_reference, 0, sizeof(s_reference));
    memset(&s_current, 0, sizeof(s_current));
    s_reference_cm = 0u;
    s_reference_range_valid = false;
    s_current_cm = 0u;
    s_current_range_valid = false;

    /* Boot dormant, at the top of the ladder. A board left powered on a still
     * desk then makes no request at all until something happens in front of it
     * — which is the whole point, and is why the first valid frame is adopted
     * as the reference rather than treated as a change. */
    s_step = (uint8_t)(PLAN_LADDER_STEPS - 1);
    s_dormant = true;
    s_last_call_ms = 0u;
    s_have_called = false;
    s_wake_pending = false;
    s_last_evidence = false;

    s_scene_score = 0u;
    s_range_score = 0u;
    s_verdict[0] = 'Z';
    s_verdict[1] = '\0';
}

/* -------------------------------------------------------------------------- */
/* Evaluation                                                                  */
/* -------------------------------------------------------------------------- */

static void verdict_set(bool woken, bool moving, bool view, bool audio, bool range)
{
    size_t n = 0u;
    if (woken) {
        s_verdict[n++] = 'W';
    }
    if (moving) {
        s_verdict[n++] = 'M';
    }
    if (view) {
        s_verdict[n++] = 'V';
    }
    if (audio) {
        s_verdict[n++] = 'A';
    }
    if (range) {
        s_verdict[n++] = 'R';
    }
    if (n == 0u) {
        s_verdict[n++] = '.';
    }
    s_verdict[n] = '\0';
}

bool plan_activity_should_call(const plan_evidence_t *ev, uint32_t now_ms)
{
    if (!ev) {
        return false;
    }

    /* ---- Scores, always, so the log can be read even while disabled ---- */
    s_current = ev->frame;
    s_current_cm = ev->distance_cm;
    s_current_range_valid = ev->range_valid;

    s_scene_score = (ev->frame.valid && s_reference.valid)
                        ? scene_fingerprint_distance(&ev->frame, &s_reference)
                        : 0u;

    s_range_score =
        (ev->range_valid && s_reference_range_valid)
            ? (unsigned)((ev->distance_cm > s_reference_cm) ? (ev->distance_cm - s_reference_cm)
                                                            : (s_reference_cm - ev->distance_cm))
            : 0u;

    /* ---- Adopt a first reference ----
     * A first observation is not evidence of change: there is nothing it could
     * have changed from. Adopting here rather than in note_call() is what lets
     * the module boot dormant and still have something to compare against. */
    if (!s_reference.valid && ev->frame.valid) {
        s_reference = ev->frame;
    }
    if (!s_reference_range_valid && ev->range_valid) {
        s_reference_cm = ev->distance_cm;
        s_reference_range_valid = true;
    }

    /* The wake flag is consumed whether or not the mechanism is enabled, so a
     * console wake cannot sit latched across a `plan off` / `plan on` pair and
     * fire much later against a scene nobody was looking at. */
    const bool woken = s_wake_pending;
    s_wake_pending = false;

    if (!s_enabled) {
        s_last_evidence = true;
        s_dormant = false;
        s_step = 0u;
        verdict_set(false, false, false, false, false);
        s_verdict[0] = '+';
        return true;
    }

    const bool view = (s_scene_threshold != 0u) && (s_scene_score >= (unsigned)s_scene_threshold);
    const bool range =
        (s_range_threshold_cm != 0u) && (s_range_score >= (unsigned)s_range_threshold_cm);
    const bool audio = ev->audio_event;
    const bool moving = ev->robot_moving;

    const bool evidence = woken || moving || view || audio || range;
    s_last_evidence = evidence;
    verdict_set(woken, moving, view, audio, range);

    if (evidence) {
        s_dormant = false;
        s_step = 0u;
        return true;
    }

    if (s_dormant) {
        s_verdict[0] = 'Z';
        s_verdict[1] = '\0';
        return false;
    }

    /* Awake but with nothing to report: a keep-alive request falls due once the
     * current rung's interval has elapsed. Unsigned subtraction, so the uint32
     * millisecond rollover at day 49 is a non-event rather than a planner that
     * goes quiet for 49 days. */
    if (!s_have_called) {
        s_verdict[0] = '+';
        s_verdict[1] = '\0';
        return true;
    }

    const uint32_t due_ms = plan_activity_period_ms();
    if ((uint32_t)(now_ms - s_last_call_ms) >= due_ms) {
        s_verdict[0] = '+';
        s_verdict[1] = '\0';
        return true;
    }

    return false;
}

void plan_activity_note_call(uint32_t now_ms)
{
    s_last_call_ms = now_ms;
    s_have_called = true;

    /* This is the view the robot has now planned on, so it becomes what the
     * next frames are measured against. An unmeasurable frame is not adopted —
     * an all-zero fingerprint is indistinguishable from a flat grey frame, so
     * adopting one would make the next real frame look like a whole new room. */
    if (s_current.valid) {
        s_reference = s_current;
    }
    if (s_current_range_valid) {
        s_reference_cm = s_current_cm;
        s_reference_range_valid = true;
    }

    if (!s_enabled || s_last_evidence) {
        return;
    }

    if (s_step + 1u < PLAN_LADDER_STEPS) {
        s_step++;
    } else {
        s_dormant = true;
    }
}

void plan_activity_wake(void)
{
    s_wake_pending = true;
}

void plan_activity_sleep(void)
{
    s_dormant = true;
    s_step = (uint8_t)(PLAN_LADDER_STEPS - 1);
}

/* -------------------------------------------------------------------------- */
/* Accessors                                                                   */
/* -------------------------------------------------------------------------- */

bool plan_activity_dormant(void)
{
    return s_enabled && s_dormant;
}

uint32_t plan_activity_period_ms(void)
{
    if (s_enabled && s_dormant) {
        return 0u;
    }
    const uint8_t mult = k_ladder[(s_step < PLAN_LADDER_STEPS) ? s_step : 0u];
    return s_base_period_ms * (uint32_t)mult;
}

uint8_t plan_activity_step(void)
{
    return s_step;
}

unsigned plan_activity_scene_score(void)
{
    return s_scene_score;
}

unsigned plan_activity_range_score(void)
{
    return s_range_score;
}

const char *plan_activity_verdict(void)
{
    return s_verdict;
}

void plan_activity_configure(uint8_t scene_threshold, uint8_t range_threshold_cm)
{
    s_scene_threshold = scene_threshold;
    s_range_threshold_cm = range_threshold_cm;
}

void plan_activity_get(uint8_t *scene_threshold, uint8_t *range_threshold_cm)
{
    if (scene_threshold) {
        *scene_threshold = s_scene_threshold;
    }
    if (range_threshold_cm) {
        *range_threshold_cm = s_range_threshold_cm;
    }
}

void plan_activity_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (enabled) {
        return;
    }
    /* Leaving the mechanism disabled must not leave a latched dormancy behind
     * for a later `plan on` to resurrect: `plan off` means "request every
     * tick", and the state it hands back on re-enable should be a fresh ladder,
     * not whatever the last experiment ended in. */
    s_dormant = false;
    s_step = 0u;
}

bool plan_activity_enabled(void)
{
    return s_enabled;
}
