/**
 * @file voice_budget.c
 * @brief Voice-turn ration and spend ceiling. See the header for why this is
 *        separate from speech_budget and counts requests, not utterances.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_voice_budget.c
 * builds it on the host with no shims.
 */

#include "voice_budget.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

static uint8_t s_ration_max = VOICE_BUDGET_RATION_MAX_DEFAULT;
static uint32_t s_ration_window_ms = VOICE_BUDGET_RATION_WINDOW_MS_DEFAULT;
static uint32_t s_max_requests = VOICE_BUDGET_MAX_REQUESTS_DEFAULT;

/* Ring of hands-free request timestamps, newest at (head - 1). */
static uint32_t s_hist[VOICE_BUDGET_HISTORY];
static uint8_t s_head;
static uint8_t s_used;

static uint32_t s_requests;
static uint32_t s_vad_requests;
static uint32_t s_ignored_vad;
static uint32_t s_ignored_listen;
static uint32_t s_refused;

/** Saturating increment. A wrapped request counter reads as a fresh boot and
 *  would silently un-trip the ceiling — plan_budget.c's reasoning, same fuse. */
static void bump(uint32_t *counter)
{
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void clear_counters(void)
{
    memset(s_hist, 0, sizeof(s_hist));
    s_head = 0u;
    s_used = 0u;
    s_requests = 0u;
    s_vad_requests = 0u;
    s_ignored_vad = 0u;
    s_ignored_listen = 0u;
    s_refused = 0u;
}

void voice_budget_init(void)
{
    s_ration_max = VOICE_BUDGET_RATION_MAX_DEFAULT;
    s_ration_window_ms = VOICE_BUDGET_RATION_WINDOW_MS_DEFAULT;
    s_max_requests = VOICE_BUDGET_MAX_REQUESTS_DEFAULT;
    clear_counters();
}

void voice_budget_configure_ration(uint8_t max_per_window, uint32_t window_ms)
{
    /* Clamped rather than rejected, as in speech_budget.c: a cap larger than the
     * ring would silently never bind. */
    s_ration_max =
        (max_per_window > VOICE_BUDGET_HISTORY) ? (uint8_t)VOICE_BUDGET_HISTORY : max_per_window;
    s_ration_window_ms = window_ms;
}

void voice_budget_get_ration(uint8_t *max_per_window, uint32_t *window_ms)
{
    if (max_per_window) {
        *max_per_window = s_ration_max;
    }
    if (window_ms) {
        *window_ms = s_ration_window_ms;
    }
}

void voice_budget_configure_ceiling(uint32_t max_requests)
{
    s_max_requests = max_requests;
}

uint32_t voice_budget_ceiling(void)
{
    return s_max_requests;
}

static bool ration_active(void)
{
    return s_ration_max != 0u && s_ration_window_ms != 0u;
}

/** Index of the n-th newest timestamp (n = 0 is the newest). */
static uint8_t nth_newest(uint8_t n)
{
    return (uint8_t)((s_head + VOICE_BUDGET_HISTORY - 1u - n) % VOICE_BUDGET_HISTORY);
}

uint8_t voice_budget_ration_used(uint32_t now_ms)
{
    if (!ration_active()) {
        return 0u;
    }
    uint8_t count = 0u;
    for (uint8_t n = 0u; n < s_used; ++n) {
        /* Unsigned difference: correct across the uint32 ms wrap. Entries walk
         * newest-first, so the first one outside the window ends the scan. */
        if ((uint32_t)(now_ms - s_hist[nth_newest(n)]) >= s_ration_window_ms) {
            break;
        }
        ++count;
    }
    return count;
}

voice_budget_verdict_t voice_budget_check(uint32_t now_ms, bool vad)
{
    /* The ceiling first: when both bind, name the one that needs a human rather
     * than the one that clears by itself within the window. */
    if (s_max_requests != 0u && s_requests >= s_max_requests) {
        return VOICE_BUDGET_TRIP_CEILING;
    }
    if (vad && ration_active() && voice_budget_ration_used(now_ms) >= s_ration_max) {
        return VOICE_BUDGET_RATIONED;
    }
    return VOICE_BUDGET_OK;
}

uint32_t voice_budget_ration_wait_ms(uint32_t now_ms)
{
    if (!ration_active() || voice_budget_ration_used(now_ms) < s_ration_max) {
        return 0u;
    }
    /* The ration lifts when the oldest in-window entry ages out: the
     * (max)-th newest, counting from zero. */
    const uint8_t nth = (uint8_t)(s_ration_max - 1u);
    if (nth >= s_used) {
        return 0u;
    }
    const uint32_t age = (uint32_t)(now_ms - s_hist[nth_newest(nth)]);
    return (age < s_ration_window_ms) ? (s_ration_window_ms - age) : 0u;
}

void voice_budget_note_request(uint32_t now_ms, bool vad)
{
    bump(&s_requests);
    if (!vad) {
        return; /* `listen` is exempt from the ration, so it takes no slot */
    }
    bump(&s_vad_requests);
    s_hist[s_head] = now_ms;
    s_head = (uint8_t)((s_head + 1u) % VOICE_BUDGET_HISTORY);
    if (s_used < VOICE_BUDGET_HISTORY) {
        ++s_used;
    }
}

void voice_budget_note_ignored(bool vad)
{
    bump(vad ? &s_ignored_vad : &s_ignored_listen);
}

void voice_budget_note_refused(void)
{
    bump(&s_refused);
}

void voice_budget_resume(void)
{
    clear_counters();
}

uint32_t voice_budget_requests(void)
{
    return s_requests;
}

uint32_t voice_budget_vad_requests(void)
{
    return s_vad_requests;
}

uint32_t voice_budget_ignored_vad(void)
{
    return s_ignored_vad;
}

uint32_t voice_budget_ignored_listen(void)
{
    return s_ignored_listen;
}

uint32_t voice_budget_refused(void)
{
    return s_refused;
}
