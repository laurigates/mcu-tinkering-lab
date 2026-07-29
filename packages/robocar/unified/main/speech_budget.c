/**
 * @file speech_budget.c
 * @brief Speech rationing. See the header for why this is enforced on-device.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_speech_budget.c
 * builds it on the host with no shims.
 */

#include "speech_budget.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

static uint32_t s_min_gap_ms = SPEECH_BUDGET_MIN_GAP_MS_DEFAULT;
static uint32_t s_window_ms = SPEECH_BUDGET_WINDOW_MS_DEFAULT;
static uint8_t s_max_per_window = SPEECH_BUDGET_MAX_PER_WINDOW_DEFAULT;

/* Ring of utterance timestamps, newest at (head - 1). */
static uint32_t s_hist[SPEECH_BUDGET_HISTORY];
static uint8_t s_head;
static uint8_t s_used;

void speech_budget_init(void)
{
    s_min_gap_ms = SPEECH_BUDGET_MIN_GAP_MS_DEFAULT;
    s_window_ms = SPEECH_BUDGET_WINDOW_MS_DEFAULT;
    s_max_per_window = SPEECH_BUDGET_MAX_PER_WINDOW_DEFAULT;
    memset(s_hist, 0, sizeof(s_hist));
    s_head = 0;
    s_used = 0;
}

void speech_budget_configure(uint32_t min_gap_ms, uint8_t max_per_window, uint32_t window_ms)
{
    s_min_gap_ms = min_gap_ms;
    s_window_ms = window_ms;
    /* Clamped rather than rejected: a cap larger than the history ring would
     * silently never bind, which reads as "the limit is off" when it is really
     * "the limit cannot be counted". */
    s_max_per_window =
        (max_per_window > SPEECH_BUDGET_HISTORY) ? (uint8_t)SPEECH_BUDGET_HISTORY : max_per_window;
}

void speech_budget_get(uint32_t *min_gap_ms, uint8_t *max_per_window, uint32_t *window_ms)
{
    if (min_gap_ms) {
        *min_gap_ms = s_min_gap_ms;
    }
    if (max_per_window) {
        *max_per_window = s_max_per_window;
    }
    if (window_ms) {
        *window_ms = s_window_ms;
    }
}

/** Newest timestamp, or false when nothing has been spoken yet. */
static bool newest(uint32_t *out)
{
    if (s_used == 0u) {
        return false;
    }
    const uint8_t idx = (uint8_t)((s_head + SPEECH_BUDGET_HISTORY - 1u) % SPEECH_BUDGET_HISTORY);
    *out = s_hist[idx];
    return true;
}

uint8_t speech_budget_used(uint32_t now_ms)
{
    if (s_window_ms == 0u) {
        return 0;
    }
    uint8_t count = 0;
    for (uint8_t n = 0; n < s_used; ++n) {
        const uint8_t idx =
            (uint8_t)((s_head + SPEECH_BUDGET_HISTORY - 1u - n) % SPEECH_BUDGET_HISTORY);
        /* Unsigned difference: correct across the uint32 ms wrap. Entries walk
         * newest-first, so the first one outside the window ends the scan. */
        if ((uint32_t)(now_ms - s_hist[idx]) >= s_window_ms) {
            break;
        }
        ++count;
    }
    return count;
}

bool speech_budget_allows(uint32_t now_ms)
{
    if (s_max_per_window == 0u) {
        return false; /* muted */
    }

    uint32_t last = 0;
    if (s_min_gap_ms > 0u && newest(&last) && (uint32_t)(now_ms - last) < s_min_gap_ms) {
        return false;
    }

    if (s_window_ms > 0u && speech_budget_used(now_ms) >= s_max_per_window) {
        return false;
    }

    return true;
}

uint32_t speech_budget_wait_ms(uint32_t now_ms)
{
    if (s_max_per_window == 0u) {
        return UINT32_MAX;
    }
    if (speech_budget_allows(now_ms)) {
        return 0;
    }

    uint32_t wait = 0;

    uint32_t last = 0;
    if (s_min_gap_ms > 0u && newest(&last)) {
        const uint32_t since = (uint32_t)(now_ms - last);
        if (since < s_min_gap_ms) {
            wait = s_min_gap_ms - since;
        }
    }

    /* Window-limited: the block lifts when the oldest in-window entry ages out.
     * That is the (max_per_window)-th newest — one slot further back than the
     * cap, counting from zero. */
    if (s_window_ms > 0u && speech_budget_used(now_ms) >= s_max_per_window) {
        const uint8_t nth = (uint8_t)(s_max_per_window - 1u);
        if (nth < s_used) {
            const uint8_t idx =
                (uint8_t)((s_head + SPEECH_BUDGET_HISTORY - 1u - nth) % SPEECH_BUDGET_HISTORY);
            const uint32_t age = (uint32_t)(now_ms - s_hist[idx]);
            const uint32_t until = (age < s_window_ms) ? (s_window_ms - age) : 0u;
            if (until > wait) {
                wait = until;
            }
        }
    }

    return wait;
}

void speech_budget_note(uint32_t now_ms)
{
    s_hist[s_head] = now_ms;
    s_head = (uint8_t)((s_head + 1u) % SPEECH_BUDGET_HISTORY);
    if (s_used < SPEECH_BUDGET_HISTORY) {
        ++s_used;
    }
}
