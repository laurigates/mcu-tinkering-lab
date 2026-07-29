/**
 * @file test_speech_budget.c
 * @brief Host tests for the speech rationing module.
 *
 * The module exists to answer one question — "may the robot speak right now?" —
 * and the expensive failure modes are all arithmetic: an off-by-one that lets a
 * fourth line through a three-line window, a window that never reopens, or a
 * comparison that breaks across the uint32 millisecond wrap and mutes the robot
 * for 49 days. Those are exactly what a host test can pin down and a bench
 * session cannot: reproducing the wrap on hardware means waiting for it.
 */

#include "speech_budget.h"

#include <assert.h>
#include <stdio.h>

/* =========================================================================
 * Test harness
 * ========================================================================= */

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* =========================================================================
 * Defaults
 * ========================================================================= */

static void test_defaults_allow_the_first_line(void)
{
    speech_budget_init();
    /* A freshly booted robot has said nothing, so nothing should hold it back —
     * the self-introduction would otherwise be swallowed by its own budget. */
    ASSERT(speech_budget_allows(0));
    ASSERT(speech_budget_wait_ms(0) == 0);
    ASSERT(speech_budget_used(0) == 0);
}

static void test_defaults_are_reported(void)
{
    speech_budget_init();
    uint32_t gap = 0;
    uint32_t window = 0;
    uint8_t max_per = 0;
    speech_budget_get(&gap, &max_per, &window);
    ASSERT(gap == SPEECH_BUDGET_MIN_GAP_MS_DEFAULT);
    ASSERT(window == SPEECH_BUDGET_WINDOW_MS_DEFAULT);
    ASSERT(max_per == SPEECH_BUDGET_MAX_PER_WINDOW_DEFAULT);

    /* Every pointer is optional — the console reads one field at a time. */
    speech_budget_get(NULL, NULL, NULL);
}

/* =========================================================================
 * Minimum gap
 * ========================================================================= */

static void test_min_gap_blocks_then_lifts(void)
{
    speech_budget_init();
    speech_budget_configure(10000u, 100u, 0u); /* gap only; window disabled */

    speech_budget_note(1000u);
    ASSERT(!speech_budget_allows(1000u));
    ASSERT(!speech_budget_allows(10999u));
    ASSERT(speech_budget_wait_ms(6000u) == 5000u);
    ASSERT(speech_budget_allows(11000u)); /* exactly at the gap */
}

static void test_zero_gap_disables_the_check(void)
{
    speech_budget_init();
    speech_budget_configure(0u, 100u, 0u);
    speech_budget_note(1000u);
    ASSERT(speech_budget_allows(1000u));
}

/* =========================================================================
 * Rolling window
 * ========================================================================= */

static void test_window_caps_the_count(void)
{
    speech_budget_init();
    speech_budget_configure(0u, 3u, 100000u); /* window only */

    speech_budget_note(0u);
    speech_budget_note(10000u);
    ASSERT(speech_budget_used(20000u) == 2);
    ASSERT(speech_budget_allows(20000u));

    speech_budget_note(20000u);
    ASSERT(speech_budget_used(20000u) == 3);
    ASSERT(!speech_budget_allows(20000u)); /* cap reached */

    /* Reopens when the oldest of the three ages out of the window, not when the
     * newest does — the off-by-one this test exists for. */
    ASSERT(!speech_budget_allows(99999u));
    ASSERT(speech_budget_allows(100000u));
    ASSERT(speech_budget_wait_ms(50000u) == 50000u);
}

static void test_window_only_counts_inside_it(void)
{
    speech_budget_init();
    speech_budget_configure(0u, 3u, 10000u);
    speech_budget_note(0u);
    speech_budget_note(1000u);
    ASSERT(speech_budget_used(5000u) == 2);
    ASSERT(speech_budget_used(10500u) == 1); /* the first has aged out */
    ASSERT(speech_budget_used(50000u) == 0);
    /* The window is half-open: an entry exactly window_ms old is already out. */
    ASSERT(speech_budget_used(9999u) == 2);
    ASSERT(speech_budget_used(10000u) == 1);
}

static void test_zero_window_disables_the_check(void)
{
    speech_budget_init();
    speech_budget_configure(0u, 1u, 0u);
    speech_budget_note(0u);
    speech_budget_note(1u);
    ASSERT(speech_budget_allows(2u));
    ASSERT(speech_budget_used(2u) == 0);
}

/* =========================================================================
 * Mute and clamping
 * ========================================================================= */

static void test_zero_cap_mutes(void)
{
    speech_budget_init();
    speech_budget_configure(0u, 0u, 60000u);
    ASSERT(!speech_budget_allows(0u));
    ASSERT(!speech_budget_allows(1000000u));
    /* UINT32_MAX rather than a huge number of milliseconds: muted is a state,
     * not a wait, and the console prints the two differently. */
    ASSERT(speech_budget_wait_ms(0u) == UINT32_MAX);
}

static void test_cap_is_clamped_to_the_history(void)
{
    /* A cap the ring cannot count would silently never bind, which reads as
     * "the limit is off" when it is really "the limit is unmeasurable". */
    speech_budget_init();
    speech_budget_configure(0u, 250u, 60000u);
    uint8_t max_per = 0;
    speech_budget_get(NULL, &max_per, NULL);
    ASSERT(max_per == SPEECH_BUDGET_HISTORY);
}

/* =========================================================================
 * Clock wrap
 * ========================================================================= */

static void test_survives_the_uint32_wrap(void)
{
    /* esp_timer_get_time()/1000 wraps every ~49.7 days. A signed or naive
     * comparison would mute the robot from the wrap onward — a fault nobody
     * would reproduce on a bench, and which reads as "the voice just stopped". */
    speech_budget_init();
    speech_budget_configure(10000u, 3u, 100000u);

    const uint32_t before_wrap = UINT32_MAX - 5000u;
    speech_budget_note(before_wrap);

    /* Deliberately expressed as an offset from the stored stamp so the wrap is
     * the compiler's arithmetic, not a hand-counted literal: `before_wrap +
     * 10000` is 4999 in wrapped time, and both sides must agree on that. */
    ASSERT(!speech_budget_allows(before_wrap + 9999u));
    ASSERT(speech_budget_used(before_wrap + 9999u) == 1);
    ASSERT(speech_budget_allows(before_wrap + 10000u));
    ASSERT((uint32_t)(before_wrap + 10000u) == 4999u); /* it really did wrap */
}

/* =========================================================================
 * Combined
 * ========================================================================= */

static void test_either_limit_can_veto(void)
{
    speech_budget_init(); /* 60 s gap, 3 per 300 s */

    ASSERT(speech_budget_allows(0u));
    speech_budget_note(0u);
    ASSERT(!speech_budget_allows(30000u)); /* gap */
    ASSERT(speech_budget_allows(60000u));
    speech_budget_note(60000u);
    speech_budget_note(120000u);
    ASSERT(!speech_budget_allows(180000u)); /* window, though the gap is clear */
    ASSERT(speech_budget_allows(300000u));  /* the first has aged out */
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== speech_budget host tests ===\n\n");

    test_run("defaults allow the first line", test_defaults_allow_the_first_line);
    test_run("defaults are reported", test_defaults_are_reported);

    test_run("minimum gap blocks then lifts", test_min_gap_blocks_then_lifts);
    test_run("zero gap disables the check", test_zero_gap_disables_the_check);

    test_run("window caps the count", test_window_caps_the_count);
    test_run("window only counts inside it", test_window_only_counts_inside_it);
    test_run("zero window disables the check", test_zero_window_disables_the_check);

    test_run("zero cap mutes", test_zero_cap_mutes);
    test_run("cap is clamped to the history", test_cap_is_clamped_to_the_history);

    test_run("survives the uint32 wrap", test_survives_the_uint32_wrap);
    test_run("either limit can veto", test_either_limit_can_veto);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
