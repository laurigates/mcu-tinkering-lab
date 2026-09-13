/**
 * @file test_ota_interval.c
 * @brief Host tests for the OTA poll-interval tick arithmetic (PR #559 review
 *        follow-up).
 *
 * The property under test: pdMS_TO_TICKS()'s formula, reproduced here in a
 * 64-bit intermediate, must reach the SAME tick count a healthy 32-bit
 * pdMS_TO_TICKS() would produce for values that fit — and must fail closed,
 * rather than silently wrap, for values that don't.
 *
 * OTA_CHECK_INTERVAL_MIN=360 at CONFIG_FREERTOS_HZ=1000 is the exact
 * shipped configuration whose 32-bit computation overflowed:
 * (360 * 60 * 1000) as a uint32_t millisecond count is 21,600,000, which
 * does not itself overflow uint32_t — but multiplying that by
 * configTICK_RATE_HZ (1000) BEFORE dividing by 1000, all in TickType_t
 * (uint32_t), is what wraps inside pdMS_TO_TICKS() on the non-SMP kernel.
 * The 21,600,000-tick expectation below is what the firmware needs; the
 * overflow case is a value chosen to blow up that intermediate multiply.
 */

#include "ota_interval.h"

#include <assert.h>
#include <stdio.h>

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

/* The shipped configuration: 360 minutes at 1000 Hz must equal exactly
 * 21,600,000 ticks — the value the 32-bit pdMS_TO_TICKS() computation was
 * silently truncating to ~125,163. */
static void test_shipped_interval_is_exact(void)
{
    uint32_t ticks = 0;
    ASSERT(ota_interval_ticks_from_minutes(360, 1000, &ticks) == true);
    ASSERT(ticks == 21600000u);
}

/* Smaller, easy-to-hand-verify cases. */
static void test_small_cases(void)
{
    uint32_t ticks = 0;
    ASSERT(ota_interval_ticks_from_minutes(1, 1000, &ticks) == true);
    ASSERT(ticks == 60000u);

    ASSERT(ota_interval_ticks_from_minutes(0, 1000, &ticks) == true);
    ASSERT(ticks == 0u);

    ASSERT(ota_interval_ticks_from_minutes(1, 100, &ticks) == true);
    ASSERT(ticks == 6000u);
}

/* A value that overflows the intermediate must fail closed — false, and the
 * caller's contract is to not read *out_ticks afterward. */
static void test_overflow_fails_closed(void)
{
    uint32_t ticks = 0xDEADBEEFu; /* sentinel: must be left untouched */
    /* minutes * 60 * tick_rate_hz must exceed UINT32_MAX (4,294,967,295).
     * 100,000 min * 60 * 1000 Hz = 6,000,000,000,000 — comfortably over. */
    ASSERT(ota_interval_ticks_from_minutes(100000u, 1000u, &ticks) == false);
    ASSERT(ticks == 0xDEADBEEFu);

    /* The exact boundary the original bug hit: minutes large enough that
     * ms * tick_rate_hz overflows uint32_t, at the project's real tick
     * rate. UINT32_MAX / 1000 / 60 ~= 71,582 min is the rough breakeven;
     * comfortably past it: */
    ASSERT(ota_interval_ticks_from_minutes(80000u, 1000u, &ticks) == false);
    ASSERT(ticks == 0xDEADBEEFu);
}

/* A NULL output pointer must fail closed rather than crash. */
static void test_null_out_ticks_fails_closed(void)
{
    ASSERT(ota_interval_ticks_from_minutes(360, 1000, NULL) == false);
}

int main(void)
{
    printf("=== OTA interval tick-arithmetic host tests ===\n\n");

    test_run("shipped_interval_is_exact", test_shipped_interval_is_exact);
    test_run("small_cases", test_small_cases);
    test_run("overflow_fails_closed", test_overflow_fails_closed);
    test_run("null_out_ticks_fails_closed", test_null_out_ticks_fails_closed);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
