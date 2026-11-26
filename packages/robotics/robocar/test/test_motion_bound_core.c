/**
 * @file test_motion_bound_core.c
 * @brief Pins the drive/rotate time bound in motion_bound_core.c (#1057).
 *
 *  - A rotate stops after its derived duration, never later than the cap.
 *  - A drive's duration follows distance and speed, capped.
 *  - Zero distance, speed or angle runs for 0 ms.
 *  - A new goal_state write restarts the timer; the same write does not.
 *  - MOTION_BOUND_NONE never expires; the millisecond wrap does not stretch
 *    or cut a bound.
 */

#include "motion_bound_core.h"

#include <stdio.h>

static int s_failures;

#define CHECK(cond, ...)                                           \
    do {                                                           \
        if (!(cond)) {                                             \
            s_failures++;                                          \
            fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                          \
            fputc('\n', stderr);                                   \
        }                                                          \
    } while (0)

static void test_rotate_duration_and_cap(void)
{
    CHECK(motion_bound_rotate_ms(90) == 500u, "90 deg: %u", (unsigned)motion_bound_rotate_ms(90));
    CHECK(motion_bound_rotate_ms(-90) == 500u, "-90 deg: %u",
          (unsigned)motion_bound_rotate_ms(-90));
    CHECK(motion_bound_rotate_ms(1) == 6u, "1 deg rounds up: %u",
          (unsigned)motion_bound_rotate_ms(1));
    CHECK(motion_bound_rotate_ms(720) == MOTION_BOUND_CAP_MS, "720 deg not capped");
    CHECK(motion_bound_rotate_ms(-32768) == MOTION_BOUND_CAP_MS, "INT16_MIN not capped");
    CHECK(motion_bound_rotate_ms(0) == 0u, "0 deg runs");
}

static void test_drive_duration_and_cap(void)
{
    /* 55 cm at 100 % is one second; 11 cm at 50 % is 400 ms. */
    CHECK(motion_bound_drive_ms(55, 100) == 1000u, "55 cm @100: %u",
          (unsigned)motion_bound_drive_ms(55, 100));
    CHECK(motion_bound_drive_ms(11, 50) == 400u, "11 cm @50: %u",
          (unsigned)motion_bound_drive_ms(11, 50));
    CHECK(motion_bound_drive_ms(100, 40) == MOTION_BOUND_CAP_MS, "100 cm @40 not capped");
    CHECK(motion_bound_drive_ms(65535, 1) == MOTION_BOUND_CAP_MS, "max not capped");
    CHECK(motion_bound_drive_ms(0, 50) == 0u, "0 cm runs");
    CHECK(motion_bound_drive_ms(50, 0) == 0u, "0 %% runs");
}

static void test_expiry_per_goal(void)
{
    motion_bound_t b;
    motion_bound_reset(&b);
    const uint32_t bound = 500u;
    CHECK(!motion_bound_expired(&b, 4u, bound, 1000u), "expired on its first tick");
    CHECK(!motion_bound_expired(&b, 4u, bound, 1499u), "expired a ms early");
    CHECK(motion_bound_expired(&b, 4u, bound, 1500u), "not expired at the bound");
    CHECK(motion_bound_expired(&b, 4u, bound, 9000u), "un-expired later under the same goal");
    CHECK(!motion_bound_expired(&b, 5u, bound, 9000u), "a new goal did not restart the timer");
    CHECK(motion_bound_expired(&b, 5u, bound, 9500u), "the new goal outlived its bound");
}

static void test_none_never_expires(void)
{
    motion_bound_t b;
    motion_bound_reset(&b);
    CHECK(!motion_bound_expired(&b, 1u, MOTION_BOUND_NONE, 0u), "NONE expired at 0");
    CHECK(!motion_bound_expired(&b, 1u, MOTION_BOUND_NONE, 4000000000u), "NONE expired");
}

static void test_zero_bound_expires_at_once(void)
{
    motion_bound_t b;
    motion_bound_reset(&b);
    CHECK(motion_bound_expired(&b, 2u, 0u, 100u), "a 0 ms bound ran a tick");
}

static void test_ms_wrap(void)
{
    motion_bound_t b;
    motion_bound_reset(&b);
    const uint32_t start = UINT32_MAX - 100u;
    CHECK(!motion_bound_expired(&b, 9u, 500u, start), "expired at start");
    CHECK(!motion_bound_expired(&b, 9u, 500u, start + 499u), "wrap cut the bound");
    CHECK(motion_bound_expired(&b, 9u, 500u, start + 500u), "wrap stretched the bound");
}

int main(void)
{
    printf("=== motion_bound_core host tests ===\n");
    test_rotate_duration_and_cap();
    test_drive_duration_and_cap();
    test_expiry_per_goal();
    test_none_never_expires();
    test_zero_bound_expires_at_once();
    test_ms_wrap();
    if (s_failures) {
        printf("FAILED: %d check(s)\n", s_failures);
        return 1;
    }
    printf("All motion_bound_core tests passed\n");
    return 0;
}
