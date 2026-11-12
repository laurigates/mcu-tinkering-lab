/**
 * @file test_plan_budget.c
 * @brief Host tests for the planner spend ceiling.
 *
 * The load-bearing cases here are the ones a bench cannot show you, because
 * they only appear after hours of running or after a change to somebody else's
 * JSON: the fuse charging a request whose token count could not be read (a
 * silent zero there is how a spend ceiling stops being a ceiling), a network
 * outage NOT tripping it, and the counters saturating rather than wrapping back
 * into an apparently-fresh state.
 */

#include <stdio.h>
#include <string.h>

#include "plan_budget.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                            \
        }                                                            \
    } while (0)

#define TEST(name)               \
    static void name(void);      \
    static void run_##name(void) \
    {                            \
        printf("- %s\n", #name); \
        plan_budget_init();      \
        name();                  \
    }                            \
    static void name(void)

/* -------------------------------------------------------------------------- */

TEST(test_a_fresh_budget_allows_requests)
{
    ASSERT(plan_budget_allows());
    ASSERT(plan_budget_state() == PLAN_BUDGET_OK);
    ASSERT(plan_budget_requests() == 0u);
    ASSERT(plan_budget_tokens() == 0u);
    ASSERT(plan_budget_assumed() == 0u);
}

TEST(test_request_ceiling_trips_and_stays_tripped)
{
    plan_budget_configure(3u, 0u /* tokens unlimited */);

    for (int i = 0; i < 3; i++) {
        ASSERT(plan_budget_allows());
        plan_budget_note(100, true);
    }

    ASSERT(!plan_budget_allows());
    ASSERT(plan_budget_state() == PLAN_BUDGET_TRIP_REQUESTS);

    /* No rolling window, no self-healing: the fuse this module exists to be
     * would be worthless if an unattended board could resume on its own. */
    ASSERT(!plan_budget_allows());
    ASSERT(!plan_budget_allows());
}

TEST(test_token_ceiling_trips_independently_of_request_count)
{
    plan_budget_configure(0u /* requests unlimited */, 1000u);

    plan_budget_note(400, true);
    ASSERT(plan_budget_allows());
    plan_budget_note(400, true);
    ASSERT(plan_budget_allows());
    plan_budget_note(400, true); /* 1200 >= 1000 */
    ASSERT(!plan_budget_allows());
    ASSERT(plan_budget_state() == PLAN_BUDGET_TRIP_TOKENS);
    ASSERT(plan_budget_tokens() == 1200u);
    ASSERT(plan_budget_requests() == 3u);
}

/**
 * THE case this module exists for. A response body arrives but its token count
 * cannot be read — Google renames a field, or gemini_parse.c regresses. If that
 * charged zero, the token ceiling would silently become no ceiling at all and
 * nothing would report it: every log line would still show a healthy budget
 * while the robot spent all night.
 */
TEST(test_a_response_with_no_usage_is_charged_the_assumed_cost)
{
    plan_budget_configure(0u, 0u);

    plan_budget_note(-1, true);

    ASSERT(plan_budget_tokens() == (uint64_t)PLAN_BUDGET_ASSUMED_TOKENS);
    ASSERT(plan_budget_assumed() == 1u);
    ASSERT(plan_budget_requests() == 1u);
}

TEST(test_assumed_charging_can_trip_the_token_ceiling_on_its_own)
{
    plan_budget_configure(0u, 2u * PLAN_BUDGET_ASSUMED_TOKENS);

    plan_budget_note(-1, true);
    ASSERT(plan_budget_allows());
    plan_budget_note(-1, true);
    ASSERT(!plan_budget_allows());
    ASSERT(plan_budget_state() == PLAN_BUDGET_TRIP_TOKENS);
    ASSERT(plan_budget_assumed() == 2u);
}

/**
 * The mirror of the case above, and the reason charging is not simply "always
 * assume the worst": a request that never produced a body spent no tokens. If
 * an outage charged phantom tokens, losing WiFi overnight would trip the fuse —
 * and a robot with no network is the one situation where it is provably not
 * spending anything.
 */
TEST(test_a_request_that_never_got_a_response_charges_no_tokens)
{
    plan_budget_configure(0u, 0u);

    for (int i = 0; i < 50; i++) {
        plan_budget_note(-1, false);
    }

    ASSERT(plan_budget_tokens() == 0u);
    ASSERT(plan_budget_assumed() == 0u);
    /* Still counted against the request ceiling — a loop hammering a dead
     * endpoint is exactly as unbounded as one hammering a live one. */
    ASSERT(plan_budget_requests() == 50u);
}

TEST(test_a_failed_request_still_counts_toward_the_request_ceiling)
{
    plan_budget_configure(2u, 0u);

    plan_budget_note(-1, false);
    ASSERT(plan_budget_allows());
    plan_budget_note(-1, false);
    ASSERT(!plan_budget_allows());
    ASSERT(plan_budget_state() == PLAN_BUDGET_TRIP_REQUESTS);
}

TEST(test_resume_clears_the_trip_and_the_counters)
{
    plan_budget_configure(1u, 0u);
    plan_budget_note(500, true);
    ASSERT(!plan_budget_allows());

    plan_budget_resume();

    ASSERT(plan_budget_allows());
    ASSERT(plan_budget_requests() == 0u);
    ASSERT(plan_budget_tokens() == 0u);
    ASSERT(plan_budget_assumed() == 0u);
    /* Ceilings survive a resume — resuming means "I have looked at it, carry
     * on", not "forget how I configured you". */
    uint32_t max_requests = 0u;
    plan_budget_get(&max_requests, NULL);
    ASSERT(max_requests == 1u);
}

TEST(test_zero_ceilings_disable_the_fuse_entirely)
{
    plan_budget_configure(0u, 0u);

    for (int i = 0; i < 10000; i++) {
        plan_budget_note(10000, true);
    }

    ASSERT(plan_budget_allows());
    ASSERT(plan_budget_tokens() == 100000000ull);
}

/**
 * A wrapped counter reads as a small number, which is byte-for-byte what a
 * fresh boot looks like — so a 32-bit accumulator would silently UN-trip the
 * fuse partway through a long run. The accumulator is 64-bit and saturating;
 * this pins the boundary a real workload can actually reach.
 */
TEST(test_token_accounting_survives_the_32_bit_boundary)
{
    plan_budget_configure(0u, 0u);

    for (int i = 0; i < 3; i++) {
        plan_budget_note(2000000000, true); /* 6e9 total, past uint32 */
    }
    ASSERT(plan_budget_tokens() == 6000000000ull);

    /* And the ceiling still binds above the 32-bit boundary. */
    plan_budget_configure(0u, 4000000000u);
    ASSERT(!plan_budget_allows());
}

TEST(test_request_ceiling_is_reported_first_when_both_have_tripped)
{
    plan_budget_configure(1u, 1u);
    plan_budget_note(5000, true);

    /* Both are over. The request count is the one an operator can convert to
     * wall-clock without knowing anything about the workload, so it is the more
     * useful half of a double trip to name. */
    ASSERT(plan_budget_state() == PLAN_BUDGET_TRIP_REQUESTS);
}

TEST(test_init_restores_defaults)
{
    plan_budget_configure(7u, 9u);
    plan_budget_note(-1, true);
    plan_budget_init();

    uint32_t max_requests = 0u;
    uint32_t max_tokens = 0u;
    plan_budget_get(&max_requests, &max_tokens);
    ASSERT(max_requests == PLAN_BUDGET_MAX_REQUESTS_DEFAULT);
    ASSERT(max_tokens == PLAN_BUDGET_MAX_TOKENS_DEFAULT);
    ASSERT(plan_budget_requests() == 0u);
    ASSERT(plan_budget_allows());
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    printf("plan_budget host tests\n");

    run_test_a_fresh_budget_allows_requests();
    run_test_request_ceiling_trips_and_stays_tripped();
    run_test_token_ceiling_trips_independently_of_request_count();
    run_test_a_response_with_no_usage_is_charged_the_assumed_cost();
    run_test_assumed_charging_can_trip_the_token_ceiling_on_its_own();
    run_test_a_request_that_never_got_a_response_charges_no_tokens();
    run_test_a_failed_request_still_counts_toward_the_request_ceiling();
    run_test_resume_clears_the_trip_and_the_counters();
    run_test_zero_ceilings_disable_the_fuse_entirely();
    run_test_token_accounting_survives_the_32_bit_boundary();
    run_test_request_ceiling_is_reported_first_when_both_have_tripped();
    run_test_init_restores_defaults();

    if (g_failures == 0) {
        printf("PASS\n");
        return 0;
    }
    printf("FAILED: %d assertion(s)\n", g_failures);
    return 1;
}
