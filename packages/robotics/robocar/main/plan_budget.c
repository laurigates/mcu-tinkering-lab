/**
 * @file plan_budget.c
 * @brief Planner spend ceiling. See the header for why this is a counter and
 *        not another gate.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_plan_budget.c
 * builds it on the host with no shims.
 */

#include "plan_budget.h"

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

static uint32_t s_max_requests = PLAN_BUDGET_MAX_REQUESTS_DEFAULT;
static uint32_t s_max_tokens = PLAN_BUDGET_MAX_TOKENS_DEFAULT;

static uint32_t s_requests;
static uint64_t s_tokens;
static uint32_t s_assumed;

void plan_budget_init(void)
{
    s_max_requests = PLAN_BUDGET_MAX_REQUESTS_DEFAULT;
    s_max_tokens = PLAN_BUDGET_MAX_TOKENS_DEFAULT;
    s_requests = 0u;
    s_tokens = 0u;
    s_assumed = 0u;
}

void plan_budget_configure(uint32_t max_requests, uint32_t max_tokens)
{
    s_max_requests = max_requests;
    s_max_tokens = max_tokens;
}

void plan_budget_get(uint32_t *max_requests, uint32_t *max_tokens)
{
    if (max_requests) {
        *max_requests = s_max_requests;
    }
    if (max_tokens) {
        *max_tokens = s_max_tokens;
    }
}

plan_budget_state_t plan_budget_state(void)
{
    /* Requests are reported first when both have tripped: it is the ceiling an
     * operator can convert to wall-clock without knowing anything about the
     * workload, so it is the more useful half of a double trip to name. */
    if (s_max_requests != 0u && s_requests >= s_max_requests) {
        return PLAN_BUDGET_TRIP_REQUESTS;
    }
    if (s_max_tokens != 0u && s_tokens >= (uint64_t)s_max_tokens) {
        return PLAN_BUDGET_TRIP_TOKENS;
    }
    return PLAN_BUDGET_OK;
}

bool plan_budget_allows(void)
{
    return plan_budget_state() == PLAN_BUDGET_OK;
}

void plan_budget_note(int32_t total_tokens, bool got_response)
{
    /* Saturate rather than wrap. A wrapped counter reads as a small number,
     * which is indistinguishable from a fresh boot and would silently un-trip
     * the fuse — the same sentinel-zero confusion .claude/rules/
     * diagnose-at-the-failure-point.md warns about, with the fuse as the
     * casualty. Reaching UINT32_MAX requests is not a scenario anyone will
     * hit; being wrong in the un-trip direction if they did is not acceptable. */
    if (s_requests < UINT32_MAX) {
        s_requests++;
    }

    if (!got_response) {
        /* Nothing came back, so nothing was spent. Charging an assumed cost
         * here would let a WiFi outage trip the fuse — the one situation in
         * which the robot is provably not spending anything. */
        return;
    }

    uint32_t charge;
    if (total_tokens < 0) {
        /* A body came back and the token count could not be read from it. Fail
         * CLOSED: charge the assumed cost, and count the fact that we did.
         * Charging zero here is how a token ceiling quietly stops being one. */
        charge = PLAN_BUDGET_ASSUMED_TOKENS;
        if (s_assumed < UINT32_MAX) {
            s_assumed++;
        }
    } else {
        charge = (uint32_t)total_tokens;
    }

    const uint64_t next = s_tokens + (uint64_t)charge;
    s_tokens = (next < s_tokens) ? UINT64_MAX : next;
}

void plan_budget_resume(void)
{
    s_requests = 0u;
    s_tokens = 0u;
    s_assumed = 0u;
}

uint32_t plan_budget_requests(void)
{
    return s_requests;
}

uint64_t plan_budget_tokens(void)
{
    return s_tokens;
}

uint32_t plan_budget_assumed(void)
{
    return s_assumed;
}
