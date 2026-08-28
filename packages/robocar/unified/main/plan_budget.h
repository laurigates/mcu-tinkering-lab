/**
 * @file plan_budget.h
 * @brief Hard ceiling on what the planner may spend at the Gemini endpoint.
 *
 * The other two mechanisms that keep planner traffic down — the dormancy gate
 * in plan_activity.h and the backoff ladder it drives — are *optimisations*:
 * both decide from on-device sensors whether a request is worth making, and
 * both are therefore only as correct as those sensors. This module is the
 * backstop, and it is deliberately the dumbest thing in the chain: it counts,
 * and past a ceiling it stops the planner regardless of what any detector
 * believes.
 *
 * It exists because of a specific evening. The board was left plugged into a
 * laptop with no speaker attached; the planner ran its unconditional 15 s loop
 * all night at 240 requests/hour and nothing on the device had any notion that
 * it was spending anything. Prepaid quota was the only thing that stopped it.
 * A gate can fail open — ambient_audio.c did exactly that when the microphone
 * was absent, and asserted novelty on every cycle for a whole boot. A counter
 * cannot.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_plan_budget.c
 * builds it on the host with no shims.
 *
 * ## Two ceilings, because the interesting one is not knowable in advance
 *
 * Requests are easy to reason about (a request is a request) and easy to
 * convert to wall-clock: at PLANNER_LOOP_PERIOD_MS = 15 s, 240 of them is an
 * hour of continuous full-rate planning. Tokens are what actually gets billed,
 * and the per-request figure varies with the prompt, the image, and how long
 * the model thinks — so it can only come from measurement. Both ceilings are
 * live; whichever binds first trips the fuse, and plan_budget_state() says
 * which so the console does not have to guess.
 *
 * ## Charging: fail closed on the parse, open on the network
 *
 * A request is counted the moment it is *attempted*, so a request ceiling
 * bounds traffic even if every response is unparseable.
 *
 * Tokens are charged only when a response body came back:
 *
 *   - usageMetadata present -> charge the reported totalTokenCount;
 *   - response body but no usageMetadata -> charge PLAN_BUDGET_ASSUMED_TOKENS.
 *
 * The second case is the load-bearing one. Google renaming a field, or a parse
 * regression, would otherwise make every request cost *zero* and turn the token
 * ceiling into no ceiling at all — a fuse that silently stops being a fuse,
 * which is worse than not having one. plan_budget_assumed() counts how often
 * that path was taken, so "the budget is running blind" is visible in `plan`
 * rather than inferred later from a bill.
 *
 * A request that never reached a response (DNS failure, TLS timeout, no WiFi)
 * charges no tokens, because it genuinely spent none. Charging phantom tokens
 * there would let an outage trip the fuse, which is the wrong direction: an
 * offline robot is already not spending anything.
 *
 * ## Tripping is terminal until a human says otherwise
 *
 * There is no rolling window and no auto-reset. A fuse that resets itself is
 * not a backstop against the case this module exists for — an unattended board
 * would resume spending on its own the moment the window rolled. Recovery is
 * `plan resume` on the console, an MQTT command, or a reboot.
 */

#ifndef PLAN_BUDGET_H
#define PLAN_BUDGET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Requests allowed per boot before the fuse trips.
 *
 *  500 is roughly two hours of continuous full-rate planning at a 15 s period.
 *  It is a starting point chosen to bound an unattended overnight run, NOT a
 *  measured constant — a bench session that genuinely wants more should raise
 *  it from the console (`plan requests <n>`) rather than have this default
 *  quietly sized for the longest thing anyone might do. */
#define PLAN_BUDGET_MAX_REQUESTS_DEFAULT 500u

/** Tokens allowed per boot before the fuse trips.
 *
 *  Sized as PLAN_BUDGET_MAX_REQUESTS_DEFAULT * PLAN_BUDGET_ASSUMED_TOKENS so
 *  the two ceilings bind at about the same time under the assumed cost, and
 *  whichever is wrong shows up as the one that trips first. */
#define PLAN_BUDGET_MAX_TOKENS_DEFAULT 2000000u

/** Charged for a completed request whose response carried no usageMetadata.
 *
 *  Deliberately an over-estimate rather than a mean: this value is only ever
 *  used when the real figure could not be read, and the failure mode to avoid
 *  is a ceiling that under-counts its way into never binding. Replace it with a
 *  measured number — every parsed response logs `tokens: prompt=… output=…
 *  total=…` from gemini_parse.c, and `plan` reports the running mean. */
#define PLAN_BUDGET_ASSUMED_TOKENS 4000u

/** Why the planner is being refused, or PLAN_BUDGET_OK. */
typedef enum {
    PLAN_BUDGET_OK = 0,        /**< Under both ceilings.                        */
    PLAN_BUDGET_TRIP_REQUESTS, /**< Request ceiling reached.                   */
    PLAN_BUDGET_TRIP_TOKENS,   /**< Token ceiling reached.                      */
} plan_budget_state_t;

/** @brief Reset counters and ceilings to their defaults and clear any trip. */
void plan_budget_init(void);

/**
 * @brief Set the two ceilings. 0 disables that ceiling.
 *
 * Disabling is offered because a bench session pointed at a moving target
 * legitimately wants to run long, and an operator who has to comment out a
 * #define to do that will instead learn to raise the default for everyone.
 * Both at 0 means no fuse — `plan` says so in as many words.
 */
void plan_budget_configure(uint32_t max_requests, uint32_t max_tokens);

/** @brief Read the current ceilings. Either pointer may be NULL. */
void plan_budget_get(uint32_t *max_requests, uint32_t *max_tokens);

/**
 * @brief Whether another planner request may be made.
 *
 * False once either ceiling has been reached, and stays false until
 * plan_budget_resume(). Cheap enough to call every cycle.
 */
bool plan_budget_allows(void);

/**
 * @brief Charge one attempted planner request.
 *
 * Call once per request that was actually sent, success or failure, from the
 * single choke point in gemini_backend_plan() — not from the planner loop.
 * A second caller of the endpoint then gets the fuse for free rather than
 * silently spending outside it, the same reason the activity_trace hooks live
 * in gemini_http_post().
 *
 * @param total_tokens  totalTokenCount from usageMetadata, or a negative value
 *                      when the response carried none (charges
 *                      PLAN_BUDGET_ASSUMED_TOKENS and increments the assumed
 *                      counter).
 * @param got_response  false when the request never produced a body at all
 *                      (network failure) — the request is still counted, no
 *                      tokens are charged, and @p total_tokens is ignored.
 */
void plan_budget_note(int32_t total_tokens, bool got_response);

/** @brief Clear the trip and zero the counters. The operator's `plan resume`. */
void plan_budget_resume(void);

/** @brief PLAN_BUDGET_OK, or which ceiling tripped. */
plan_budget_state_t plan_budget_state(void);

/** @brief Requests charged since init or the last resume. */
uint32_t plan_budget_requests(void);

/** @brief Tokens charged since init or the last resume. */
uint64_t plan_budget_tokens(void);

/**
 * @brief Requests charged at the assumed cost because usageMetadata was absent.
 *
 * Non-zero means the token ceiling is partly guessing. A value equal to
 * plan_budget_requests() means it is guessing entirely, which is the signature
 * of a parse that has stopped finding the field rather than of an expensive
 * workload.
 */
uint32_t plan_budget_assumed(void);

#ifdef __cplusplus
}
#endif

#endif /* PLAN_BUDGET_H */
