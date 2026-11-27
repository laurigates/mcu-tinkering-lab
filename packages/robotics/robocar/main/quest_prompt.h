/**
 * @file quest_prompt.h
 * @brief What the planner is told about the active quest, and what it may do
 *        about it (ADR-039 §3).
 *
 * While a quest is active, the planner request carries one clause naming it
 * (kind, title, why, its last two notes) and may declare two tools:
 *
 *   - `log_discovery(text)`: offered only on a cycle with view evidence
 *     (plan_activity's `why: V`) or after a `track` goal on the previous
 *     request, so a frame that has not changed cannot be noted over and over;
 *   - `quest_done(outcome, achieved)`: offered once the quest has a note, once
 *     discovery is offered, or once QUEST_DONE_AFTER_REQUESTS requests have
 *     carried the quest.
 *
 * As with `speak` and `note`, a tool the firmware will not accept is withheld
 * rather than discouraged (.claude/rules/stateless-model-gating.md), and the
 * clause states why each tool is there from what the robot measured. An `ask`
 * quest gets no clause: it is a question for a person, and the planner sees
 * none.
 *
 * quest_prompt_apply() writes the reply back into the log. It takes only what
 * the request offered, and only while the quest it named is still the active
 * one, because the console can change the log during the seconds a request
 * takes.
 *
 * Pure C, no ESP-IDF: host-tested in test/test_quest_prompt.c.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "quest_log_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Longest quest clause in bytes (NUL not counted). The title, the why and
 *  both tool sentences always fit; notes are dropped whole, oldest first. */
#define QUEST_CLAUSE_MAX 1024u

/** Requests on one quest after which `quest_done` is offered even with no
 *  note: half of ADR-039's QUEST_TOTAL_REQUESTS (48). */
#define QUEST_DONE_AFTER_REQUESTS 24u

/** Notes quoted in the clause, newest kept. */
#define QUEST_CLAUSE_NOTES 2u

/** Requests that have carried the active quest's clause. Reset when another
 *  quest becomes active. Kept in RAM only: a reboot starts it again. */
typedef struct {
    uint16_t quest_id;
    uint32_t requests;
} quest_ration_t;

/** What the robot measured for this cycle. */
typedef struct {
    bool view_changed;       /**< plan_activity's `why:` holds V */
    bool tracked_last_cycle; /**< the previous request's goal was `track` */
} quest_prompt_inputs_t;

/** What the request about to be made offers about the quest. */
typedef struct {
    /** The active quest named in the clause; 0 for no clause and no tools. */
    uint16_t quest_id;
    bool offer_discovery;
    bool offer_done;
    char clause[QUEST_CLAUSE_MAX + 1];
} quest_offer_t;

/** The quest calls a planner reply carried (gemini_parse_last_quest()). */
typedef struct {
    /** Cleaned `log_discovery` text; "" when there was none. */
    char discovery[QUEST_LOG_NOTE_MAX + 1];
    bool done; /**< a `quest_done` call was present */
    bool achieved;
    char outcome[QUEST_LOG_OUTCOME_MAX + 1];
} quest_reply_t;

/** What quest_prompt_apply() did. */
typedef struct {
    bool stale;           /**< the offered quest is no longer active: nothing applied */
    bool unoffered;       /**< the reply carried a call the request did not offer */
    bool noted;           /**< the discovery was stored */
    quest_err_t note_err; /**< QUEST_OK, or why the discovery was not stored */
    bool closed;          /**< the quest was closed */
    quest_state_t closed_as;
} quest_apply_result_t;

/** Clear @p reply to "no quest calls". */
void quest_reply_clear(quest_reply_t *reply);

/**
 * Decide the offer for the request about to be made and write its clause.
 *
 * With no active quest, or an `ask` one, @p out has quest_id 0 and an empty
 * clause. Otherwise this charges one request to @p ration (resetting it first
 * when the active quest changed), so call it once per request actually built.
 */
void quest_prompt_build(const quest_log_t *log, const quest_prompt_inputs_t *in,
                        quest_ration_t *ration, quest_offer_t *out);

/**
 * Apply @p reply to @p log under what @p offer allowed: the discovery as a
 * note with @p pose and @p now, then the close (DONE when achieved, DROPPED
 * otherwise). @return true when the log changed and should be saved.
 */
bool quest_prompt_apply(quest_log_t *log, const quest_offer_t *offer, const quest_reply_t *reply,
                        char pose, quest_time_t now, quest_apply_result_t *res);

#ifdef __cplusplus
}
#endif
