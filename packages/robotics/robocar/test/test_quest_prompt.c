/**
 * @file test_quest_prompt.c
 * @brief Host tests for the planner's quest clause and its two tools (ADR-039 §3).
 *
 * What a bench cannot stage: every text field at its cap at once, the console
 * closing a quest while a request is in flight, a reply carrying a tool the
 * request withheld, and the 24th request on one quest.
 */

#include "quest_prompt.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        fflush(stdout);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static quest_log_t s_log;
static quest_ration_t s_ration;
static quest_offer_t s_offer;
static quest_reply_t s_reply;
static quest_apply_result_t s_res;

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    quest_log_init(&s_log);
    memset(&s_ration, 0, sizeof(s_ration));
    memset(&s_offer, 0x5A, sizeof(s_offer)); /* build must overwrite every field */
    quest_reply_clear(&s_reply);
    memset(&s_res, 0x5A, sizeof(s_res));
    fn();
    test_pass++;
    printf("     PASS\n");
}

static quest_time_t at(int64_t s)
{
    return quest_time_from(QUEST_LOG_EPOCH_VALID_S + s, 0);
}

static uint16_t open_active(quest_kind_t kind, const char *title, const char *why)
{
    uint16_t id = 0;
    ASSERT(quest_log_open(&s_log, kind, title, why, at(0), &id) == QUEST_OK);
    ASSERT(quest_log_set_active(&s_log, id) == QUEST_OK);
    return id;
}

static const quest_prompt_inputs_t k_static = {.view_changed = false, .tracked_last_cycle = false};
static const quest_prompt_inputs_t k_view = {.view_changed = true, .tracked_last_cycle = false};
static const quest_prompt_inputs_t k_track = {.view_changed = false, .tracked_last_cycle = true};

static void build(const quest_prompt_inputs_t *in)
{
    quest_prompt_build(&s_log, in, &s_ration, &s_offer);
}

static void fill(char *out, size_t n, char c)
{
    memset(out, c, n);
    out[n] = '\0';
}

/* ---- The offer ----------------------------------------------------------- */

static void no_active_quest_offers_nothing(void)
{
    uint16_t id = 0;
    ASSERT(quest_log_open(&s_log, QUEST_KIND_FIND, "the red cup", "", at(0), &id) == QUEST_OK);
    build(&k_view); /* open, not active */
    ASSERT(s_offer.quest_id == 0);
    ASSERT(!s_offer.offer_discovery);
    ASSERT(!s_offer.offer_done);
    ASSERT(s_offer.clause[0] == '\0');
    ASSERT(s_ration.requests == 0);
}

static void an_ask_quest_gets_no_clause(void)
{
    open_active(QUEST_KIND_ASK, "what is the plant called", "");
    build(&k_view);
    ASSERT(s_offer.quest_id == 0);
    ASSERT(s_offer.clause[0] == '\0');
}

static void a_static_view_offers_neither_tool(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "Lauri lost it");
    build(&k_static);
    ASSERT(s_offer.quest_id == id);
    ASSERT(!s_offer.offer_discovery);
    ASSERT(!s_offer.offer_done);
    ASSERT(strstr(s_offer.clause, "the red cup") != NULL);
    ASSERT(strstr(s_offer.clause, "Lauri lost it") != NULL);
    ASSERT(strstr(s_offer.clause, "find") != NULL);
    /* A tool that is not declared is not named either. */
    ASSERT(strstr(s_offer.clause, "log_discovery") == NULL);
    ASSERT(strstr(s_offer.clause, "quest_done") == NULL);
}

static void a_changed_view_offers_discovery_and_says_why(void)
{
    open_active(QUEST_KIND_EXPLORE, "the hallway", "");
    build(&k_view);
    ASSERT(s_offer.offer_discovery);
    ASSERT(strstr(s_offer.clause, "log_discovery") != NULL);
    ASSERT(strstr(s_offer.clause, "view has changed") != NULL);
    /* No why given: no empty "Why:" sentence. */
    ASSERT(strstr(s_offer.clause, "Why") == NULL);
}

static void a_track_on_the_last_cycle_offers_discovery(void)
{
    open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_track);
    ASSERT(s_offer.offer_discovery);
    ASSERT(strstr(s_offer.clause, "track something on your last turn") != NULL);
}

static void done_is_offered_with_discovery(void)
{
    open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    ASSERT(s_offer.offer_done);
    ASSERT(strstr(s_offer.clause, "quest_done") != NULL);
}

static void done_is_offered_once_the_quest_has_a_note(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    ASSERT(quest_log_note(&s_log, id, 'C', "a blue mug on the desk", at(1)) == QUEST_OK);
    build(&k_static);
    ASSERT(!s_offer.offer_discovery);
    ASSERT(s_offer.offer_done);
}

static void done_is_offered_after_half_the_ration(void)
{
    open_active(QUEST_KIND_EXPLORE, "the hallway", "");
    for (uint32_t i = 0; i < QUEST_DONE_AFTER_REQUESTS; i++) {
        build(&k_static);
        ASSERT(!s_offer.offer_done);
    }
    ASSERT(s_ration.requests == QUEST_DONE_AFTER_REQUESTS);
    build(&k_static);
    ASSERT(s_offer.offer_done);
}

static void the_ration_restarts_for_another_quest(void)
{
    const uint16_t a = open_active(QUEST_KIND_EXPLORE, "the hallway", "");
    for (int i = 0; i < 5; i++) {
        build(&k_static);
    }
    ASSERT(s_ration.quest_id == a);
    ASSERT(s_ration.requests == 5);
    const uint16_t b = open_active(QUEST_KIND_EXPLORE, "the kitchen", "");
    build(&k_static);
    ASSERT(s_ration.quest_id == b);
    ASSERT(s_ration.requests == 1);
}

/* ---- The clause ---------------------------------------------------------- */

static void the_clause_quotes_the_newest_two_notes(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    ASSERT(quest_log_note(&s_log, id, 'C', "first note about a chair", at(1)) == QUEST_OK);
    ASSERT(quest_log_note(&s_log, id, 'L', "second note about a lamp", at(2)) == QUEST_OK);
    ASSERT(quest_log_note(&s_log, id, 'R', "third note about a window", at(3)) == QUEST_OK);
    build(&k_static);
    ASSERT(strstr(s_offer.clause, "first note") == NULL);
    const char *second = strstr(s_offer.clause, "second note about a lamp");
    const char *third = strstr(s_offer.clause, "third note about a window");
    ASSERT(second != NULL && third != NULL);
    ASSERT(second < third); /* oldest first, as a log reads */
}

static void notes_of_another_quest_are_not_quoted(void)
{
    uint16_t other = 0;
    ASSERT(quest_log_open(&s_log, QUEST_KIND_EXPLORE, "the hallway", "", at(0), &other) ==
           QUEST_OK);
    ASSERT(quest_log_note(&s_log, other, 'C', "a coat on a hook", at(1)) == QUEST_OK);
    open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_static);
    ASSERT(strstr(s_offer.clause, "coat") == NULL);
}

static void every_field_at_its_cap_still_fits_with_the_tool_sentences(void)
{
    char title[QUEST_LOG_TITLE_MAX + 1];
    char why[QUEST_LOG_WHY_MAX + 1];
    fill(title, QUEST_LOG_TITLE_MAX, 't');
    fill(why, QUEST_LOG_WHY_MAX, 'w');
    const uint16_t id = open_active(QUEST_KIND_REVISIT, title, why);
    char note[QUEST_LOG_NOTE_MAX + 1];
    fill(note, QUEST_LOG_NOTE_MAX, 'a');
    ASSERT(quest_log_note(&s_log, id, 'C', note, at(1)) == QUEST_OK);
    /* Different words, so the near-repeat check keeps both. */
    for (size_t i = 0; i < QUEST_LOG_NOTE_MAX; i += 4) {
        memcpy(&note[i], (i / 4) % 2 ? "xyz " : "pqr ", 4);
    }
    note[QUEST_LOG_NOTE_MAX] = '\0';
    ASSERT(quest_log_note(&s_log, id, 'L', note, at(2)) == QUEST_OK);

    build(&k_view);
    const size_t len = strlen(s_offer.clause);
    ASSERT(len <= QUEST_CLAUSE_MAX);
    ASSERT(strstr(s_offer.clause, title) != NULL);
    ASSERT(strstr(s_offer.clause, why) != NULL);
    ASSERT(strstr(s_offer.clause, "log_discovery") != NULL);
    ASSERT(strstr(s_offer.clause, "quest_done") != NULL);
    /* The newest note is quoted whole; the older one is the one dropped. */
    ASSERT(strstr(s_offer.clause, s_log.notes[1].text) != NULL);
    ASSERT(strstr(s_offer.clause, "aaaa") == NULL);
    /* The clause ends on a whole sentence, never mid-quote. */
    ASSERT(len > 2 && s_offer.clause[len - 1] == ' ' && s_offer.clause[len - 2] == '.');
}

/* ---- Applying the reply -------------------------------------------------- */

static void a_discovery_is_stored_with_pose_and_time(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "a red cup beside the lamp");
    ASSERT(quest_prompt_apply(&s_log, &s_offer, &s_reply, 'L', at(7), &s_res));
    ASSERT(s_res.noted && s_res.note_err == QUEST_OK);
    ASSERT(!s_res.stale && !s_res.unoffered && !s_res.closed);
    ASSERT(quest_log_note_count(&s_log, id) == 1);
    ASSERT(s_log.notes[0].pose == 'L');
    ASSERT(s_log.notes[0].at.s == at(7).s);
    ASSERT(strcmp(s_log.notes[0].text, "a red cup beside the lamp") == 0);
}

static void a_near_repeat_discovery_is_reported_not_stored(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    ASSERT(quest_log_note(&s_log, id, 'C', "a red cup beside the lamp", at(1)) == QUEST_OK);
    build(&k_view);
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "A red cup beside the lamp!");
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(7), &s_res));
    ASSERT(!s_res.noted && s_res.note_err == QUEST_ERR_REPEAT);
    ASSERT(quest_log_note_count(&s_log, id) == 1);
}

static void done_closes_as_done_or_dropped(void)
{
    const uint16_t a = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    s_reply.done = true;
    s_reply.achieved = true;
    snprintf(s_reply.outcome, sizeof(s_reply.outcome), "found it by the lamp");
    ASSERT(quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.closed && s_res.closed_as == QUEST_STATE_DONE);
    const quest_t *q = quest_log_find(&s_log, a);
    ASSERT(q->state == QUEST_STATE_DONE);
    ASSERT(q->closed.s == at(9).s);
    ASSERT(strcmp(q->outcome, "found it by the lamp") == 0);
    ASSERT(quest_log_active(&s_log) == NULL);

    const uint16_t b = open_active(QUEST_KIND_FIND, "the green ball", "");
    build(&k_view);
    s_reply.achieved = false;
    ASSERT(quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(10), &s_res));
    ASSERT(s_res.closed && s_res.closed_as == QUEST_STATE_DROPPED);
    ASSERT(quest_log_find(&s_log, b)->state == QUEST_STATE_DROPPED);
}

static void the_discovery_lands_before_the_close(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "the red cup is under the desk");
    s_reply.done = true;
    s_reply.achieved = true;
    ASSERT(quest_prompt_apply(&s_log, &s_offer, &s_reply, 'F', at(9), &s_res));
    ASSERT(s_res.noted && s_res.closed);
    ASSERT(quest_log_note_count(&s_log, id) == 1);
}

static void calls_the_request_withheld_are_dropped(void)
{
    const uint16_t id = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_static); /* neither tool */
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "a red cup");
    s_reply.done = true;
    s_reply.achieved = true;
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.unoffered);
    ASSERT(!s_res.noted && !s_res.closed);
    ASSERT(quest_log_note_count(&s_log, id) == 0);
    ASSERT(quest_log_find(&s_log, id)->state == QUEST_STATE_ACTIVE);
}

static void a_close_the_request_withheld_is_dropped_alone(void)
{
    const uint16_t id = open_active(QUEST_KIND_EXPLORE, "the hallway", "");
    build(&k_static); /* no note, no evidence, ration fresh: neither tool */
    ASSERT(!s_offer.offer_done);
    s_reply.done = true;
    s_reply.achieved = true;
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.unoffered);
    ASSERT(!s_res.closed);
    ASSERT(quest_log_find(&s_log, id)->state == QUEST_STATE_ACTIVE);
}

static void a_reply_with_no_clause_changes_nothing(void)
{
    build(&k_view); /* no quest at all */
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "a red cup");
    s_reply.done = true;
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.unoffered);
    ASSERT(s_log.note_count == 0);
}

static void a_quest_changed_during_the_request_is_left_alone(void)
{
    const uint16_t a = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    /* The console makes another quest active while the request is out. */
    const uint16_t b = open_active(QUEST_KIND_EXPLORE, "the hallway", "");
    snprintf(s_reply.discovery, sizeof(s_reply.discovery), "a red cup");
    s_reply.done = true;
    s_reply.achieved = true;
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.stale);
    ASSERT(s_log.note_count == 0);
    ASSERT(quest_log_find(&s_log, a)->state == QUEST_STATE_OPEN);
    ASSERT(quest_log_find(&s_log, b)->state == QUEST_STATE_ACTIVE);
}

static void a_quest_closed_during_the_request_is_left_alone(void)
{
    const uint16_t a = open_active(QUEST_KIND_FIND, "the red cup", "");
    build(&k_view);
    ASSERT(quest_log_close(&s_log, a, QUEST_STATE_DROPPED, "by hand", at(5)) == QUEST_OK);
    s_reply.done = true;
    s_reply.achieved = true;
    ASSERT(!quest_prompt_apply(&s_log, &s_offer, &s_reply, 'C', at(9), &s_res));
    ASSERT(s_res.stale);
    ASSERT(strcmp(quest_log_find(&s_log, a)->outcome, "by hand") == 0);
}

int main(void)
{
    printf("=== quest_prompt host tests ===\n");
    test_run("no_active_quest_offers_nothing", no_active_quest_offers_nothing);
    test_run("an_ask_quest_gets_no_clause", an_ask_quest_gets_no_clause);
    test_run("a_static_view_offers_neither_tool", a_static_view_offers_neither_tool);
    test_run("a_changed_view_offers_discovery_and_says_why",
             a_changed_view_offers_discovery_and_says_why);
    test_run("a_track_on_the_last_cycle_offers_discovery",
             a_track_on_the_last_cycle_offers_discovery);
    test_run("done_is_offered_with_discovery", done_is_offered_with_discovery);
    test_run("done_is_offered_once_the_quest_has_a_note",
             done_is_offered_once_the_quest_has_a_note);
    test_run("done_is_offered_after_half_the_ration", done_is_offered_after_half_the_ration);
    test_run("the_ration_restarts_for_another_quest", the_ration_restarts_for_another_quest);
    test_run("the_clause_quotes_the_newest_two_notes", the_clause_quotes_the_newest_two_notes);
    test_run("notes_of_another_quest_are_not_quoted", notes_of_another_quest_are_not_quoted);
    test_run("every_field_at_its_cap_still_fits_with_the_tool_sentences",
             every_field_at_its_cap_still_fits_with_the_tool_sentences);
    test_run("a_discovery_is_stored_with_pose_and_time", a_discovery_is_stored_with_pose_and_time);
    test_run("a_near_repeat_discovery_is_reported_not_stored",
             a_near_repeat_discovery_is_reported_not_stored);
    test_run("done_closes_as_done_or_dropped", done_closes_as_done_or_dropped);
    test_run("the_discovery_lands_before_the_close", the_discovery_lands_before_the_close);
    test_run("calls_the_request_withheld_are_dropped", calls_the_request_withheld_are_dropped);
    test_run("a_close_the_request_withheld_is_dropped_alone",
             a_close_the_request_withheld_is_dropped_alone);
    test_run("a_reply_with_no_clause_changes_nothing", a_reply_with_no_clause_changes_nothing);
    test_run("a_quest_changed_during_the_request_is_left_alone",
             a_quest_changed_during_the_request_is_left_alone);
    test_run("a_quest_closed_during_the_request_is_left_alone",
             a_quest_closed_during_the_request_is_left_alone);
    printf("=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? EXIT_SUCCESS : EXIT_FAILURE;
}
