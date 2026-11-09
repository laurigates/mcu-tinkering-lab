/**
 * @file test_note_gate.c
 * @brief Host tests for the planner note gate (ADR-033).
 *
 * The properties a bench cannot stage: the 20th offer and the one after it,
 * a five-minute window across the day-49 millisecond wrap, and several triggers
 * landing in one window.
 */

#include "note_gate.h"

#include <assert.h>
#include <stdio.h>
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

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    note_gate_init();
    fn();
    test_pass++;
    printf("     PASS\n");
}

static void nothing_pending_offers_nothing(void)
{
    ASSERT(note_gate_take(1000) == NOTE_TRIGGER_NONE);
    ASSERT(note_gate_offers() == 0);
}

static void a_trigger_is_offered_once(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(1000) == NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_offers() == 1);
    ASSERT(note_gate_pending() == 0);
    /* Consumed: the next request carries no offer. */
    ASSERT(note_gate_take(1000 + NOTE_GATE_MIN_INTERVAL_MS) == NOTE_TRIGGER_NONE);
}

static void highest_priority_wins_and_all_are_cleared(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLECT);
    note_gate_raise(NOTE_TRIGGER_BLIND);
    note_gate_raise(NOTE_TRIGGER_REPEATS);
    ASSERT(note_gate_take(0) == NOTE_TRIGGER_BLIND);
    ASSERT(note_gate_pending() == 0);
}

static void a_trigger_inside_the_window_waits_for_it(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(10000) == NOTE_TRIGGER_REFLEX);
    note_gate_raise(NOTE_TRIGGER_TRACK_STUCK);
    ASSERT(note_gate_take(10000 + NOTE_GATE_MIN_INTERVAL_MS - 1) == NOTE_TRIGGER_NONE);
    ASSERT(note_gate_wait_ms(10000 + NOTE_GATE_MIN_INTERVAL_MS - 1) == 1);
    ASSERT(note_gate_pending() != 0); /* kept, not dropped */
    ASSERT(note_gate_take(10000 + NOTE_GATE_MIN_INTERVAL_MS) == NOTE_TRIGGER_TRACK_STUCK);
}

static void the_window_survives_the_millisecond_wrap(void)
{
    const uint32_t before_wrap = 0xFFFFFFFFu - 1000u;
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(before_wrap) == NOTE_TRIGGER_REFLEX);
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    /* 2 s later, past the wrap: still inside the window. */
    ASSERT(note_gate_take(before_wrap + 2000u) == NOTE_TRIGGER_NONE);
    ASSERT(note_gate_take(before_wrap + NOTE_GATE_MIN_INTERVAL_MS) == NOTE_TRIGGER_REFLEX);
}

static void the_per_boot_count_trips_and_stays_tripped(void)
{
    uint32_t t = 0;
    for (uint32_t i = 0; i < NOTE_GATE_MAX_PER_BOOT; i++) {
        note_gate_raise(NOTE_TRIGGER_REFLECT);
        ASSERT(note_gate_take(t) == NOTE_TRIGGER_REFLECT);
        t += NOTE_GATE_MIN_INTERVAL_MS;
    }
    ASSERT(note_gate_spent());
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(t) == NOTE_TRIGGER_NONE);
    ASSERT(note_gate_take(t + 100u * NOTE_GATE_MIN_INTERVAL_MS) == NOTE_TRIGGER_NONE);
    ASSERT(note_gate_offers() == NOTE_GATE_MAX_PER_BOOT);
}

static void resume_rearms_the_count_and_the_window(void)
{
    for (uint32_t i = 0; i < NOTE_GATE_MAX_PER_BOOT; i++) {
        note_gate_raise(NOTE_TRIGGER_REFLECT);
        (void)note_gate_take(i * NOTE_GATE_MIN_INTERVAL_MS);
    }
    note_gate_noted();
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    note_gate_resume();
    ASSERT(!note_gate_spent());
    ASSERT(note_gate_written() == 0);
    /* The pending reflex survives the resume and goes out at once. */
    ASSERT(note_gate_take(1) == NOTE_TRIGGER_REFLEX);
}

/* A request that never left the board gives its offer back: offline, a
 * reflex must not be spent on requests the model never saw. */
static void an_unsent_offer_is_given_back(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(1000) == NOTE_TRIGGER_REFLEX);
    note_gate_untake(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_offers() == 0);
    ASSERT(note_gate_wait_ms(1001) == 0);
    ASSERT(note_gate_take(16000) == NOTE_TRIGGER_REFLEX); /* next request offers it */
    /* Only once: a second untake of the same take does nothing. */
    note_gate_untake(NOTE_TRIGGER_REFLEX);
    note_gate_untake(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_offers() == 0);
    ASSERT(note_gate_take(16001) == NOTE_TRIGGER_REFLEX);
    note_gate_untake(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_pending() != 0);
}

/* Untaking restores the window the earlier offer opened, not an open one. */
static void untake_restores_the_previous_window(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    ASSERT(note_gate_take(0) == NOTE_TRIGGER_REFLEX);
    note_gate_raise(NOTE_TRIGGER_BLIND);
    ASSERT(note_gate_take(NOTE_GATE_MIN_INTERVAL_MS) == NOTE_TRIGGER_BLIND);
    note_gate_untake(NOTE_TRIGGER_BLIND);
    ASSERT(note_gate_offers() == 1);
    ASSERT(note_gate_wait_ms(NOTE_GATE_MIN_INTERVAL_MS) == 0);
    ASSERT(note_gate_wait_ms(1) == NOTE_GATE_MIN_INTERVAL_MS - 1);
}

/* A take clears every pending trigger; giving the offer back restores all of
 * them, not only the one that was chosen. */
static void untake_restores_every_pending_trigger(void)
{
    note_gate_raise(NOTE_TRIGGER_REFLEX);
    note_gate_raise(NOTE_TRIGGER_BLIND);
    const uint32_t before = note_gate_pending();
    const note_trigger_t chosen = note_gate_take(0);
    ASSERT(chosen != NOTE_TRIGGER_NONE);
    ASSERT(note_gate_pending() == 0);
    note_gate_untake(chosen);
    ASSERT(note_gate_pending() == before);
}

static void bad_triggers_are_ignored(void)
{
    note_gate_raise(NOTE_TRIGGER_NONE);
    note_gate_raise(NOTE_TRIGGER_COUNT);
    note_gate_raise((note_trigger_t)99);
    ASSERT(note_gate_pending() == 0);
}

static void names_and_reasons(void)
{
    ASSERT(strcmp(note_trigger_name(NOTE_TRIGGER_TRACK_STUCK), "track_stuck") == 0);
    ASSERT(strcmp(note_trigger_reason(NOTE_TRIGGER_NONE), "") == 0);
    for (int t = NOTE_TRIGGER_NONE + 1; t < NOTE_TRIGGER_COUNT; t++) {
        ASSERT(strlen(note_trigger_reason((note_trigger_t)t)) > 20);
    }
    /* The reflex reason states the measurement, not a story about it. */
    ASSERT(strstr(note_trigger_reason(NOTE_TRIGGER_REFLEX), "rangefinder") != NULL);
}

static void kinds_round_trip(void)
{
    for (int k = NOTE_KIND_NONE + 1; k < NOTE_KIND_COUNT; k++) {
        ASSERT(note_kind_from_name(note_kind_name((note_kind_t)k)) == (note_kind_t)k);
    }
    ASSERT(note_kind_from_name("behavior") == NOTE_KIND_NONE); /* US spelling is not a kind */
    ASSERT(note_kind_from_name("") == NOTE_KIND_NONE);
    ASSERT(note_kind_from_name(NULL) == NOTE_KIND_NONE);
}

static void text_is_one_trimmed_line(void)
{
    char out[NOTE_TEXT_MAX + 1];
    ASSERT(note_text_clean(out, sizeof(out), "  I keep\n\n hitting\tthe  chair.  ") > 0);
    ASSERT(strcmp(out, "I keep hitting the chair.") == 0);
    ASSERT(note_text_clean(out, sizeof(out), " \n\t ") == 0);
    ASSERT(out[0] == '\0');
    ASSERT(note_text_clean(out, sizeof(out), NULL) == 0);
}

static void text_is_capped_at_the_limit(void)
{
    char in[NOTE_TEXT_MAX * 2];
    memset(in, 'a', sizeof(in) - 1);
    in[sizeof(in) - 1] = '\0';
    char out[NOTE_TEXT_MAX * 2];
    ASSERT(note_text_clean(out, sizeof(out), in) == NOTE_TEXT_MAX);
    char small[8];
    ASSERT(note_text_clean(small, sizeof(small), in) == 7);
}

static void a_cut_never_splits_a_utf8_sequence(void)
{
    /* "ä" is two bytes; fill so the cut lands between them. */
    char in[NOTE_TEXT_MAX + 8];
    memset(in, 'a', NOTE_TEXT_MAX - 1);
    memcpy(in + NOTE_TEXT_MAX - 1, "\xc3\xa4tail", 7);
    in[NOTE_TEXT_MAX - 1 + 7] = '\0';
    char out[NOTE_TEXT_MAX + 1];
    const size_t n = note_text_clean(out, sizeof(out), in);
    ASSERT(n == NOTE_TEXT_MAX - 1);
    ASSERT(((unsigned char)out[n - 1] & 0x80u) == 0);
}

int main(void)
{
    printf("=== note_gate host tests ===\n");
    test_run("nothing_pending_offers_nothing", nothing_pending_offers_nothing);
    test_run("a_trigger_is_offered_once", a_trigger_is_offered_once);
    test_run("highest_priority_wins_and_all_are_cleared",
             highest_priority_wins_and_all_are_cleared);
    test_run("a_trigger_inside_the_window_waits_for_it", a_trigger_inside_the_window_waits_for_it);
    test_run("the_window_survives_the_millisecond_wrap", the_window_survives_the_millisecond_wrap);
    test_run("the_per_boot_count_trips_and_stays_tripped",
             the_per_boot_count_trips_and_stays_tripped);
    test_run("resume_rearms_the_count_and_the_window", resume_rearms_the_count_and_the_window);
    test_run("an_unsent_offer_is_given_back", an_unsent_offer_is_given_back);
    test_run("untake_restores_the_previous_window", untake_restores_the_previous_window);
    test_run("untake_restores_every_pending_trigger", untake_restores_every_pending_trigger);
    test_run("bad_triggers_are_ignored", bad_triggers_are_ignored);
    test_run("names_and_reasons", names_and_reasons);
    test_run("kinds_round_trip", kinds_round_trip);
    test_run("text_is_one_trimmed_line", text_is_one_trimmed_line);
    test_run("text_is_capped_at_the_limit", text_is_capped_at_the_limit);
    test_run("a_cut_never_splits_a_utf8_sequence", a_cut_never_splits_a_utf8_sequence);
    printf("\n%d/%d tests passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
