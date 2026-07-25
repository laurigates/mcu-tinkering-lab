/**
 * @file test_dialogue_style.c
 * @brief Host tests for the per-utterance dialogue variation module.
 *
 * The bug this module exists to prevent is behavioural — "the robot opens
 * every line the same way" — so the tests are mostly *distribution* tests
 * rather than single-input/single-output ones: no immediate repeats, every
 * entry reachable, and the avoid-list tracking what was actually said.
 *
 * The rest is buffer discipline. The composed directive lands in a fixed
 * prompt buffer on an 8 KB task stack, and the openings are extracted from
 * UTF-8 Finnish, so the truncation paths are exercised deliberately: a cut
 * that lands mid multi-byte sequence would put invalid UTF-8 into a JSON
 * request body.
 */

#include "dialogue_style.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

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
 * Fixtures
 * ========================================================================= */

static const char *const k_openers[] = {
    "Start straight in.",
    "Open with a filler.",
    "Open with an exclamation.",
    "Open with a question.",
};
static const dialogue_pool_t k_opener_pool = {k_openers, DIALOGUE_POOL_COUNT(k_openers)};

static const char *const k_shapes[] = {
    "Keep it short.",
    "Say it flatly.",
    "Make it dry.",
};
static const dialogue_pool_t k_shape_pool = {k_shapes, DIALOGUE_POOL_COUNT(k_shapes)};

static const char *const k_single[] = {"Only option."};
static const dialogue_pool_t k_single_pool = {k_single, DIALOGUE_POOL_COUNT(k_single)};

static const dialogue_pool_t k_empty_pool = {NULL, 0};

/** Reset everything a test could have perturbed. */
static void fresh(uint32_t seed)
{
    dialogue_style_seed(seed);
    dialogue_style_reset_recent();
}

/* =========================================================================
 * Pick
 * ========================================================================= */

static void test_pick_stays_in_pool(void)
{
    fresh(12345u);
    for (int i = 0; i < 500; ++i) {
        const char *s = dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER);
        ASSERT(s != NULL);
        int found = 0;
        for (size_t j = 0; j < k_opener_pool.count; ++j) {
            if (s == k_openers[j]) {
                found = 1;
            }
        }
        ASSERT(found); /* the exact literal, not a copy */
    }
}

static void test_pick_never_repeats_immediately(void)
{
    fresh(99u);
    const char *prev = NULL;
    for (int i = 0; i < 500; ++i) {
        const char *s = dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER);
        ASSERT(s != prev);
        prev = s;
    }
}

static void test_pick_reaches_every_entry(void)
{
    /* A no-repeat rule implemented as "always advance by one" would also pass
     * the test above while producing a fully predictable cycle, so check the
     * draws actually spread over the pool. */
    fresh(7u);
    int seen[4] = {0};
    for (int i = 0; i < 400; ++i) {
        const char *s = dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER);
        for (size_t j = 0; j < k_opener_pool.count; ++j) {
            if (s == k_openers[j]) {
                seen[j]++;
            }
        }
    }
    for (size_t j = 0; j < k_opener_pool.count; ++j) {
        ASSERT(seen[j] > 20); /* ~100 expected each; 20 is a very loose floor */
    }
}

static void test_pick_slots_are_independent(void)
{
    /* Drawing a shape must not consume the opener's no-repeat state, or the
     * two pools would gate each other. */
    fresh(4242u);
    const char *prev = NULL;
    for (int i = 0; i < 200; ++i) {
        const char *o = dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER);
        (void)dialogue_style_pick(&k_shape_pool, DIALOGUE_SLOT_SHAPE);
        ASSERT(o != prev);
        prev = o;
    }
}

static void test_pick_degenerate_pools(void)
{
    fresh(1u);
    /* A single-entry pool has no alternative — it must return it, not NULL. */
    for (int i = 0; i < 5; ++i) {
        ASSERT(dialogue_style_pick(&k_single_pool, DIALOGUE_SLOT_OPENER) == k_single[0]);
    }
    ASSERT(dialogue_style_pick(&k_empty_pool, DIALOGUE_SLOT_OPENER) == NULL);
    ASSERT(dialogue_style_pick(NULL, DIALOGUE_SLOT_OPENER) == NULL);
}

static void test_seed_changes_the_sequence(void)
{
    /* Without this, every boot opens the self-introduction the same way. */
    char a[16][64];
    char b[16][64];
    fresh(1u);
    for (int i = 0; i < 16; ++i) {
        snprintf(a[i], sizeof(a[i]), "%s",
                 dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER));
    }
    fresh(0xDEADBEEFu);
    for (int i = 0; i < 16; ++i) {
        snprintf(b[i], sizeof(b[i]), "%s",
                 dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER));
    }
    int differs = 0;
    for (int i = 0; i < 16; ++i) {
        if (strcmp(a[i], b[i]) != 0) {
            differs = 1;
        }
    }
    ASSERT(differs);
}

static void test_zero_seed_does_not_freeze_the_rng(void)
{
    /* xorshift is absorbing at zero: an unguarded 0 seed would return the same
     * entry forever, which the no-repeat rule would then paper over. */
    fresh(0u);
    const char *first = dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER);
    int varied = 0;
    for (int i = 0; i < 50; ++i) {
        if (dialogue_style_pick(&k_opener_pool, DIALOGUE_SLOT_OPENER) != first) {
            varied = 1;
        }
    }
    ASSERT(varied);
}

/* =========================================================================
 * Recent openings
 * ========================================================================= */

static void test_note_spoken_keeps_first_words(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Asianlaita on oikeastaan niin, että tie on tukossa.");

    char buf[256];
    ASSERT(dialogue_style_recent_openings(buf, sizeof(buf)) > 0);
    ASSERT(strcmp(buf, "\"Asianlaita on oikeastaan\"") == 0);
}

static void test_note_spoken_trims_trailing_punctuation(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Jaahas, tuota noin, edessä on seinä.");

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    ASSERT(strcmp(buf, "\"Jaahas, tuota noin\"") == 0);
}

static void test_recent_lists_newest_first(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Alpha one two three");
    dialogue_style_note_spoken("Beta one two three");

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    ASSERT(strcmp(buf, "\"Beta one two\", \"Alpha one two\"") == 0);
}

static void test_recent_ring_evicts_oldest(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Alpha a a");
    dialogue_style_note_spoken("Beta b b");
    dialogue_style_note_spoken("Gamma c c");
    dialogue_style_note_spoken("Delta d d");

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    ASSERT(strstr(buf, "Delta") != NULL);
    ASSERT(strstr(buf, "Gamma") != NULL);
    ASSERT(strstr(buf, "Beta") != NULL);
    ASSERT(strstr(buf, "Alpha") == NULL); /* pushed out of a 3-slot ring */
}

static void test_repeated_opening_does_not_flood_the_ring(void)
{
    /* The whole point of the avoid-list is to name *several* things to steer
     * away from. A model stuck on one phrase would otherwise fill all three
     * slots with it and the list would stop being useful. */
    fresh(1u);
    dialogue_style_note_spoken("Alpha a a");
    dialogue_style_note_spoken("Beta b b");
    for (int i = 0; i < 10; ++i) {
        dialogue_style_note_spoken("Beta b b but with a different tail each time");
    }

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    ASSERT(strstr(buf, "Alpha") != NULL);
    ASSERT(strcmp(buf, "\"Beta b b\", \"Alpha a a\"") == 0);
}

static void test_note_spoken_ignores_empty_and_null(void)
{
    fresh(1u);
    dialogue_style_note_spoken(NULL);
    dialogue_style_note_spoken("");
    dialogue_style_note_spoken("   ");
    dialogue_style_note_spoken(",,,");

    char buf[256];
    ASSERT(dialogue_style_recent_openings(buf, sizeof(buf)) == 0);
    ASSERT(buf[0] == '\0');
}

static void test_long_first_word_is_dropped_not_split(void)
{
    /* Finnish compounds get long, and the input is UTF-8: cutting at the byte
     * cap could emit half a multi-byte sequence into a JSON request body. A
     * word that cannot fit whole is dropped instead. */
    fresh(1u);
    dialogue_style_note_spoken("Lentokonesuihkuturbiinimoottoriapumekaanikkoaliupseerioppilas kas");

    char buf[256];
    ASSERT(dialogue_style_recent_openings(buf, sizeof(buf)) == 0);
}

static void test_opening_truncates_at_a_word_boundary(void)
{
    fresh(1u);
    /* Three words that together exceed DIALOGUE_OPENING_MAX (32). */
    dialogue_style_note_spoken("kaksitoista kolmetoista neljätoista viisitoista");

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    /* Whatever survived must be a whole-word prefix of the input. */
    ASSERT(strcmp(buf, "\"kaksitoista kolmetoista\"") == 0);
}

static void test_utf8_opening_survives_intact(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Hyvää päivää, minä olen Robocar.");

    char buf[256];
    (void)dialogue_style_recent_openings(buf, sizeof(buf));
    ASSERT(strcmp(buf, "\"Hyvää päivää, minä\"") == 0);
}

static void test_recent_openings_respects_a_small_buffer(void)
{
    fresh(1u);
    dialogue_style_note_spoken("Alpha a a");
    dialogue_style_note_spoken("Beta b b");
    dialogue_style_note_spoken("Gamma c c");

    for (size_t cap = 1; cap < 64; ++cap) {
        char buf[64];
        memset(buf, 0x7F, sizeof(buf));
        const size_t n = dialogue_style_recent_openings(buf, cap);
        ASSERT(n < cap);
        ASSERT(buf[n] == '\0');
        ASSERT(strlen(buf) == n);
        ASSERT((unsigned char)buf[cap] == 0x7Fu); /* nothing written past cap */
    }
    ASSERT(dialogue_style_recent_openings(NULL, 16) == 0);
}

/* =========================================================================
 * Composed directive
 * ========================================================================= */

static void test_directive_contains_an_opener_and_a_shape(void)
{
    fresh(31337u);
    char buf[DIALOGUE_STYLE_MAX];
    const size_t n =
        dialogue_style_directive(&k_opener_pool, &k_shape_pool, NULL, buf, sizeof(buf));
    ASSERT(n > 0);
    ASSERT(strlen(buf) == n);

    int has_opener = 0;
    for (size_t j = 0; j < k_opener_pool.count; ++j) {
        if (strstr(buf, k_openers[j]) != NULL) {
            has_opener = 1;
        }
    }
    int has_shape = 0;
    for (size_t j = 0; j < k_shape_pool.count; ++j) {
        if (strstr(buf, k_shapes[j]) != NULL) {
            has_shape = 1;
        }
    }
    ASSERT(has_opener);
    ASSERT(has_shape);
}

static void test_directive_varies_between_calls(void)
{
    fresh(2024u);
    char prev[DIALOGUE_STYLE_MAX] = {0};
    int distinct = 0;
    for (int i = 0; i < 40; ++i) {
        char buf[DIALOGUE_STYLE_MAX];
        (void)dialogue_style_directive(&k_opener_pool, &k_shape_pool, NULL, buf, sizeof(buf));
        if (strcmp(buf, prev) != 0) {
            distinct++;
        }
        snprintf(prev, sizeof(prev), "%s", buf);
    }
    ASSERT(distinct == 40); /* 4 openers x 3 shapes, neither repeating */
}

static void test_directive_omits_the_avoid_clause_when_nothing_is_recorded(void)
{
    fresh(5u);
    char buf[DIALOGUE_STYLE_MAX];
    (void)dialogue_style_directive(&k_opener_pool, &k_shape_pool, "Do not begin with:", buf,
                                   sizeof(buf));
    ASSERT(strstr(buf, "Do not begin with:") == NULL);
}

static void test_directive_includes_the_avoid_clause_once_recorded(void)
{
    fresh(5u);
    dialogue_style_note_spoken("Asianlaita on oikeastaan niin, että tie on tukossa.");

    char buf[DIALOGUE_STYLE_MAX];
    (void)dialogue_style_directive(&k_opener_pool, &k_shape_pool, "Do not begin with:", buf,
                                   sizeof(buf));
    ASSERT(strstr(buf, "Do not begin with: \"Asianlaita on oikeastaan\".") != NULL);
}

static void test_directive_drops_a_clause_that_will_not_fit(void)
{
    /* A prompt ending "Do not begin with:" with no list reads as a broken
     * instruction — worse than no clause at all. */
    fresh(5u);
    dialogue_style_note_spoken("Asianlaita on oikeastaan niin, että tie on tukossa.");

    for (size_t cap = 1; cap < 200; ++cap) {
        char buf[256];
        memset(buf, 0x7F, sizeof(buf));
        const size_t n =
            dialogue_style_directive(&k_opener_pool, &k_shape_pool, "Do not begin with:", buf, cap);
        ASSERT(n < cap);
        ASSERT(strlen(buf) == n);
        ASSERT((unsigned char)buf[cap] == 0x7Fu);
        if (strstr(buf, "Do not begin with:") != NULL) {
            ASSERT(buf[n - 1] == '.'); /* present means complete */
        }
    }
}

static void test_directive_handles_empty_pools(void)
{
    fresh(5u);
    char buf[DIALOGUE_STYLE_MAX];
    memset(buf, 0x7F, sizeof(buf));
    /* A persona that leaves the pools zeroed simply never varies — it must not
     * emit a dangling "For this one line only:" clause at the call site, which
     * is why 0 has to mean "write nothing". */
    ASSERT(dialogue_style_directive(&k_empty_pool, &k_empty_pool, NULL, buf, sizeof(buf)) == 0);
    ASSERT(buf[0] == '\0');
    ASSERT(dialogue_style_directive(NULL, NULL, NULL, buf, sizeof(buf)) == 0);
    ASSERT(dialogue_style_directive(&k_opener_pool, &k_shape_pool, NULL, NULL, 16) == 0);
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== dialogue_style host tests ===\n\n");

    test_run("pick stays inside the pool", test_pick_stays_in_pool);
    test_run("pick never repeats immediately", test_pick_never_repeats_immediately);
    test_run("pick reaches every entry", test_pick_reaches_every_entry);
    test_run("pick slots are independent", test_pick_slots_are_independent);
    test_run("pick handles degenerate pools", test_pick_degenerate_pools);
    test_run("seed changes the sequence", test_seed_changes_the_sequence);
    test_run("zero seed does not freeze the rng", test_zero_seed_does_not_freeze_the_rng);

    test_run("note_spoken keeps the first words", test_note_spoken_keeps_first_words);
    test_run("note_spoken trims trailing punctuation", test_note_spoken_trims_trailing_punctuation);
    test_run("recent lists newest first", test_recent_lists_newest_first);
    test_run("recent ring evicts the oldest", test_recent_ring_evicts_oldest);
    test_run("a repeated opening does not flood the ring",
             test_repeated_opening_does_not_flood_the_ring);
    test_run("note_spoken ignores empty and NULL", test_note_spoken_ignores_empty_and_null);
    test_run("a long first word is dropped, not split", test_long_first_word_is_dropped_not_split);
    test_run("opening truncates at a word boundary", test_opening_truncates_at_a_word_boundary);
    test_run("utf-8 opening survives intact", test_utf8_opening_survives_intact);
    test_run("recent_openings respects a small buffer",
             test_recent_openings_respects_a_small_buffer);

    test_run("directive contains an opener and a shape",
             test_directive_contains_an_opener_and_a_shape);
    test_run("directive varies between calls", test_directive_varies_between_calls);
    test_run("directive omits the avoid clause when nothing is recorded",
             test_directive_omits_the_avoid_clause_when_nothing_is_recorded);
    test_run("directive includes the avoid clause once recorded",
             test_directive_includes_the_avoid_clause_once_recorded);
    test_run("directive drops a clause that will not fit",
             test_directive_drops_a_clause_that_will_not_fit);
    test_run("directive handles empty pools", test_directive_handles_empty_pools);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
