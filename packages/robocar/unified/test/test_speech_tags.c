/**
 * @file test_speech_tags.c
 * @brief Host tests for the inline TTS delivery-tag filter.
 *
 * The property under test is that nothing unrecognised reaches the speaker: a
 * bracketed run the TTS model does not know is liable to be *read aloud*, so a
 * leak here makes the robot announce its own stage directions. The tests are
 * therefore weighted toward the adversarial cases (invented tags, unclosed
 * brackets, tag spam) rather than the happy path.
 */

#include "speech_tags.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

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

/** Sanitize a literal in a writable buffer and compare against @p expect. */
static void check_sanitize(const char *in, const char *expect, size_t expect_removed)
{
    char buf[512];
    strncpy(buf, in, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    const size_t removed = speech_tags_sanitize(buf);
    if (strcmp(buf, expect) != 0) {
        printf("     in=\"%s\"\n     got=\"%s\"\n     want=\"%s\"\n", in, buf, expect);
    }
    ASSERT(strcmp(buf, expect) == 0);
    ASSERT(removed == expect_removed);
}

/* An allowed tag survives untouched — this is the whole point of the feature. */
static void test_keeps_allowed_tags(void)
{
    check_sanitize("[sighs] Jaahas, taas este edessä.", "[sighs] Jaahas, taas este edessä.", 0);
    check_sanitize("Kas vain. [laughs]", "Kas vain. [laughs]", 0);
    check_sanitize("[whispers] Hiljaa nyt.", "[whispers] Hiljaa nyt.", 0);
}

/* Text with no brackets at all must come through byte-identical. */
static void test_untagged_text_unchanged(void)
{
    check_sanitize("Hyvää päivää, minä olen Robocar.", "Hyvää päivää, minä olen Robocar.", 0);
    check_sanitize("", "", 0);
}

/* An invented tag is the failure this module exists to prevent: left in place
 * the TTS model reads it out as words. */
static void test_removes_unknown_tags(void)
{
    check_sanitize("[robottiäänellä] Hyvää päivää.", "Hyvää päivää.", 1);
    check_sanitize("[pauses dramatically] Kas.", "Kas.", 1);
    check_sanitize("Kas. [MUSIC PLAYS]", "Kas.", 1);
}

/* A tag matches only in its exact documented form — no case folding, no
 * near-misses. Anything else is an invented tag by another name. */
static void test_tag_match_is_exact(void)
{
    check_sanitize("[Sighs] Jaahas.", "Jaahas.", 1);
    check_sanitize("[sigh] Jaahas.", "Jaahas.", 1);
    check_sanitize("[ sighs ] Jaahas.", "Jaahas.", 1);
}

/* Beyond the per-line cap, even allowed tags are dropped — a sentence wearing
 * four stage directions reads as parody, not character. */
static void test_caps_tags_per_line(void)
{
    check_sanitize("[sighs] Kas. [laughs] Vai niin. [gasp] Oho. [bored] No.",
                   "[sighs] Kas. [laughs] Vai niin. Oho. No.", 2);
}

/* An unclosed bracket is a truncated tag; speaking "[whis" is worse than
 * dropping the tail. */
static void test_unclosed_bracket_drops_tail(void)
{
    check_sanitize("Kas vain. [whis", "Kas vain.", 1);
    check_sanitize("[sighs] Kas. [unfinished", "[sighs] Kas.", 1);
}

/* Removal must not leave the double spaces or leading space that would
 * otherwise show up in logs and in the recent-openings list. */
static void test_collapses_whitespace_after_removal(void)
{
    check_sanitize("Kas [MUSIC] vain.", "Kas vain.", 1);
    check_sanitize("[MUSIC] Kas vain.", "Kas vain.", 1);
    check_sanitize("Kas vain [MUSIC]", "Kas vain", 1);
}

/* strip() takes *everything* out, including allowed tags: its consumers want
 * the words alone. */
static void test_strip_removes_all_tags(void)
{
    char out[256];

    const size_t n = speech_tags_strip("[sighs] Jaahas, taas este edessä.", out, sizeof(out));
    ASSERT(strcmp(out, "Jaahas, taas este edessä.") == 0);
    ASSERT(n == strlen(out));

    speech_tags_strip("[whispers] Kas [laughs] vain.", out, sizeof(out));
    ASSERT(strcmp(out, "Kas vain.") == 0);

    speech_tags_strip("Ei tageja lainkaan.", out, sizeof(out));
    ASSERT(strcmp(out, "Ei tageja lainkaan.") == 0);
}

/* strip() must never overrun a short destination, and must still terminate. */
static void test_strip_truncates_safely(void)
{
    char out[8];
    const size_t n = speech_tags_strip("[sighs] Jaahas, taas este edessä.", out, sizeof(out));

    ASSERT(n < sizeof(out));
    ASSERT(out[n] == '\0');
    ASSERT(strncmp(out, "Jaahas,", 7) == 0);
}

/* NULL guards. */
static void test_null_args(void)
{
    char out[16];

    ASSERT(speech_tags_sanitize(NULL) == 0);
    ASSERT(speech_tags_strip(NULL, out, sizeof(out)) == 0);
    ASSERT(out[0] == '\0');
    ASSERT(speech_tags_strip("x", NULL, 16) == 0);
    ASSERT(speech_tags_strip("x", out, 0) == 0);
}

int main(void)
{
    printf("=== speech tag filter host tests ===\n\n");

    test_run("keeps_allowed_tags", test_keeps_allowed_tags);
    test_run("untagged_text_unchanged", test_untagged_text_unchanged);
    test_run("removes_unknown_tags", test_removes_unknown_tags);
    test_run("tag_match_is_exact", test_tag_match_is_exact);
    test_run("caps_tags_per_line", test_caps_tags_per_line);
    test_run("unclosed_bracket_drops_tail", test_unclosed_bracket_drops_tail);
    test_run("collapses_whitespace_after_removal", test_collapses_whitespace_after_removal);
    test_run("strip_removes_all_tags", test_strip_removes_all_tags);
    test_run("strip_truncates_safely", test_strip_truncates_safely);
    test_run("null_args", test_null_args);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
