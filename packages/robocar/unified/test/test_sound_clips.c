/**
 * @file test_sound_clips.c
 * @brief Host tests for sound clip names, the line split and the reflex honk.
 *
 * The property that matters most is that a sound tag never survives into the
 * text handed to TTS: there it would be read out as a word ("hakasulku horn").
 * After that, that the clip lands on the side of the words the model put it,
 * and that the reflex cannot honk at a person standing in front of a robot that
 * was not trying to move, nor honk again within the cooldown, day-49 wrap
 * included.
 */

#include "sound_clips_core.h"

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

/** Split a literal and compare the words and both sides. */
static void check_split(const char *in, const char *words, sound_clip_t lead, sound_clip_t trail)
{
    char buf[320];
    strncpy(buf, in, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    const sound_split_t s = sound_clips_split(buf);
    if (strcmp(buf, words) != 0 || s.lead != lead || s.trail != trail) {
        printf(
            "     in=\"%s\"\n     got=\"%s\" lead=%d trail=%d\n     want=\"%s\" lead=%d trail=%d\n",
            in, buf, s.lead, s.trail, words, lead, trail);
    }
    ASSERT(strcmp(buf, words) == 0);
    ASSERT(s.lead == lead);
    ASSERT(s.trail == trail);
}

static void test_names_round_trip(void)
{
    for (int i = 0; i < SOUND_CLIP_COUNT; i++) {
        ASSERT(sound_clip_name((sound_clip_t)i) != NULL);
        ASSERT(sound_clip_from_name(sound_clip_name((sound_clip_t)i)) == (sound_clip_t)i);
    }
    ASSERT(sound_clip_name(SOUND_CLIP_NONE) == NULL);
    ASSERT(sound_clip_name(SOUND_CLIP_COUNT) == NULL);
    ASSERT(sound_clip_from_name("trumpet") == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_name("Horn") == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_name(NULL) == SOUND_CLIP_NONE);
}

static void test_tag_match_is_exact(void)
{
    ASSERT(sound_clip_from_tag("[horn]", 6) == SOUND_CLIP_HORN);
    ASSERT(sound_clip_from_tag("[aoogah]", 8) == SOUND_CLIP_AOOGAH);
    ASSERT(sound_clip_from_tag("[horns]", 7) == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_tag("[hor]", 5) == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_tag("[sighs]", 7) == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_tag("horn", 4) == SOUND_CLIP_NONE);
    ASSERT(sound_clip_from_tag("[]", 2) == SOUND_CLIP_NONE);
}

/* The tag is gone from every line, wherever it sat. */
static void test_tag_never_reaches_tts(void)
{
    check_split("[horn] Anteeksi, pääsisinkö ohi?", "Anteeksi, pääsisinkö ohi?", SOUND_CLIP_HORN,
                SOUND_CLIP_NONE);
    check_split("Pois alta! [aoogah]", "Pois alta!", SOUND_CLIP_NONE, SOUND_CLIP_AOOGAH);
    check_split("Kas [bell] vain.", "Kas vain.", SOUND_CLIP_NONE, SOUND_CLIP_BELL);
}

/* A line that is only a tag leaves no words, so no TTS request is made. */
static void test_tag_only_line_is_empty(void)
{
    check_split("[horn]", "", SOUND_CLIP_HORN, SOUND_CLIP_NONE);
    check_split("  [boing]  ", "", SOUND_CLIP_BOING, SOUND_CLIP_NONE);
}

/* Delivery tags are not words: a sound after one still leads, and the
 * delivery tag stays for TTS to perform. */
static void test_delivery_tags_kept_and_not_words(void)
{
    check_split("[sighs] [horn] Taas este.", "[sighs] Taas este.", SOUND_CLIP_HORN,
                SOUND_CLIP_NONE);
    check_split("Hups. [laughs] [boing]", "Hups. [laughs]", SOUND_CLIP_NONE, SOUND_CLIP_BOING);
}

/* Only the first sound plays; later ones are removed with it. */
static void test_only_first_sound_plays(void)
{
    check_split("[horn] Hei [bell]", "Hei", SOUND_CLIP_HORN, SOUND_CLIP_NONE);
    check_split("Hei [bell] [horn]", "Hei", SOUND_CLIP_NONE, SOUND_CLIP_BELL);
}

static void test_untagged_and_unclosed(void)
{
    check_split("Tavallinen lause.", "Tavallinen lause.", SOUND_CLIP_NONE, SOUND_CLIP_NONE);
    /* An unknown or unclosed bracket is not a sound; speech_tags_sanitize()
     * deals with those before the line is queued. */
    check_split("Hei [hor", "Hei [hor", SOUND_CLIP_NONE, SOUND_CLIP_NONE);
    check_split("Hei [trumpet]", "Hei [trumpet]", SOUND_CLIP_NONE, SOUND_CLIP_NONE);
    ASSERT(sound_clips_split(NULL).lead == SOUND_CLIP_NONE);
}

static void test_reflex_needs_blocked_motion(void)
{
    ASSERT(!sound_clips_reflex_due(1000, 0, false, false));
    ASSERT(sound_clips_reflex_due(1000, 0, false, true));
}

static void test_reflex_cooldown(void)
{
    const uint32_t last = 50000;
    ASSERT(!sound_clips_reflex_due(last + SOUND_REFLEX_COOLDOWN_MS - 1, last, true, true));
    ASSERT(sound_clips_reflex_due(last + SOUND_REFLEX_COOLDOWN_MS, last, true, true));
}

/* The millisecond clock wraps at day 49; a honk just before the wrap must not
 * read as 49 days ago, nor mute the reflex forever after it. */
static void test_reflex_cooldown_across_wrap(void)
{
    const uint32_t last = 0xFFFFFFFFu - 5000u;
    ASSERT(!sound_clips_reflex_due(last + 10000u, last, true, true)); /* wrapped, 10 s later */
    ASSERT(sound_clips_reflex_due(last + SOUND_REFLEX_COOLDOWN_MS, last, true, true));
}

int main(void)
{
    printf("=== sound clip host tests ===\n\n");

    test_run("names_round_trip", test_names_round_trip);
    test_run("tag_match_is_exact", test_tag_match_is_exact);
    test_run("tag_never_reaches_tts", test_tag_never_reaches_tts);
    test_run("tag_only_line_is_empty", test_tag_only_line_is_empty);
    test_run("delivery_tags_kept_and_not_words", test_delivery_tags_kept_and_not_words);
    test_run("only_first_sound_plays", test_only_first_sound_plays);
    test_run("untagged_and_unclosed", test_untagged_and_unclosed);
    test_run("reflex_needs_blocked_motion", test_reflex_needs_blocked_motion);
    test_run("reflex_cooldown", test_reflex_cooldown);
    test_run("reflex_cooldown_across_wrap", test_reflex_cooldown_across_wrap);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
