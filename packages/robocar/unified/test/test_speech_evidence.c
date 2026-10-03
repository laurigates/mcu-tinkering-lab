/**
 * @file test_speech_evidence.c
 * @brief Host tests for the planner's speech-evidence clause (issue #618).
 *
 * The planner request carries the camera JPEG and no audio. The audio-only
 * clause used to assert "something happened out of frame or behind you. Remark
 * on that", and the model, with nothing to describe, invented sources: bangs,
 * echoes, moving furniture. That failure is a fluent sentence, so nothing on a
 * bench reports it as an error — the wording itself is what has to be pinned.
 */

#include "speech_evidence.h"

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
#define CONTAINS(hay, needle) (strstr((hay), (needle)) != NULL)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* The clause the model actually read on an audio-only opening before #618. */
static void test_audio_only_drops_the_assertive_wording(void)
{
    const char *c = speech_evidence_clause(false, true);
    ASSERT(!CONTAINS(c, "something happened"));
    ASSERT(!CONTAINS(c, "behind you"));
    ASSERT(!CONTAINS(c, "out of frame"));
    ASSERT(!CONTAINS(c, "Remark on that"));
}

static void test_audio_only_states_the_source_is_unknown(void)
{
    const char *c = speech_evidence_clause(false, true);
    ASSERT(CONTAINS(c, "NO audio"));
    ASSERT(CONTAINS(c, "source of the sound is unknown"));
    ASSERT(CONTAINS(c, "Do NOT name, describe or guess"));
    /* At most a non-specific acknowledgement or question. */
    ASSERT(CONTAINS(c, "ask what happened"));
    /* And it still says the view did not change, so the model does not go
     * looking for a visual cause either. */
    ASSERT(CONTAINS(c, "view has NOT changed"));
}

/* The combined case has the same blind spot: the model still has no audio. */
static void test_view_and_audio_also_forbid_naming_a_source(void)
{
    const char *c = speech_evidence_clause(true, true);
    ASSERT(CONTAINS(c, "view has changed"));
    ASSERT(CONTAINS(c, "NO audio"));
    ASSERT(CONTAINS(c, "Do NOT name, describe or guess"));
    ASSERT(!CONTAINS(c, "something happened"));
}

static void test_scene_only_says_nothing_about_sound(void)
{
    const char *c = speech_evidence_clause(true, false);
    ASSERT(CONTAINS(c, "view has changed"));
    ASSERT(!CONTAINS(c, "sound"));
    ASSERT(!CONTAINS(c, "audio"));
}

/* No evidence, no assertion. The caller only asks when one sense fired, but an
 * unmeasured change must never be stated as one (stateless-model-gating §4). */
static void test_no_evidence_asserts_nothing(void)
{
    ASSERT(strcmp(speech_evidence_clause(false, false), "") == 0);
}

/* Each clause is spliced into "...sentence out loud. %sSay something..." and
 * relies on its own trailing space. */
static void test_every_non_empty_clause_ends_in_a_space(void)
{
    const bool cases[3][2] = {{true, false}, {false, true}, {true, true}};
    for (int i = 0; i < 3; i++) {
        const char *c = speech_evidence_clause(cases[i][0], cases[i][1]);
        const size_t n = strlen(c);
        ASSERT(n > 0u);
        ASSERT(c[n - 1u] == ' ');
    }
}

int main(void)
{
    printf("=== speech_evidence host tests ===\n");
    test_run("audio-only drops the assertive wording", test_audio_only_drops_the_assertive_wording);
    test_run("audio-only states the source is unknown",
             test_audio_only_states_the_source_is_unknown);
    test_run("view+audio also forbids naming a source",
             test_view_and_audio_also_forbid_naming_a_source);
    test_run("scene-only says nothing about sound", test_scene_only_says_nothing_about_sound);
    test_run("no evidence asserts nothing", test_no_evidence_asserts_nothing);
    test_run("every non-empty clause ends in a space", test_every_non_empty_clause_ends_in_a_space);
    printf("=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
