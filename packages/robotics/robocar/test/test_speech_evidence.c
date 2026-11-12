/**
 * @file test_speech_evidence.c
 * @brief Host tests for the planner's speech-evidence clause (issue #618).
 *
 * The planner request carries the camera JPEG and no audio. The audio-only
 * clause used to assert "something happened out of frame or behind you. Remark
 * on that", and the model, with nothing to describe, invented sources: bangs,
 * echoes, moving furniture. That failure is a fluent sentence, so nothing on a
 * bench reports it as an error — the wording itself is what has to be pinned.
 *
 * Issue #631 is the same failure one step earlier: every clause said "since you
 * last spoke", including on cycles where the gate opened without comparing
 * anything — its first impression after boot, or the scene gate switched off
 * (`voice scene 0`). A sense that was not compared now contributes no sentence.
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

#define UNKNOWN SPEECH_SENSE_UNKNOWN
#define SAME SPEECH_SENSE_SAME
#define CHANGED SPEECH_SENSE_CHANGED

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
    const char *c = speech_evidence_clause(SAME, CHANGED);
    ASSERT(!CONTAINS(c, "something happened"));
    ASSERT(!CONTAINS(c, "behind you"));
    ASSERT(!CONTAINS(c, "out of frame"));
    ASSERT(!CONTAINS(c, "Remark on that"));
}

static void test_audio_only_states_the_source_is_unknown(void)
{
    const char *c = speech_evidence_clause(SAME, CHANGED);
    ASSERT(CONTAINS(c, "NO audio"));
    ASSERT(CONTAINS(c, "source of the sound is unknown"));
    ASSERT(CONTAINS(c, "Do NOT name, describe or guess"));
    /* At most a non-specific acknowledgement or question. */
    ASSERT(CONTAINS(c, "ask what happened"));
    /* And it still says the view did not change, so the model does not go
     * looking for a visual cause either. */
    ASSERT(CONTAINS(c, "view has NOT changed"));
    ASSERT(CONTAINS(c, "sound level or character has changed"));
    ASSERT(!CONTAINS(c, "SOUNDS different"));
}

/* The combined case has the same blind spot: the model still has no audio. */
static void test_view_and_audio_also_forbid_naming_a_source(void)
{
    const char *c = speech_evidence_clause(CHANGED, CHANGED);
    ASSERT(CONTAINS(c, "view has changed"));
    ASSERT(CONTAINS(c, "NO audio"));
    ASSERT(CONTAINS(c, "Do NOT name, describe or guess"));
    ASSERT(!CONTAINS(c, "something happened"));
    ASSERT(!CONTAINS(c, "SOUNDS different"));
    /* It still reports the sound change, and points the remark at the view —
     * the one thing in this request the model can actually describe. */
    ASSERT(CONTAINS(c, "sound level or character has also changed"));
    ASSERT(CONTAINS(c, "Remark on what you can see"));
}

static void test_scene_only_says_nothing_about_sound(void)
{
    const char *c = speech_evidence_clause(CHANGED, SAME);
    ASSERT(CONTAINS(c, "view has changed"));
    ASSERT(!CONTAINS(c, "sound"));
    ASSERT(!CONTAINS(c, "audio"));
}

/* No evidence, no assertion. The caller only asks when one sense fired, but an
 * unmeasured change must never be stated as one (stateless-model-gating §4). */
static void test_no_evidence_asserts_nothing(void)
{
    ASSERT(strcmp(speech_evidence_clause(SAME, SAME), "") == 0);
}

/* A gate's verdict is a comparison only when it actually compared. Novel with
 * nothing compared — the first impression, or a disabled scene gate — is not
 * evidence of a change, and not-novel with nothing compared is not evidence of
 * "no change" either (issue #631). */
static void test_a_sense_is_only_known_when_it_was_compared(void)
{
    ASSERT(speech_sense_from_gate(true, true) == CHANGED);
    ASSERT(speech_sense_from_gate(false, true) == SAME);
    ASSERT(speech_sense_from_gate(true, false) == UNKNOWN);
    ASSERT(speech_sense_from_gate(false, false) == UNKNOWN);
}

/* Boot: neither gate has a spoken-about reference, both report novel, and the
 * old clause told the model the view and the sound had changed since it last
 * spoke — before it had spoken at all. */
static void test_first_impression_claims_no_comparison(void)
{
    ASSERT(strcmp(speech_evidence_clause(UNKNOWN, UNKNOWN), "") == 0);
}

/* The view gate opened without comparing (first frame, or `voice scene 0`) and
 * the sound gate compared and found nothing: there is nothing true to say. */
static void test_uncompared_view_says_nothing(void)
{
    ASSERT(strcmp(speech_evidence_clause(UNKNOWN, SAME), "") == 0);
}

/* The sound did change since the robot last spoke, but no view comparison was
 * made: the clause must say nothing about the view in either direction — "The
 * view has NOT changed" was the issue #631 wording — while keeping every #618
 * guard. */
static void test_sound_change_with_an_uncompared_view(void)
{
    const char *c = speech_evidence_clause(UNKNOWN, CHANGED);
    ASSERT(!CONTAINS(c, "view"));
    ASSERT(CONTAINS(c, "sound level or character has changed since you last spoke"));
    ASSERT(CONTAINS(c, "NO audio"));
    ASSERT(CONTAINS(c, "source of the sound is unknown"));
    ASSERT(CONTAINS(c, "Do NOT name, describe or guess"));
    ASSERT(CONTAINS(c, "ask what happened"));
    ASSERT(!CONTAINS(c, "something happened"));
}

/* A real view change with an uncompared sound gate (its first impression of the
 * room) is the plain scene clause: nothing about sound. */
static void test_view_change_with_an_uncompared_sound(void)
{
    const char *c = speech_evidence_clause(CHANGED, UNKNOWN);
    ASSERT(strcmp(c, speech_evidence_clause(CHANGED, SAME)) == 0);
    ASSERT(!CONTAINS(c, "sound"));
}

/* Every combination: a sentence about a sense appears only when that sense
 * measured what the sentence says, and "since you last spoke" only when one of
 * them measured a change. */
static void test_each_sentence_needs_its_measurement(void)
{
    const speech_sense_t all[3] = {UNKNOWN, SAME, CHANGED};
    for (int v = 0; v < 3; v++) {
        for (int a = 0; a < 3; a++) {
            const char *c = speech_evidence_clause(all[v], all[a]);
            if (all[v] != CHANGED && all[a] != CHANGED) {
                ASSERT(strcmp(c, "") == 0);
            }
            if (CONTAINS(c, "view has changed")) {
                ASSERT(all[v] == CHANGED);
            }
            if (CONTAINS(c, "view has NOT changed")) {
                ASSERT(all[v] == SAME);
            }
            if (CONTAINS(c, "sound")) {
                ASSERT(all[a] == CHANGED);
            }
        }
    }
}

/* Each clause is spliced into "...sentence out loud. %sSay something..." and
 * relies on its own trailing space. */
static void test_every_non_empty_clause_ends_in_a_space(void)
{
    const speech_sense_t cases[4][2] = {
        {CHANGED, SAME}, {SAME, CHANGED}, {CHANGED, CHANGED}, {UNKNOWN, CHANGED}};
    for (int i = 0; i < 4; i++) {
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
    test_run("a sense is only known when it was compared",
             test_a_sense_is_only_known_when_it_was_compared);
    test_run("first impression claims no comparison", test_first_impression_claims_no_comparison);
    test_run("an uncompared view says nothing", test_uncompared_view_says_nothing);
    test_run("sound change with an uncompared view", test_sound_change_with_an_uncompared_view);
    test_run("view change with an uncompared sound", test_view_change_with_an_uncompared_sound);
    test_run("each sentence needs its measurement", test_each_sentence_needs_its_measurement);
    printf("=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
