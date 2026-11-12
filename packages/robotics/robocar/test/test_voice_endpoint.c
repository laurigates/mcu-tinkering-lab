/**
 * @file test_voice_endpoint.c
 * @brief Host tests for end-of-speech detection in a hands-free voice turn
 *        (issue #616).
 *
 * The expensive failures are timing ones a bench cannot isolate: a turn cut off
 * before the minimum, a pause that never ends the turn, a ceiling that is not
 * enforced while someone keeps talking, and a clock comparison that breaks
 * across the uint32 millisecond wrap at day 49 — either ending every turn on its
 * first frame or never ending one on silence. Frames are 64 ms, as on the device.
 */

#include "voice_endpoint.h"

#include <assert.h>
#include <stdio.h>

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

#define FLOOR 30
#define SPEECH (FLOOR + 20)
#define QUIET (FLOOR + 1)

static voice_endpoint_cfg_t cfg(void)
{
    const voice_endpoint_cfg_t c = {
        .min_ms = 1000u, .max_ms = 6000u, .silence_ms = 700u, .margin_db = 6u};
    return c;
}

static void test_speech_is_measured_against_the_floor(void)
{
    ASSERT(voice_endpoint_is_speech(FLOOR + 6, FLOOR, 6));
    ASSERT(!voice_endpoint_is_speech(FLOOR + 5, FLOOR, 6));
    ASSERT(!voice_endpoint_is_speech(FLOOR - 10, FLOOR, 6));
    /* Margin 0 makes every frame speech, so only the ceiling ends a turn. */
    ASSERT(voice_endpoint_is_speech(FLOOR - 10, FLOOR, 0));
}

static void test_defaults_are_the_documented_ones(void)
{
    const voice_endpoint_cfg_t c = voice_endpoint_default_cfg(4321u);
    ASSERT(c.max_ms == 4321u);
    ASSERT(c.min_ms == VOICE_ENDPOINT_MIN_MS_DEFAULT);
    ASSERT(c.silence_ms == VOICE_ENDPOINT_SILENCE_MS_DEFAULT);
    ASSERT(c.margin_db == VOICE_ENDPOINT_MARGIN_DB_DEFAULT);
}

static void test_ends_after_the_silence_hangover(void)
{
    const voice_endpoint_cfg_t c = cfg();
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 0u);

    /* Speech for 25 frames (to 1600 ms), then quiet. */
    for (uint32_t t = 64u; t <= 1600u; t += 64u) {
        ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, t) == VOICE_ENDPOINT_CONTINUE);
    }
    const uint32_t last_speech = 1600u;

    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, last_speech + 699u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, last_speech + 700u) ==
           VOICE_ENDPOINT_END_SILENCE);
}

static void test_speech_restarts_the_silence_timer(void)
{
    /* The pause between "Teuvo," and the question must not end the turn. */
    const voice_endpoint_cfg_t c = cfg();
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 0u);

    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, 1000u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 1600u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, 1650u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 2300u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 2350u) == VOICE_ENDPOINT_END_SILENCE);
    ASSERT(ep.speech_frames == 2u);
}

static void test_silence_cannot_end_a_turn_before_the_minimum(void)
{
    /* begin() counts as speech, and the hangover (700) is shorter than the
     * minimum (1000): quiet from the start must still record the minimum. */
    const voice_endpoint_cfg_t c = cfg();
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 0u);

    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 700u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 999u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 1000u) == VOICE_ENDPOINT_END_SILENCE);
}

static void test_begin_counts_as_speech(void)
{
    /* With a minimum shorter than the hangover, a turn that is quiet from the
     * cue onward still waits the full hangover measured from begin(). */
    voice_endpoint_cfg_t c = cfg();
    c.min_ms = 100u;
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 5000u);

    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 5000u + 699u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, 5000u + 700u) == VOICE_ENDPOINT_END_SILENCE);
}

static void test_the_ceiling_ends_a_turn_that_never_goes_quiet(void)
{
    const voice_endpoint_cfg_t c = cfg();
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 0u);

    for (uint32_t t = 64u; t < 6000u; t += 64u) {
        ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, t) == VOICE_ENDPOINT_CONTINUE);
    }
    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, 6000u) == VOICE_ENDPOINT_END_MAX);
}

static void test_margin_zero_runs_to_the_ceiling(void)
{
    voice_endpoint_cfg_t c = cfg();
    c.margin_db = 0u;
    voice_endpoint_t ep;
    voice_endpoint_begin(&ep, &c, 0u);

    ASSERT(voice_endpoint_update(&ep, QUIET - 20, FLOOR, 3000u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET - 20, FLOOR, 5999u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET - 20, FLOOR, 6000u) == VOICE_ENDPOINT_END_MAX);
}

static void test_survives_the_uint32_wrap(void)
{
    /* esp_timer_get_time()/1000 wraps every ~49.7 days. Start 300 ms before it
     * and run a whole turn across it: speech until start+1500, then quiet. */
    const voice_endpoint_cfg_t c = cfg();
    voice_endpoint_t ep;
    const uint32_t start = UINT32_MAX - 300u;
    voice_endpoint_begin(&ep, &c, start);

    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, start + 200u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, start + 1500u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT((uint32_t)(start + 1500u) < start); /* it really did wrap */

    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, start + 2199u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, QUIET, FLOOR, start + 2200u) == VOICE_ENDPOINT_END_SILENCE);

    /* And the ceiling, measured across the wrap. */
    voice_endpoint_begin(&ep, &c, start);
    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, start + 5999u) == VOICE_ENDPOINT_CONTINUE);
    ASSERT(voice_endpoint_update(&ep, SPEECH, FLOOR, start + 6000u) == VOICE_ENDPOINT_END_MAX);
}

static void test_verdicts_have_names(void)
{
    ASSERT(voice_endpoint_verdict_name(VOICE_ENDPOINT_END_SILENCE)[0] == 's');
    ASSERT(voice_endpoint_verdict_name(VOICE_ENDPOINT_END_MAX)[0] == 'm');
    ASSERT(voice_endpoint_verdict_name(VOICE_ENDPOINT_CONTINUE)[0] != '\0');
}

int main(void)
{
    printf("=== voice_endpoint host tests ===\n\n");

    test_run("speech is measured against the floor", test_speech_is_measured_against_the_floor);
    test_run("defaults are the documented ones", test_defaults_are_the_documented_ones);
    test_run("ends after the silence hangover", test_ends_after_the_silence_hangover);
    test_run("speech restarts the silence timer", test_speech_restarts_the_silence_timer);
    test_run("silence cannot end a turn before the minimum",
             test_silence_cannot_end_a_turn_before_the_minimum);
    test_run("begin counts as speech", test_begin_counts_as_speech);
    test_run("the ceiling ends a turn that never goes quiet",
             test_the_ceiling_ends_a_turn_that_never_goes_quiet);
    test_run("margin 0 runs to the ceiling", test_margin_zero_runs_to_the_ceiling);
    test_run("survives the uint32 wrap", test_survives_the_uint32_wrap);
    test_run("verdicts have names", test_verdicts_have_names);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
