/**
 * @file test_voice_fx_core.c
 * @brief Host tests for the retro-robot voice effect.
 *
 * The load-bearing test here is the (1-g) input trim. A feedback comb without
 * it has DC gain 1/(1-g), so it does not merely sound louder — it pins the
 * output at the rail and the "resonator" becomes a clipper. That is the exact
 * bug in the design sketch this module was written from, and it is invisible on
 * a bench because a clipped robot voice still sounds like a robot voice.
 *
 * The rest pin things a bench cannot stage at all: that `voice fx off` is a
 * byte-exact bypass rather than a unity-gain round trip through the arithmetic,
 * that the delay line is exactly D samples long, and that a full-scale positive
 * sample never comes out negative — the int16 wrap that turns a peak into
 * full-scale noise.
 */

#include "voice_fx_core.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
    printf("    PASS\n");
}

#define RATE 24000u

static void fill(int16_t *buf, size_t n, int16_t v)
{
    for (size_t i = 0; i < n; i++) {
        buf[i] = v;
    }
}

/* =========================================================================
 * The trim
 * ========================================================================= */

/** Half-scale DC in, at maximum feedback, must not reach the rail.
 *
 * With the trim the comb converges to y = x, so a 0.5 input leaves 0.5 into the
 * saturator and comes out near 0.70 full scale. Without it the comb converges
 * to x/(1-g) = 3.3, the saturator is driven deep into its flat region and every
 * sample pins at 32767.
 *
 * MUTATION-CHECKED: replacing `dry * x` with plain `x` in voice_fx_apply()
 * fails this with a steady-state value of exactly 32767.
 */
static void test_the_input_trim_keeps_a_loud_signal_off_the_rail(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    ASSERT(voice_fx_set_feedback(&fx, VOICE_FX_FEEDBACK_MAX));

    int16_t buf[2048];
    fill(buf, 2048, 16384); /* half scale */
    voice_fx_apply(&fx, buf, 2048);

    /* Sample well past the comb's settling time. */
    for (size_t i = 1500; i < 2048; i++) {
        ASSERT(buf[i] < 30000);
        ASSERT(buf[i] > 0);
    }
}

/** Full-scale DC must stay positive — the wrap this module's cast can produce. */
static void test_full_scale_never_wraps_sign(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    ASSERT(voice_fx_set_feedback(&fx, VOICE_FX_FEEDBACK_MAX));
    ASSERT(voice_fx_set_drive(&fx, VOICE_FX_DRIVE_MAX));

    int16_t buf[2048];
    fill(buf, 2048, 32767);
    voice_fx_apply(&fx, buf, 2048);
    for (size_t i = 0; i < 2048; i++) {
        ASSERT(buf[i] >= 0);
    }

    /* Reset between polarities. The comb has D samples of memory, so without
     * this the negative block is measured while the line is still settling out
     * of the positive one — the transient is legitimately positive, and
     * asserting otherwise tests the harness rather than the cast. */
    voice_fx_reset(&fx);
    fill(buf, 2048, -32768);
    voice_fx_apply(&fx, buf, 2048);
    for (size_t i = 0; i < 2048; i++) {
        ASSERT(buf[i] <= 0);
    }
}

/* =========================================================================
 * Delay geometry
 * ========================================================================= */

/** An impulse must echo at exactly D samples, and nowhere before it. */
static void test_the_echo_lands_exactly_one_delay_later(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    ASSERT(voice_fx_set_body_ms(&fx, 1.0f));             /* 24 samples at 24 kHz */
    ASSERT(voice_fx_set_drive(&fx, VOICE_FX_DRIVE_MIN)); /* near-linear */
    const size_t d = fx.delay_samples;
    ASSERT(d == 24);

    int16_t buf[128];
    memset(buf, 0, sizeof(buf));
    buf[0] = 32767;
    voice_fx_apply(&fx, buf, 128);

    ASSERT(buf[0] != 0);
    for (size_t i = 1; i < d; i++) {
        ASSERT(buf[i] == 0); /* silence until the line comes round */
    }
    ASSERT(buf[d] != 0);     /* first echo */
    ASSERT(buf[2 * d] != 0); /* and it keeps ringing */
    /* Decaying, not growing: the trim again, from the other side. */
    ASSERT(abs(buf[2 * d]) < abs(buf[d]));
}

/** Chunk boundaries must not restart the ring.
 *
 * The player hands over ~128 samples at a time, so a delay line reset per call
 * would ring at the chunk rate rather than at the body frequency. Applying one
 * buffer in two halves must equal applying it whole.
 */
static void test_state_survives_a_chunk_boundary(void)
{
    int16_t whole[256];
    int16_t split[256];
    memset(whole, 0, sizeof(whole));
    whole[0] = 32767;
    memcpy(split, whole, sizeof(whole));

    voice_fx_t a;
    voice_fx_init(&a, RATE);
    ASSERT(voice_fx_set_body_ms(&a, 1.0f));
    voice_fx_apply(&a, whole, 256);

    voice_fx_t b;
    voice_fx_init(&b, RATE);
    ASSERT(voice_fx_set_body_ms(&b, 1.0f));
    voice_fx_apply(&b, split, 128);
    voice_fx_apply(&b, split + 128, 128);

    ASSERT(memcmp(whole, split, sizeof(whole)) == 0);
}

/* =========================================================================
 * Bypass and reset
 * ========================================================================= */

/** Disabled must be byte-exact, not merely quiet. */
static void test_disabled_is_a_byte_exact_bypass(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    voice_fx_set_enabled(&fx, false);

    int16_t buf[64];
    int16_t ref[64];
    for (size_t i = 0; i < 64; i++) {
        buf[i] = (int16_t)(i * 500 - 16000);
    }
    memcpy(ref, buf, sizeof(buf));
    voice_fx_apply(&fx, buf, 64);
    ASSERT(memcmp(buf, ref, sizeof(buf)) == 0);
}

/** Reset must leave no tail — silence in, exact silence out. */
static void test_reset_clears_the_tail(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);

    int16_t loud[256];
    fill(loud, 256, 20000);
    voice_fx_apply(&fx, loud, 256);

    int16_t quiet[256];
    memset(quiet, 0, sizeof(quiet));
    voice_fx_apply(&fx, quiet, 256);
    int nonzero = 0;
    for (size_t i = 0; i < 256; i++) {
        nonzero += (quiet[i] != 0);
    }
    ASSERT(nonzero > 0); /* control: without reset there IS a ringing tail */

    voice_fx_reset(&fx);
    memset(quiet, 0, sizeof(quiet));
    voice_fx_apply(&fx, quiet, 256);
    for (size_t i = 0; i < 256; i++) {
        ASSERT(quiet[i] == 0);
    }
}

/* =========================================================================
 * Parameter validation
 * ========================================================================= */

static void test_out_of_range_parameters_are_refused(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);

    const float body = fx.body_ms;
    const float fb = fx.feedback;
    const float drive = fx.drive;

    ASSERT(!voice_fx_set_body_ms(&fx, VOICE_FX_BODY_MS_MAX + 1.0f));
    ASSERT(!voice_fx_set_body_ms(&fx, 0.0f));
    ASSERT(!voice_fx_set_feedback(&fx, 0.99f)); /* above the stable/usable max */
    ASSERT(!voice_fx_set_feedback(&fx, -0.1f));
    ASSERT(!voice_fx_set_drive(&fx, 0.0f));
    ASSERT(!voice_fx_set_drive(&fx, 100.0f));

    /* Refused means unchanged, not clamped — a clamp hides a typo. */
    ASSERT(fx.body_ms == body);
    ASSERT(fx.feedback == fb);
    ASSERT(fx.drive == drive);
}

/** NaN must be refused. A plain `v < MIN || v > MAX` lets it through, and a NaN
 *  parameter turns every subsequent sample into NaN, which casts to 0 — the
 *  robot goes permanently silent with no error anywhere. */
static void test_nan_parameters_are_refused(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    const float nan_v = (float)NAN;
    ASSERT(!voice_fx_set_body_ms(&fx, nan_v));
    ASSERT(!voice_fx_set_feedback(&fx, nan_v));
    ASSERT(!voice_fx_set_drive(&fx, nan_v));
    ASSERT(fx.delay_samples > 0);
}

/** Zero feedback is legal and must be a clean dry path through the saturator. */
static void test_zero_feedback_is_a_dry_path(void)
{
    voice_fx_t fx;
    voice_fx_init(&fx, RATE);
    ASSERT(voice_fx_set_feedback(&fx, 0.0f));
    ASSERT(voice_fx_set_drive(&fx, VOICE_FX_DRIVE_MIN));

    int16_t buf[64];
    memset(buf, 0, sizeof(buf));
    buf[0] = 16384;
    voice_fx_apply(&fx, buf, 64);
    ASSERT(buf[0] > 0);
    for (size_t i = 1; i < 64; i++) {
        ASSERT(buf[i] == 0); /* no echo at all */
    }
}

/* ========================================================================= */

int main(void)
{
    printf("=== voice_fx_core tests ===\n\n");

    test_run("the input trim keeps a loud signal off the rail",
             test_the_input_trim_keeps_a_loud_signal_off_the_rail);
    test_run("full scale never wraps sign", test_full_scale_never_wraps_sign);

    test_run("the echo lands exactly one delay later", test_the_echo_lands_exactly_one_delay_later);
    test_run("state survives a chunk boundary", test_state_survives_a_chunk_boundary);

    test_run("disabled is a byte-exact bypass", test_disabled_is_a_byte_exact_bypass);
    test_run("reset clears the tail", test_reset_clears_the_tail);

    test_run("out-of-range parameters are refused", test_out_of_range_parameters_are_refused);
    test_run("NaN parameters are refused", test_nan_parameters_are_refused);
    test_run("zero feedback is a dry path", test_zero_feedback_is_a_dry_path);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
