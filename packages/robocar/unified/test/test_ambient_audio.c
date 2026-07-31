/**
 * @file test_ambient_audio.c
 * @brief Host tests for the ambient-sound gate.
 *
 * The behaviour under test is a claim about a room — "the robot notices when
 * something happens that the camera cannot see, and stops noticing the fan" —
 * and the two ways it fails are opposites:
 *
 *   - too sensitive, and the robot remarks on the microphone's own AGC, on the
 *     air conditioning, and on its own motors (so the gain-invariance and
 *     steady-sound cases below are the load-bearing ones);
 *   - too blind, and a door slamming 200 microphone frames before the planner
 *     next asks is never heard at all (the latch cases).
 *
 * Nearly all of it is unstageable on a bench: you cannot hand a real room an
 * exact +12 dB gain change over a byte-identical soundscape, you cannot add a
 * controlled DC offset to it, and you certainly cannot run it up to the uint32
 * millisecond wrap at day 49. All three are one memset here.
 */

#include "ambient_audio.h"

#include <assert.h>
#include <math.h>
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
 * Synthetic PCM — the real frame geometry (1024 samples at 16 kHz = 64 ms)
 * ========================================================================= */

#define N AMBIENT_FRAME_SAMPLES
#define FRAME_MS 64u

static int16_t g_pcm[N];

static void silence(void)
{
    memset(g_pcm, 0, sizeof(g_pcm));
}

/** Add a sine at @p hz with amplitude @p amp. */
static void tone(float hz, float amp)
{
    for (int i = 0; i < N; ++i) {
        const float v =
            amp * sinf(2.0f * 3.14159265358979f * hz * (float)i / (float)AMBIENT_SAMPLE_RATE_HZ);
        g_pcm[i] = (int16_t)(g_pcm[i] + (int)v);
    }
}

/** Add deterministic pseudo-noise of +/- @p amp, so the upper bands are not
 *  sitting on the 1-LSB RMS floor where gain invariance would stop holding. */
static void noise(int amp)
{
    uint32_t s = 0x13579BDFu;
    for (int i = 0; i < N; ++i) {
        s = (s * 1664525u) + 1013904223u;
        const int v = (int)((s >> 16) % (uint32_t)(2 * amp + 1)) - amp;
        g_pcm[i] = (int16_t)(g_pcm[i] + v);
    }
}

/** Alternating +/- @p amp. Zero mean and RMS exactly @p amp, so the frame's
 *  level in dB is known in closed form — which is what makes the floor and
 *  rounding assertions below exact rather than approximate. */
static void alt(int amp)
{
    for (int i = 0; i < N; ++i) {
        g_pcm[i] = (int16_t)((i & 1) ? -amp : amp);
    }
}

static void scale(int k)
{
    for (int i = 0; i < N; ++i) {
        g_pcm[i] = (int16_t)(g_pcm[i] * k);
    }
}

static void offset(int d)
{
    for (int i = 0; i < N; ++i) {
        g_pcm[i] = (int16_t)(g_pcm[i] + d);
    }
}

/** A broadband frame whose eight bands all carry real energy. */
static void broadband(void)
{
    silence();
    tone(100.0f, 1000.0f);
    tone(700.0f, 1000.0f);
    tone(1500.0f, 1000.0f);
    tone(4000.0f, 1000.0f);
    tone(7000.0f, 1000.0f);
    noise(200);
}

static void fingerprint(ambient_fingerprint_t *out)
{
    ambient_fingerprint_from_pcm(g_pcm, N, out);
}

/** Fingerprint an alternating frame of amplitude @p amp (level = 20log10 amp). */
static void fp_alt(int amp, ambient_fingerprint_t *out)
{
    alt(amp);
    fingerprint(out);
}

/* Two levels exactly 20 dB apart and comfortably away from a .5 dB rounding
 * boundary: 20log10(316) = 49.99 dB, 20log10(3162) = 69.999 dB. */
#define QUIET_AMP 316
#define LOUD_AMP 3162

/* =========================================================================
 * Fingerprint construction
 * ========================================================================= */

static void test_uniform_gain_change_scores_exactly_zero(void)
{
    /* THE test. The microphone's own gain is not a fact about the room: an AGC
     * step, a different mic, or the robot driving closer to a wall all scale
     * every band by the same factor. In the log domain that is a constant added
     * to all eight bands, and it must cancel against their mean exactly — if it
     * does not, this gate tracks the microphone instead of the room. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;

    broadband();
    fingerprint(&quiet);
    ASSERT(quiet.valid);

    broadband();
    scale(4); /* +12.04 dB, every sample, no change of content whatsoever */
    fingerprint(&loud);
    ASSERT(loud.valid);

    ASSERT(loud.level_db > quiet.level_db);                   /* the LEVEL did change... */
    ASSERT(ambient_fingerprint_distance(&quiet, &loud) == 0); /* ...the SHAPE did not */
}

static void test_dc_offset_scores_exactly_zero(void)
{
    /* A PDM microphone carries a large subsonic term. Left in, it lands entirely
     * in band 0, swamps the other seven, and every frame fingerprints the same.
     * You cannot add a controlled DC offset to a real room. */
    ambient_fingerprint_t plain;
    ambient_fingerprint_t shifted;

    broadband();
    fingerprint(&plain);

    broadband();
    offset(2000);
    fingerprint(&shifted);

    ASSERT(ambient_fingerprint_distance(&plain, &shifted) == 0);
    ASSERT(plain.level_db == shifted.level_db);
}

static void test_digital_silence_is_valid(void)
{
    /* The audio equivalent of scene_change's flat grey frame, and the single
     * case most likely to be got wrong: log10f(0) is -inf, and an all-zero
     * fingerprint is byte-identical to a zeroed struct. Silence is a reading. */
    ambient_fingerprint_t fp;
    silence();
    fingerprint(&fp);

    ASSERT(fp.valid);
    ASSERT(fp.level_db == 0); /* floored at one LSB, not -inf, not NaN */
    for (int b = 0; b < AMBIENT_BANDS; ++b) {
        ASSERT(fp.band[b] == 0);
    }
}

static void test_degenerate_input_is_marked_invalid(void)
{
    ambient_fingerprint_t fp;

    ambient_fingerprint_from_pcm(NULL, N, &fp);
    ASSERT(!fp.valid);

    broadband();
    ambient_fingerprint_from_pcm(g_pcm, 0, &fp);
    ASSERT(!fp.valid);
    ambient_fingerprint_from_pcm(g_pcm, AMBIENT_MIN_SAMPLES - 1, &fp);
    ASSERT(!fp.valid);
    ambient_fingerprint_from_pcm(g_pcm, N, NULL); /* must not crash */

    /* "Cannot tell" must read as "no change", never as a new soundscape. */
    ambient_fingerprint_t good;
    ambient_fingerprint_t bad;
    fingerprint(&good);
    memset(&bad, 0, sizeof(bad));
    ASSERT(ambient_fingerprint_distance(&good, &bad) == 0);
    ASSERT(ambient_fingerprint_distance(&bad, &good) == 0);
    ASSERT(ambient_fingerprint_distance(NULL, &good) == 0);
    ASSERT(ambient_fingerprint_distance(&good, NULL) == 0);
}

static void test_rounding_is_to_nearest(void)
{
    /* 20*log10(3020) = 69.600 dB. Truncation gives 69; rounding to nearest gives
     * 70. Truncating biases every band level down by half a dB on average, and
     * the bias partly cancels when the mean is subtracted — which is exactly
     * what would let it hide. */
    ambient_fingerprint_t fp;
    fp_alt(3020, &fp);
    ASSERT(fp.valid);
    ASSERT(fp.level_db == 70);
}

static void test_changed_shape_at_identical_level(void)
{
    /* "Empty room -> someone talking", with the loudness confound removed. If
     * the two sub-measures were really one measure counted twice, this would
     * score nothing. */
    ambient_fingerprint_t low;
    ambient_fingerprint_t high;

    silence();
    tone(500.0f, 4000.0f);
    fingerprint(&low);

    silence();
    tone(2000.0f, 4000.0f);
    fingerprint(&high);

    ASSERT(low.level_db == high.level_db); /* same RMS, so no level excursion */
    ASSERT(ambient_fingerprint_distance(&low, &high) >= AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT);

    ambient_audio_init();
    ambient_audio_note(&low, 0u);
    ambient_audio_mark_spoken();
    ambient_audio_note(&high, FRAME_MS);

    ASSERT(ambient_audio_loud_score() < AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_shape_score() >= AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_novel(FRAME_MS));
}

/* =========================================================================
 * The floor and the latch
 * ========================================================================= */

static void test_a_steady_sound_is_absorbed_by_the_floor(void)
{
    /* The fan switches on. It is genuinely news for a few seconds, and then it
     * is furniture — and it must stop being news WITHOUT a separate "is this
     * steady?" test, because the floor rising through it is the mechanism.
     * Staging this on a bench needs a fan on a timer. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;
    fp_alt(QUIET_AMP, &quiet);
    fp_alt(LOUD_AMP, &loud);

    ambient_audio_init();
    uint32_t t = 0u;
    for (int i = 0; i < 50; ++i, t += FRAME_MS) {
        ambient_audio_note(&quiet, t);
    }
    ASSERT(ambient_audio_loud_score() == 0u);

    const uint32_t onset = t;
    ambient_audio_note(&loud, t);
    t += FRAME_MS;
    ASSERT(ambient_audio_loud_score() >= AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);

    uint32_t absorbed_at = 0u;
    for (int i = 0; i < 400; ++i, t += FRAME_MS) {
        ambient_audio_note(&loud, t);
        if (absorbed_at == 0u && ambient_audio_loud_score() < AMBIENT_LOUD_THRESHOLD_DB_DEFAULT) {
            absorbed_at = t;
        }
    }

    ASSERT(absorbed_at != 0u);
    /* A 20 dB excursion against a 1.0 dB/s rise: gone inside ~20 s, and it must
     * not be instant either or a real transient would never be heard. */
    ASSERT((absorbed_at - onset) <= 21000u);
    ASSERT((absorbed_at - onset) >= 2000u);
    ASSERT(ambient_audio_loud_score() < AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_floor_db() > (int16_t)50); /* the floor climbed to meet it */
}

static void test_a_single_transient_is_not_absorbed(void)
{
    /* The case the whole latch design exists for. The microphone runs ~200x
     * faster than the planner, so a door slam has to still be there when the
     * planner finally asks — an instantaneous-reading implementation passes
     * every other test here and silently fails this one. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;
    fp_alt(QUIET_AMP, &quiet);
    fp_alt(LOUD_AMP, &loud);

    ambient_audio_init();
    uint32_t t = 0u;
    for (int i = 0; i < 100; ++i, t += FRAME_MS) {
        ambient_audio_note(&quiet, t);
    }
    ASSERT(ambient_audio_loud_score() == 0u);

    ambient_audio_note(&loud, t);
    t += FRAME_MS;
    const unsigned peak = ambient_audio_loud_score();
    ASSERT(peak >= AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);

    for (int i = 0; i < 100; ++i, t += FRAME_MS) { /* 6.4 s of quiet room */
        ambient_audio_note(&quiet, t);
    }
    /* Still latched, and NOT inflated by the floor falling back either. */
    ASSERT(ambient_audio_loud_score() >= AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_loud_score() <= peak);
}

static void test_the_latch_expires(void)
{
    /* Otherwise a bang the budget suppressed ten minutes ago still licenses a
     * remark, and the robot reacts to things nobody remembers happening. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;
    fp_alt(QUIET_AMP, &quiet);
    fp_alt(LOUD_AMP, &loud);

    ambient_audio_init();
    uint32_t t = 0u;
    for (int i = 0; i < 20; ++i, t += FRAME_MS) {
        ambient_audio_note(&quiet, t);
    }
    ambient_audio_mark_spoken();

    const uint32_t bang = t;
    ambient_audio_note(&loud, bang);
    ASSERT(ambient_audio_novel(bang));
    ASSERT(ambient_audio_novel(bang + AMBIENT_LATCH_TTL_MS_DEFAULT - 1u));
    ASSERT(!ambient_audio_novel(bang + AMBIENT_LATCH_TTL_MS_DEFAULT));
}

static void test_uint32_millisecond_wrap_at_day_49(void)
{
    /* Getting the sign wrong here either mutes the robot for 49 days or arms it
     * forever, and neither is reachable on a bench. Every elapsed comparison
     * must be an unsigned difference, never `now > stamp + ttl`. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;
    fp_alt(QUIET_AMP, &quiet);
    fp_alt(LOUD_AMP, &loud);

    ambient_audio_init();
    uint32_t t = 0xFFFF0000u;
    for (int i = 0; i < 20; ++i, t += FRAME_MS) {
        ambient_audio_note(&quiet, t);
    }
    ambient_audio_mark_spoken();

    const uint32_t bang = 0xFFFFF000u;
    ambient_audio_note(&loud, bang);

    /* 8192 ms later, on the far side of the wrap. */
    ASSERT(ambient_audio_novel(0x00001000u));
    /* And it must still expire on schedule across the wrap, not 49 days late. */
    ASSERT(!ambient_audio_novel(bang + AMBIENT_LATCH_TTL_MS_DEFAULT));

    /* The floor tracker's own elapsed maths must survive the wrap too: a frame
     * arriving just after it must not credit the floor with a 49-day rise. */
    ambient_audio_note(&quiet, 0x00000040u);
    ASSERT(ambient_audio_floor_db() <= (int16_t)51);
}

static void test_an_invalid_frame_moves_nothing(void)
{
    /* A run of failed reads must not read as a brand-new soundscape — the same
     * trap scene_change_note() closes for undecodable camera frames. */
    ambient_fingerprint_t quiet;
    ambient_fingerprint_t loud;
    fp_alt(QUIET_AMP, &quiet);
    fp_alt(LOUD_AMP, &loud);

    ambient_audio_init();
    uint32_t t = 0u;
    for (int i = 0; i < 50; ++i, t += FRAME_MS) {
        ambient_audio_note(&quiet, t);
    }
    ambient_audio_note(&loud, t);
    t += FRAME_MS;

    const int16_t floor_before = ambient_audio_floor_db();
    const unsigned loud_before = ambient_audio_loud_score();
    const unsigned shape_before = ambient_audio_shape_score();

    ambient_fingerprint_t bad;
    memset(&bad, 0, sizeof(bad)); /* valid == false */
    for (int i = 0; i < 50; ++i, t += FRAME_MS) {
        ambient_audio_note(&bad, t);
    }
    ambient_audio_note(NULL, t);

    ASSERT(ambient_audio_floor_db() == floor_before);
    ASSERT(ambient_audio_loud_score() == loud_before);
    ASSERT(ambient_audio_shape_score() == shape_before);
}

/* =========================================================================
 * The gate
 * ========================================================================= */

static void test_first_observation_is_novel(void)
{
    /* Nothing has been responded to yet, and the robot's first impression of a
     * room genuinely is new. Getting this wrong mutes the boot greeting. */
    ambient_fingerprint_t quiet;
    fp_alt(QUIET_AMP, &quiet);

    ambient_audio_init();
    ASSERT(ambient_audio_novel(0u));
    ambient_audio_note(&quiet, 0u);
    ASSERT(ambient_audio_novel(FRAME_MS));
}

/** Reference on a quiet 500 Hz room, then one frame that is both louder and a
 *  different shape — so each sub-gate can be disabled independently. */
static void arm_both_subgates(void)
{
    ambient_fingerprint_t before;
    ambient_fingerprint_t after;

    silence();
    tone(500.0f, 300.0f);
    fingerprint(&before);

    silence();
    tone(2000.0f, 8000.0f);
    fingerprint(&after);

    ambient_audio_init();
    ambient_audio_note(&before, 0u);
    ambient_audio_mark_spoken();
    ambient_audio_note(&after, FRAME_MS);
}

static void test_threshold_zero_disables_each_subgate(void)
{
    /* THE polarity test. scene_change is AND-ed, so its 0 means "always novel";
     * these two are OR-ed, so their 0 must mean "never contributes". Same
     * intent, opposite constant — the neutral element follows the operator, and
     * getting it backwards makes the robot speak on every permitted cycle and
     * look like a tuning problem. */
    arm_both_subgates();
    ASSERT(ambient_audio_loud_score() >= AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_shape_score() >= AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT);
    ASSERT(ambient_audio_novel(FRAME_MS));

    ambient_audio_set_loud_threshold(0);
    ASSERT(ambient_audio_loud_threshold() == 0);
    ASSERT(ambient_audio_novel(FRAME_MS)); /* shape alone still carries it */

    ambient_audio_set_loud_threshold(AMBIENT_LOUD_THRESHOLD_DB_DEFAULT);
    ambient_audio_set_shape_threshold(0);
    ASSERT(ambient_audio_novel(FRAME_MS)); /* loudness alone still carries it */

    /* And with BOTH disabled the whole term drops out of the OR: may_speak
     * reduces exactly to the pre-microphone `budget && scene`. That reduction is
     * the slice's safety property — one console command puts the robot back to
     * behaviour that has already been lived with. */
    ambient_audio_set_loud_threshold(0);
    ASSERT(!ambient_audio_novel(FRAME_MS));
    ASSERT(!ambient_audio_novel(0u));
    ASSERT(!ambient_audio_novel(0xFFFFFFFFu));

    ambient_audio_init();
    ambient_audio_set_loud_threshold(0);
    ambient_audio_set_shape_threshold(0);
    ASSERT(!ambient_audio_novel(0u)); /* not even the first observation */
}

static void test_threshold_equal_to_the_score_counts_as_novel(void)
{
    /* >=, not >. Mirrors test_scene_change's boundary case. */
    arm_both_subgates();

    const unsigned loud = ambient_audio_loud_score();
    const unsigned shape = ambient_audio_shape_score();
    ASSERT(loud > 0u && loud < 255u);
    ASSERT(shape > 0u && shape < 255u);

    ambient_audio_set_shape_threshold(0); /* isolate the loudness branch */
    ambient_audio_set_loud_threshold((uint8_t)loud);
    ASSERT(ambient_audio_novel(FRAME_MS));
    ambient_audio_set_loud_threshold((uint8_t)(loud + 1u));
    ASSERT(!ambient_audio_novel(FRAME_MS));

    ambient_audio_set_loud_threshold(0); /* isolate the shape branch */
    ambient_audio_set_shape_threshold((uint8_t)shape);
    ASSERT(ambient_audio_novel(FRAME_MS));
    ambient_audio_set_shape_threshold((uint8_t)(shape + 1u));
    ASSERT(!ambient_audio_novel(FRAME_MS));
}

static void test_mark_spoken_clears_the_latches(void)
{
    /* Once remarked on, the same bang must not license a second remark on the
     * next planner cycle. */
    arm_both_subgates();
    ASSERT(ambient_audio_novel(FRAME_MS));

    ambient_audio_mark_spoken();
    ASSERT(!ambient_audio_novel(FRAME_MS));
    ASSERT(ambient_audio_loud_score() == 0u);
    ASSERT(ambient_audio_shape_score() == 0u);
}

/* =========================================================================
 * Capture policy
 * ========================================================================= */

static void test_capture_allowed_honours_the_playback_hangover(void)
{
    /* The microphone hears the MAX98357A perfectly well, and the room keeps
     * ringing after the amplifier stops. Without the hangover the robot hears
     * its own last word, latches it, and remarks on it one cycle later. */
    const uint32_t hang = AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT;

    ASSERT(!ambient_capture_allowed(true, 10000u, 0u, hang));
    ASSERT(!ambient_capture_allowed(true, 10000u, 9000u, hang));

    ASSERT(!ambient_capture_allowed(false, 10000u, 10000u, hang));             /* just stopped */
    ASSERT(!ambient_capture_allowed(false, 10000u + hang - 1u, 10000u, hang)); /* still ringing */
    ASSERT(ambient_capture_allowed(false, 10000u + hang, 10000u, hang));       /* clear */

    ASSERT(ambient_capture_allowed(false, 10000u, 10000u, 0u)); /* hangover disabled */

    /* Across the uint32 wrap. `now > end + hangover` overflows here and deafens
     * the robot for the rest of the 49-day cycle. */
    const uint32_t end = 0xFFFFFF00u;
    ASSERT(!ambient_capture_allowed(false, end + hang - 1u, end, hang));
    ASSERT(ambient_capture_allowed(false, end + hang, end, hang));
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== ambient_audio host tests ===\n\n");

    test_run("a uniform gain change scores exactly zero on shape",
             test_uniform_gain_change_scores_exactly_zero);
    test_run("a DC offset scores exactly zero on shape", test_dc_offset_scores_exactly_zero);
    test_run("digital silence is valid, not invalid", test_digital_silence_is_valid);
    test_run("degenerate input is marked invalid", test_degenerate_input_is_marked_invalid);
    test_run("dB rounding is to nearest, not truncation", test_rounding_is_to_nearest);
    test_run("a changed shape at an identical level fires the shape gate",
             test_changed_shape_at_identical_level);

    test_run("a steady sound is absorbed by the floor",
             test_a_steady_sound_is_absorbed_by_the_floor);
    test_run("a single transient is not absorbed, and stays latched",
             test_a_single_transient_is_not_absorbed);
    test_run("the latch expires after the TTL", test_the_latch_expires);
    test_run("the uint32 millisecond wrap at day 49", test_uint32_millisecond_wrap_at_day_49);
    test_run("an invalid frame moves neither the floor nor the latch",
             test_an_invalid_frame_moves_nothing);

    test_run("the first observation is novel", test_first_observation_is_novel);
    test_run("threshold zero disables each sub-gate (inverted polarity)",
             test_threshold_zero_disables_each_subgate);
    test_run("a threshold equal to the score counts as novel",
             test_threshold_equal_to_the_score_counts_as_novel);
    test_run("mark_spoken clears both latches", test_mark_spoken_clears_the_latches);

    test_run("capture_allowed honours the playback hangover, including the wrap",
             test_capture_allowed_honours_the_playback_hangover);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
