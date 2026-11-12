/**
 * @file test_speech_trigger.c
 * @brief Host tests for the speech-shaped VAD trigger (issue #617).
 *
 * The trigger must start a voice turn for a voice and not for a door slam, a
 * dropped object or a burst of broadband noise. None of those is reproducible on
 * demand on a bench, and a false positive there costs a beep and a request, so
 * each is synthesised here: a voiced signal (harmonics of a 140 Hz fundamental
 * under a three-formant envelope), a single-sample click, white noise, and an
 * in-band tone shorter and longer than the sustain time. Frames are 1024 samples
 * at 16 kHz (64 ms), as on the device.
 */

#include "speech_trigger.h"

#include <assert.h>
#include <math.h>
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

#define FRAME 1024
#define FRAME_MS 64u
#define RATE 16000.0
#define PI 3.14159265358979

/* Levels passed to note(): the module takes them from the caller (the listener
 * computes them with ambient_fingerprint_from_pcm), so the tests state them. */
#define FLOOR_DB 30
#define LOUD_DB (FLOOR_DB + 25)
#define QUIET_DB (FLOOR_DB + 3)

/** Voiced speech stand-in: harmonics of f0 up to 4 kHz, each weighted by a
 *  three-formant envelope (500, 1500, 2500 Hz). @p offset keeps phase continuous
 *  across frames. A DC term is added because the real PDM mic carries one. */
static void voiced(int16_t *f, size_t n, size_t offset, double amp)
{
    const double f0 = 140.0;
    const double formants[3] = {500.0, 1500.0, 2500.0};
    const double bw = 200.0;
    const int harmonics = (int)(4000.0 / f0); /* 28: up to 3920 Hz */
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)(offset + i) / RATE;
        double s = 0.0;
        for (int h = 1; h <= harmonics; ++h) {
            const double fh = f0 * h;
            double g = 0.02;
            for (int k = 0; k < 3; ++k) {
                const double d = (fh - formants[k]) / bw;
                g += exp(-0.5 * d * d) / (double)(k + 1);
            }
            s += g * sin(2.0 * PI * fh * t);
        }
        f[i] = (int16_t)(400.0 + amp * s);
    }
}

static void tone(int16_t *f, size_t n, size_t offset, double hz, double amp)
{
    for (size_t i = 0; i < n; ++i) {
        f[i] = (int16_t)(amp * sin(2.0 * PI * hz * (double)(offset + i) / RATE));
    }
}

/** Deterministic white noise (xorshift32), roughly uniform. */
static uint32_t s_rng = 2463534242u;
static void noise(int16_t *f, size_t n, int amp)
{
    for (size_t i = 0; i < n; ++i) {
        s_rng ^= s_rng << 13;
        s_rng ^= s_rng >> 17;
        s_rng ^= s_rng << 5;
        f[i] = (int16_t)((int)(s_rng % (uint32_t)(2 * amp + 1)) - amp);
    }
}

static void test_voiced_speech_is_mostly_in_band(void)
{
    int16_t f[FRAME];
    voiced(f, FRAME, 0, 900.0);
    const uint8_t share = speech_trigger_band_share_pct(f, FRAME);
    printf("     voiced share = %u%%\n", (unsigned)share);
    ASSERT(share >= 80u);
}

static void test_white_noise_and_a_click_are_not(void)
{
    int16_t f[FRAME];
    noise(f, FRAME, 8000);
    const uint8_t noise_share = speech_trigger_band_share_pct(f, FRAME);

    memset(f, 0, sizeof(f));
    f[FRAME / 2] = 30000;
    const uint8_t click_share = speech_trigger_band_share_pct(f, FRAME);

    printf("     noise share = %u%%, click share = %u%%\n", (unsigned)noise_share,
           (unsigned)click_share);
    ASSERT(noise_share < SPEECH_TRIGGER_SHARE_PCT_DEFAULT - 10u);
    ASSERT(click_share < SPEECH_TRIGGER_SHARE_PCT_DEFAULT - 10u);
}

static void test_low_rumble_is_not(void)
{
    int16_t f[FRAME];
    tone(f, FRAME, 0, 90.0, 10000.0); /* motor / mains rumble */
    ASSERT(speech_trigger_band_share_pct(f, FRAME) < 30u);
}

static void test_silence_and_degenerate_input_score_zero(void)
{
    int16_t f[FRAME];
    memset(f, 0, sizeof(f));
    ASSERT(speech_trigger_band_share_pct(f, FRAME) == 0u);
    for (size_t i = 0; i < FRAME; ++i) {
        f[i] = 1234; /* pure DC */
    }
    ASSERT(speech_trigger_band_share_pct(f, FRAME) == 0u);
    ASSERT(speech_trigger_band_share_pct(NULL, FRAME) == 0u);
    ASSERT(speech_trigger_band_share_pct(f, 0) == 0u);
}

/** Feed @p frames frames from @p gen and return the frame index at which the
 *  trigger first fired, or -1. */
typedef void (*frame_gen_t)(int16_t *f, size_t k);

static int run_frames(frame_gen_t gen, int frames, int16_t level_db, uint32_t t0)
{
    int16_t f[FRAME];
    for (int k = 0; k < frames; ++k) {
        gen(f, (size_t)k);
        if (speech_trigger_note(f, FRAME, level_db, FLOOR_DB, t0 + (uint32_t)k * FRAME_MS)) {
            return k;
        }
    }
    return -1;
}

static void gen_voiced(int16_t *f, size_t k)
{
    voiced(f, FRAME, k * FRAME, 900.0);
}

static void gen_noise(int16_t *f, size_t k)
{
    (void)k;
    noise(f, FRAME, 8000);
}

static void gen_click(int16_t *f, size_t k)
{
    memset(f, 0, FRAME * sizeof(int16_t));
    if (k == 0) {
        f[100] = 32000;
    }
}

static void test_sustained_voiced_speech_triggers(void)
{
    speech_trigger_init();
    /* 150 ms needs three 64 ms frames: fires on the third (index 2). */
    ASSERT(run_frames(gen_voiced, 10, LOUD_DB, 1000u) == 2);
    ASSERT(speech_trigger_run_ms() >= SPEECH_TRIGGER_SUSTAIN_MS_DEFAULT);
}

static void test_a_click_does_not_trigger(void)
{
    /* A loud click, then the quiet room: one frame above the floor. Rejected on
     * shape and on duration alike — the noise-burst test below is the one that
     * isolates the shape check. */
    speech_trigger_init();
    int16_t f[FRAME];
    for (int k = 0; k < 6; ++k) {
        gen_click(f, (size_t)k);
        const int16_t level = (k == 0) ? LOUD_DB : FLOOR_DB;
        ASSERT(!speech_trigger_note(f, FRAME, level, FLOOR_DB, 1000u + (uint32_t)k * FRAME_MS));
    }
}

static void test_a_broadband_noise_burst_does_not_trigger(void)
{
    /* Loud and sustained — half a second — and still not a voice. */
    speech_trigger_init();
    ASSERT(run_frames(gen_noise, 8, LOUD_DB, 1000u) == -1);
}

static void test_voiced_speech_below_the_floor_margin_does_not_trigger(void)
{
    speech_trigger_init();
    ASSERT(run_frames(gen_voiced, 10, QUIET_DB, 1000u) == -1);
}

static void test_an_in_band_tone_shorter_than_the_sustain_does_not_trigger(void)
{
    /* 100 ms of 1 kHz then silence: the tone covers frame 0 and part of frame 1,
     * so the run reaches 128 ms and is broken by the silent frame after it. */
    speech_trigger_init();
    int16_t f[FRAME];
    const size_t tone_samples = 1600; /* 100 ms */

    tone(f, FRAME, 0, 1000.0, 8000.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 1000u));

    tone(f, tone_samples - FRAME, FRAME, 1000.0, 8000.0);
    memset(&f[tone_samples - FRAME], 0, (2 * FRAME - tone_samples) * sizeof(int16_t));
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 1064u));

    memset(f, 0, sizeof(f));
    ASSERT(!speech_trigger_note(f, FRAME, FLOOR_DB, FLOOR_DB, 1128u));
    ASSERT(speech_trigger_run_ms() == 0u);
}

static void test_a_sustained_in_band_tone_does_trigger(void)
{
    /* Pinned so nobody reads the duration as a tonality check: a whistle that
     * outlasts the sustain time is speech-shaped by this rule. */
    speech_trigger_init();
    int16_t f[FRAME];
    bool fired = false;
    for (int k = 0; k < 4 && !fired; ++k) {
        tone(f, FRAME, (size_t)k * FRAME, 1000.0, 8000.0);
        fired = speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 1000u + (uint32_t)k * FRAME_MS);
    }
    ASSERT(fired);
}

static void test_one_non_speech_frame_breaks_the_run(void)
{
    speech_trigger_init();
    int16_t f[FRAME];
    voiced(f, FRAME, 0, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 0u));
    voiced(f, FRAME, FRAME, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 64u));
    noise(f, FRAME, 8000);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 128u));
    voiced(f, FRAME, 3 * FRAME, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 192u)); /* starts over */
    ASSERT(speech_trigger_run_ms() == FRAME_MS);
}

static void test_reset_run_requires_fresh_speech(void)
{
    speech_trigger_init();
    ASSERT(run_frames(gen_voiced, 3, LOUD_DB, 0u) == 2);
    speech_trigger_reset_run();
    ASSERT(speech_trigger_run_ms() == 0u);
    int16_t f[FRAME];
    voiced(f, FRAME, 0, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, 300u));
}

static void test_a_gap_breaks_the_run_across_the_wrap(void)
{
    /* A run straddling the uint32 millisecond wrap keeps accumulating when the
     * frames are contiguous, and a gap longer than SPEECH_TRIGGER_MAX_GAP_MS
     * breaks it — on both sides of the wrap. */
    speech_trigger_init();
    int16_t f[FRAME];
    const uint32_t t0 = UINT32_MAX - 70u;

    voiced(f, FRAME, 0, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0));
    voiced(f, FRAME, FRAME, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0 + 64u));
    voiced(f, FRAME, 2 * FRAME, 900.0);
    ASSERT((uint32_t)(t0 + 128u) < t0); /* it really did wrap */
    ASSERT(speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0 + 128u));

    /* The listener was locked out for a second: speech before it is not now. */
    speech_trigger_init();
    voiced(f, FRAME, 0, 900.0);
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0));
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0 + 64u));
    ASSERT(!speech_trigger_note(f, FRAME, LOUD_DB, FLOOR_DB, t0 + 64u + 1000u));
    ASSERT(speech_trigger_run_ms() == FRAME_MS);
}

static void test_zero_thresholds_drop_their_check(void)
{
    speech_trigger_init();
    speech_trigger_set_share_pct(0u);
    ASSERT(run_frames(gen_noise, 4, LOUD_DB, 0u) == 2); /* loudness alone */

    speech_trigger_init();
    speech_trigger_set_margin_db(0u);
    ASSERT(run_frames(gen_voiced, 4, QUIET_DB, 0u) == 2); /* shape alone */
}

static void test_knobs_round_trip_and_defaults(void)
{
    speech_trigger_init();
    ASSERT(speech_trigger_share_pct() == SPEECH_TRIGGER_SHARE_PCT_DEFAULT);
    ASSERT(speech_trigger_margin_db() == SPEECH_TRIGGER_MARGIN_DB_DEFAULT);
    ASSERT(speech_trigger_sustain_ms() == SPEECH_TRIGGER_SUSTAIN_MS_DEFAULT);

    speech_trigger_set_share_pct(150u);
    ASSERT(speech_trigger_share_pct() == 100u); /* a share is a percentage */
    speech_trigger_set_share_pct(70u);
    ASSERT(speech_trigger_share_pct() == 70u);
    speech_trigger_set_margin_db(12u);
    speech_trigger_set_sustain_ms(300u);
    ASSERT(speech_trigger_margin_db() == 12u);
    ASSERT(speech_trigger_sustain_ms() == 300u);

    /* 300 ms needs five frames now. */
    ASSERT(run_frames(gen_voiced, 10, LOUD_DB, 0u) == 4);
}

static void test_best_reports_a_near_miss(void)
{
    /* Tuning aid: with the share threshold set above what the voice reaches, no
     * run forms, but the peak share still says how close it came. */
    speech_trigger_init();
    speech_trigger_set_share_pct(100u);
    ASSERT(run_frames(gen_voiced, 5, LOUD_DB, 0u) == -1);

    uint32_t run = 99u;
    uint8_t peak = 0u;
    speech_trigger_best(&run, &peak, true);
    ASSERT(run == 0u);
    ASSERT(peak >= 80u && peak < 100u);

    speech_trigger_best(&run, &peak, false); /* cleared by the previous call */
    ASSERT(run == 0u && peak == 0u);

    speech_trigger_set_share_pct(SPEECH_TRIGGER_SHARE_PCT_DEFAULT);
    ASSERT(run_frames(gen_voiced, 5, LOUD_DB, 1000u) == 2);
    speech_trigger_best(&run, NULL, false);
    ASSERT(run == 3u * FRAME_MS);
}

int main(void)
{
    printf("=== speech_trigger host tests ===\n\n");

    test_run("voiced speech is mostly in band", test_voiced_speech_is_mostly_in_band);
    test_run("white noise and a click are not", test_white_noise_and_a_click_are_not);
    test_run("low rumble is not", test_low_rumble_is_not);
    test_run("silence and degenerate input score zero",
             test_silence_and_degenerate_input_score_zero);
    test_run("sustained voiced speech triggers", test_sustained_voiced_speech_triggers);
    test_run("a click does not trigger", test_a_click_does_not_trigger);
    test_run("a broadband noise burst does not trigger",
             test_a_broadband_noise_burst_does_not_trigger);
    test_run("voiced speech below the floor margin does not trigger",
             test_voiced_speech_below_the_floor_margin_does_not_trigger);
    test_run("an in-band tone shorter than the sustain does not trigger",
             test_an_in_band_tone_shorter_than_the_sustain_does_not_trigger);
    test_run("a sustained in-band tone does trigger", test_a_sustained_in_band_tone_does_trigger);
    test_run("one non-speech frame breaks the run", test_one_non_speech_frame_breaks_the_run);
    test_run("reset_run requires fresh speech", test_reset_run_requires_fresh_speech);
    test_run("a gap breaks the run, across the wrap", test_a_gap_breaks_the_run_across_the_wrap);
    test_run("zero thresholds drop their check", test_zero_thresholds_drop_their_check);
    test_run("knobs round-trip, and defaults", test_knobs_round_trip_and_defaults);
    test_run("best reports a near miss", test_best_reports_a_near_miss);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
