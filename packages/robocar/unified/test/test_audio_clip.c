/**
 * @file test_audio_clip.c
 * @brief Host tests for clip sizing, WAV framing and normalisation.
 *
 * Two of these pin failures that are effectively impossible to find any other
 * way:
 *
 *   - The **WAV header**, byte for byte. Gemini rejects raw PCM with an HTTP 400
 *     that names no field, so a wrong field width or byte order here does not
 *     fail on the device at all — it fails at a remote server, opaquely, after a
 *     ~340 kB upload. Asserting the exact 44 bytes converts that into a local
 *     failure with an obvious cause.
 *
 *   - The **size arithmetic at the overflow boundary**. A wrapped product is an
 *     ordinary-looking small number, so the bug is a short allocation that is
 *     then overrun — and no console input a person would ever type produces it.
 *     Only calling the function directly does.
 *
 * The normalisation tests are cheaper but still worth having: `peak` and
 * `clipped` are what tell a mis-set microphone gain from a bad model reply, so a
 * wrong count would misdirect exactly the debugging session they exist to serve.
 */

#include "audio_clip.h"

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
    fn();
    test_pass++;
    printf("  ok: %s\n", name);
}

/* =========================================================================
 * Sizing
 * ========================================================================= */

static void test_nominal_window_sizes_correctly(void)
{
    /* 4 s at 16 kHz mono 16-bit = 64000 samples = 128000 bytes. */
    ASSERT(audio_clip_pcm_bytes(4000u, 16000u) == 128000u);
    ASSERT(audio_clip_pcm_bytes(1000u, 16000u) == 32000u);
}

static void test_zero_parameters_refuse(void)
{
    ASSERT(audio_clip_pcm_bytes(0u, 16000u) == 0u);
    ASSERT(audio_clip_pcm_bytes(4000u, 0u) == 0u);
}

static void test_the_cap_is_enforced_exactly(void)
{
    /* AUDIO_CLIP_MAX_BYTES is 256000 = exactly 8 s. 8 s must pass and the next
     * millisecond must refuse — an off-by-one here either rejects the documented
     * maximum window or admits one past the memory budget. */
    ASSERT(audio_clip_pcm_bytes(8000u, 16000u) == 256000u);
    ASSERT(audio_clip_pcm_bytes(8001u, 16000u) == 0u);
}

static void test_overflow_refuses_rather_than_wraps(void)
{
    /* THE case a bench cannot produce. window_ms * sample_rate_hz overflows 32
     * bits comfortably here; the function must return 0 (refuse) and never a
     * small wrapped value that would be used as an allocation size. */
    ASSERT(audio_clip_pcm_bytes(0xFFFFFFFFu, 16000u) == 0u);
    ASSERT(audio_clip_pcm_bytes(0xFFFFFFFFu, 0xFFFFFFFFu) == 0u);
    ASSERT(audio_clip_pcm_bytes(1000000u, 48000u) == 0u);
}

static void test_b64_length_includes_the_header(void)
{
    /* 128000 PCM + 44 header = 128044 bytes -> ceil(128044/3)*4 = 170728, +1 NUL.
     * Computed independently here rather than by calling the same expression, so
     * the test would catch the header being dropped from the calculation. */
    const size_t pcm = 128000u;
    const size_t total = pcm + 44u;
    const size_t expect = ((total + 2u) / 3u) * 4u + 1u;
    ASSERT(audio_clip_b64_length(pcm) == expect);
    /* And it must genuinely differ from the header-less answer, or the test
     * above would pass with the header omitted. */
    const size_t without = ((pcm + 2u) / 3u) * 4u + 1u;
    ASSERT(audio_clip_b64_length(pcm) > without);
}

static void test_b64_length_refuses_out_of_range(void)
{
    ASSERT(audio_clip_b64_length(0u) == 0u);
    ASSERT(audio_clip_b64_length(AUDIO_CLIP_MAX_BYTES + 1u) == 0u);
}

/* =========================================================================
 * WAV header — asserted byte for byte
 * ========================================================================= */

static void test_wav_header_is_byte_exact(void)
{
    uint8_t h[AUDIO_CLIP_WAV_HEADER_BYTES];
    memset(h, 0xAA, sizeof(h));
    ASSERT(audio_clip_wav_header(h, 128000u, 16000u, 1u, 16u));

    /* Hand-computed for 16 kHz / mono / 16-bit / 128000 payload:
     *   RIFF size  = 36 + 128000 = 128036 = 0x0001F424
     *   byte rate  = 16000 * 2   = 32000  = 0x00007D00
     *   block align= 2, bits = 16, channels = 1, fmt = 1 (PCM)
     *   data size  = 128000               = 0x0001F400 */
    static const uint8_t expect[AUDIO_CLIP_WAV_HEADER_BYTES] = {
        'R',  'I',  'F',  'F',  0x24, 0xF4, 0x01, 0x00, 'W',  'A',  'V',  'E',  'f',  'm',  't',
        ' ',  0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x7D,
        0x00, 0x00, 0x02, 0x00, 0x10, 0x00, 'd',  'a',  't',  'a',  0x00, 0xF4, 0x01, 0x00,
    };
    ASSERT(memcmp(h, expect, sizeof(expect)) == 0);
}

static void test_wav_header_refuses_degenerate_input(void)
{
    uint8_t h[AUDIO_CLIP_WAV_HEADER_BYTES];
    ASSERT(!audio_clip_wav_header(NULL, 1000u, 16000u, 1u, 16u));
    ASSERT(!audio_clip_wav_header(h, 0u, 16000u, 1u, 16u));
    ASSERT(!audio_clip_wav_header(h, 1000u, 0u, 1u, 16u));
    ASSERT(!audio_clip_wav_header(h, 1000u, 16000u, 0u, 16u));
    ASSERT(!audio_clip_wav_header(h, 1000u, 16000u, 1u, 12u)); /* not a byte multiple */
}

/* =========================================================================
 * Normalisation
 * ========================================================================= */

static void test_dc_is_removed_and_reported(void)
{
    int16_t pcm[8];
    for (size_t i = 0; i < 8; ++i) {
        pcm[i] = 1000; /* pure DC, no signal at all */
    }
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 8, &st);

    ASSERT(st.dc == 1000);
    ASSERT(st.peak == 0); /* after removal there is nothing left */
    ASSERT(st.samples == 8u);
    for (size_t i = 0; i < 8; ++i) {
        ASSERT(pcm[i] == 0);
    }
}

static void test_peak_survives_dc_removal(void)
{
    int16_t pcm[4] = {100, -100, 300, -300};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 4, &st);
    ASSERT(st.dc == 0);
    ASSERT(st.raw_peak == 300);
    ASSERT(st.peak == (int16_t)AUDIO_CLIP_TARGET_PEAK);
    ASSERT(st.gain_q8 == 80u * 256u);
    ASSERT(st.clipped == 0u);
    ASSERT(pcm[0] == 8000);
    ASSERT(pcm[1] == -8000);
    ASSERT(pcm[2] == (int16_t)AUDIO_CLIP_TARGET_PEAK);
    ASSERT(pcm[3] == -(int16_t)AUDIO_CLIP_TARGET_PEAK);
}

static void test_faint_audio_amplified_to_target(void)
{
    /* Faint conversational audio (peak 200, zero DC offset).
     * Needs 120x gain to reach AUDIO_CLIP_TARGET_PEAK (24000). */
    int16_t pcm[4] = {50, -100, 200, -150};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 4, &st);

    ASSERT(st.dc == 0);
    ASSERT(st.raw_peak == 200);
    ASSERT(st.gain_q8 == 120u * 256u);
    ASSERT(st.peak == (int16_t)AUDIO_CLIP_TARGET_PEAK);
    ASSERT(st.clipped == 0u);
    ASSERT(pcm[0] == 6000);                            /* 50 * 120 */
    ASSERT(pcm[1] == -12000);                          /* -100 * 120 */
    ASSERT(pcm[2] == (int16_t)AUDIO_CLIP_TARGET_PEAK); /* 200 * 120 = 24000 */
    ASSERT(pcm[3] == -18000);                          /* -150 * 120 */
}

static void test_silence_and_sub_threshold_noise_not_amplified(void)
{
    /* Sub-threshold noise with peak 10 (below AUDIO_CLIP_NOISE_THRESHOLD = 32).
     * Must remain at unity gain (1.0x) so background floor is not blown up. */
    int16_t pcm[4] = {5, -10, 8, -3};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 4, &st);

    ASSERT(st.raw_peak == 10);
    ASSERT(st.gain_q8 == 256u); /* 1.0x unity gain */
    ASSERT(st.peak == 10);
    ASSERT(st.clipped == 0u);
    for (size_t i = 0; i < 4; ++i) {
        ASSERT(pcm[i] >= -10 && pcm[i] <= 10);
    }
}

static void test_flat_silence_not_amplified(void)
{
    /* Pure digital silence (flat zeros). */
    int16_t pcm[8] = {0};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 8, &st);

    ASSERT(st.raw_peak == 0);
    ASSERT(st.peak == 0);
    ASSERT(st.gain_q8 == 256u);
    ASSERT(st.clipped == 0u);
    for (size_t i = 0; i < 8; ++i) {
        ASSERT(pcm[i] == 0);
    }
}

static void test_loud_audio_not_over_amplified(void)
{
    /* Audio already louder than target peak: peak 28000.
     * Must remain at unity gain (1.0x) with no over-amplification or distortion. */
    int16_t pcm[4] = {14000, -28000, 20000, -6000};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 4, &st);

    ASSERT(st.dc == 0);
    ASSERT(st.raw_peak == 28000);
    ASSERT(st.gain_q8 == 256u); /* Unity gain */
    ASSERT(st.peak == 28000);
    ASSERT(st.clipped == 0u);
    ASSERT(pcm[0] == 14000);
    ASSERT(pcm[1] == -28000);
    ASSERT(pcm[2] == 20000);
    ASSERT(pcm[3] == -6000);
}

static void test_audio_at_exact_target_peak(void)
{
    int16_t pcm[2] = {24000, -24000};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 2, &st);

    ASSERT(st.raw_peak == 24000);
    ASSERT(st.gain_q8 == 256u);
    ASSERT(st.peak == 24000);
    ASSERT(st.clipped == 0u);
    ASSERT(pcm[0] == 24000);
    ASSERT(pcm[1] == -24000);
}

static void test_max_gain_cap_enforced(void)
{
    /* Faint signal just above threshold (peak 50).
     * 24000 / 50 = 480x, which exceeds AUDIO_CLIP_MAX_GAIN_FACTOR (128x).
     * Gain must be capped at 128x. */
    int16_t pcm[2] = {50, -50};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 2, &st);

    ASSERT(st.raw_peak == 50);
    ASSERT(st.gain_q8 == (uint32_t)AUDIO_CLIP_MAX_GAIN_FACTOR * 256u);
    ASSERT(st.peak == (int16_t)(50 * AUDIO_CLIP_MAX_GAIN_FACTOR));
    ASSERT(st.clipped == 0u);
    ASSERT(pcm[0] == (int16_t)(50 * AUDIO_CLIP_MAX_GAIN_FACTOR));
    ASSERT(pcm[1] == (int16_t)(-50 * AUDIO_CLIP_MAX_GAIN_FACTOR));
}

static void test_clipping_count_accuracy(void)
{
    /* 6 samples: two at +32000, four at -32000.
     * sum = 64000 - 128000 = -64000.
     * dc = -64000 / 6 = -10666.
     * Positive samples after DC subtraction: 32000 - (-10666) = 42666 > 32767.
     * Both must saturate to 32767 and st.clipped must accurately report 2. */
    int16_t pcm[6] = {32000, 32000, -32000, -32000, -32000, -32000};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 6, &st);

    ASSERT(st.clipped == 2u);
    ASSERT(pcm[0] == 32767);
    ASSERT(pcm[1] == 32767);
    ASSERT(st.peak == 32767);
}

static void test_clipping_saturates_rather_than_wraps(void)
{
    /* A negative DC pushes positive samples past full scale. Wrapping would flip
     * their sign — an audible click, and it would also under-count `clipped`,
     * which is the number that says "the gain is too high". */
    int16_t pcm[4] = {32000, 32000, 32000, -32768};
    audio_clip_stats_t st;
    audio_clip_normalise(pcm, 4, &st);

    for (size_t i = 0; i < 3; ++i) {
        ASSERT(pcm[i] > 0); /* still positive: saturated, not wrapped */
    }
    ASSERT(st.clipped >= 1u);
    ASSERT(st.peak == 32767);
}

static void test_degenerate_input_is_safe(void)
{
    audio_clip_stats_t st;
    audio_clip_normalise(NULL, 8, &st);
    ASSERT(st.samples == 0u);

    int16_t pcm[1] = {5};
    audio_clip_normalise(pcm, 0, &st);
    ASSERT(st.samples == 0u);
}

/* =========================================================================
 * Runner
 * ========================================================================= */

int main(void)
{
    printf("=== audio_clip host tests ===\n\n");

    test_run("a nominal window sizes correctly", test_nominal_window_sizes_correctly);
    test_run("zero parameters refuse", test_zero_parameters_refuse);
    test_run("the cap is enforced exactly", test_the_cap_is_enforced_exactly);
    test_run("overflow refuses rather than wraps", test_overflow_refuses_rather_than_wraps);
    test_run("base64 length includes the header", test_b64_length_includes_the_header);
    test_run("base64 length refuses out of range", test_b64_length_refuses_out_of_range);

    test_run("the WAV header is byte exact", test_wav_header_is_byte_exact);
    test_run("the WAV header refuses degenerate input", test_wav_header_refuses_degenerate_input);

    test_run("DC is removed and reported", test_dc_is_removed_and_reported);
    test_run("peak survives DC removal", test_peak_survives_dc_removal);
    test_run("faint audio amplified to target", test_faint_audio_amplified_to_target);
    test_run("silence and sub-threshold noise not amplified",
             test_silence_and_sub_threshold_noise_not_amplified);
    test_run("flat silence not amplified", test_flat_silence_not_amplified);
    test_run("loud audio not over-amplified", test_loud_audio_not_over_amplified);
    test_run("audio at exact target peak", test_audio_at_exact_target_peak);
    test_run("max gain cap enforced", test_max_gain_cap_enforced);
    test_run("clipping count accuracy", test_clipping_count_accuracy);
    test_run("clipping saturates rather than wraps", test_clipping_saturates_rather_than_wraps);
    test_run("degenerate input is safe", test_degenerate_input_is_safe);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
