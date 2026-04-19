/**
 * @file test_audio.c
 * @brief Host-based unit tests for thinkpack-audio.
 */

#include <string.h>

#include "thinkpack_audio.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Volume cap                                                          */
/* ------------------------------------------------------------------ */

static void test_volume_cap_scales_samples(void)
{
    int16_t samples[4] = {10000, -10000, 0, 20000};
    thinkpack_audio_apply_volume_cap(samples, 4);
    /* 60% scale: 10000*60/100 = 6000; -10000 -> -6000; 0 -> 0; 20000 -> 12000 */
    TEST_ASSERT_EQUAL(6000, samples[0]);
    TEST_ASSERT_EQUAL(-6000, samples[1]);
    TEST_ASSERT_EQUAL(0, samples[2]);
    TEST_ASSERT_EQUAL(12000, samples[3]);
}

static void test_volume_cap_handles_null_and_zero(void)
{
    thinkpack_audio_apply_volume_cap(NULL, 0);
    int16_t s = 100;
    thinkpack_audio_apply_volume_cap(&s, 0);
    TEST_ASSERT_EQUAL(100, s);
}

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */

static void test_ring_basic_write_read(void)
{
    int16_t storage[8];
    thinkpack_audio_ring_t ring;
    thinkpack_audio_ring_init(&ring, storage, 8);

    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_used(&ring));
    TEST_ASSERT_EQUAL(8, thinkpack_audio_ring_free(&ring));

    int16_t in[] = {1, 2, 3, 4, 5};
    size_t w = thinkpack_audio_ring_write(&ring, in, 5);
    TEST_ASSERT_EQUAL(5, w);
    TEST_ASSERT_EQUAL(5, thinkpack_audio_ring_used(&ring));
    TEST_ASSERT_EQUAL(3, thinkpack_audio_ring_free(&ring));

    int16_t out[10] = {0};
    size_t r = thinkpack_audio_ring_read(&ring, out, 10);
    TEST_ASSERT_EQUAL(5, r);
    TEST_ASSERT_EQUAL(1, out[0]);
    TEST_ASSERT_EQUAL(5, out[4]);
    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_used(&ring));
}

static void test_ring_write_full_rejects_overflow(void)
{
    int16_t storage[4];
    thinkpack_audio_ring_t ring;
    thinkpack_audio_ring_init(&ring, storage, 4);

    int16_t in[] = {10, 20, 30, 40, 50, 60};
    size_t w = thinkpack_audio_ring_write(&ring, in, 6);
    TEST_ASSERT_EQUAL(4, w);
    TEST_ASSERT_EQUAL(4, thinkpack_audio_ring_used(&ring));
    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_free(&ring));

    /* Further writes return 0 and do not corrupt content */
    size_t w2 = thinkpack_audio_ring_write(&ring, in + 4, 2);
    TEST_ASSERT_EQUAL(0, w2);

    int16_t out[4];
    size_t r = thinkpack_audio_ring_read(&ring, out, 4);
    TEST_ASSERT_EQUAL(4, r);
    TEST_ASSERT_EQUAL(10, out[0]);
    TEST_ASSERT_EQUAL(40, out[3]);
}

static void test_ring_wraparound(void)
{
    int16_t storage[4];
    thinkpack_audio_ring_t ring;
    thinkpack_audio_ring_init(&ring, storage, 4);

    /* Fill, drain 2, refill across the seam */
    int16_t in1[] = {1, 2, 3, 4};
    thinkpack_audio_ring_write(&ring, in1, 4);
    int16_t tmp[2];
    thinkpack_audio_ring_read(&ring, tmp, 2);
    TEST_ASSERT_EQUAL(1, tmp[0]);
    TEST_ASSERT_EQUAL(2, tmp[1]);

    int16_t in2[] = {5, 6};
    size_t w = thinkpack_audio_ring_write(&ring, in2, 2);
    TEST_ASSERT_EQUAL(2, w);
    TEST_ASSERT_EQUAL(4, thinkpack_audio_ring_used(&ring));

    /* Drain — must read 3,4,5,6 in order despite head wrap */
    int16_t out[4];
    thinkpack_audio_ring_read(&ring, out, 4);
    TEST_ASSERT_EQUAL(3, out[0]);
    TEST_ASSERT_EQUAL(4, out[1]);
    TEST_ASSERT_EQUAL(5, out[2]);
    TEST_ASSERT_EQUAL(6, out[3]);
    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_used(&ring));
}

static void test_ring_reset_drops_content(void)
{
    int16_t storage[4];
    thinkpack_audio_ring_t ring;
    thinkpack_audio_ring_init(&ring, storage, 4);

    int16_t in[] = {7, 8, 9};
    thinkpack_audio_ring_write(&ring, in, 3);
    thinkpack_audio_ring_reset(&ring);

    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_used(&ring));
    TEST_ASSERT_EQUAL(4, thinkpack_audio_ring_free(&ring));

    int16_t out[4];
    TEST_ASSERT_EQUAL(0, thinkpack_audio_ring_read(&ring, out, 4));
}

/* ------------------------------------------------------------------ */
/* Pitch-shift resampler                                               */
/* ------------------------------------------------------------------ */

static void test_pitch_shift_zero_passes_through(void)
{
    int16_t in[] = {1, 2, 3, 4, 5, 6};
    int16_t out[6] = {0};
    size_t n = thinkpack_audio_pitch_shift(in, 6, 0, out, 6);
    TEST_ASSERT_EQUAL(6, n);
    for (size_t i = 0; i < 6; i++) {
        TEST_ASSERT_EQUAL(in[i], out[i]);
    }
}

static void test_pitch_shift_octave_up_halves_output(void)
{
    int16_t in[8];
    for (int i = 0; i < 8; i++)
        in[i] = (int16_t)i;

    int16_t out[8] = {0};
    size_t n = thinkpack_audio_pitch_shift(in, 8, +12, out, 8);
    /* Octave up: ratio = 2, consumes input twice as fast.  From 8 input
     * samples we should emit 4 output samples before exhausting input. */
    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_EQUAL(0, out[0]);
    TEST_ASSERT_EQUAL(2, out[1]);
    TEST_ASSERT_EQUAL(4, out[2]);
    TEST_ASSERT_EQUAL(6, out[3]);
}

static void test_pitch_shift_octave_down_doubles_output(void)
{
    int16_t in[4] = {10, 20, 30, 40};
    int16_t out[8] = {0};
    size_t n = thinkpack_audio_pitch_shift(in, 4, -12, out, 8);
    /* Octave down: ratio = 0.5, each input sample emits two copies. */
    TEST_ASSERT_EQUAL(8, n);
    TEST_ASSERT_EQUAL(10, out[0]);
    TEST_ASSERT_EQUAL(10, out[1]);
    TEST_ASSERT_EQUAL(20, out[2]);
    TEST_ASSERT_EQUAL(20, out[3]);
    TEST_ASSERT_EQUAL(40, out[6]);
    TEST_ASSERT_EQUAL(40, out[7]);
}

static void test_pitch_shift_respects_out_capacity(void)
{
    int16_t in[16];
    for (int i = 0; i < 16; i++)
        in[i] = (int16_t)(100 + i);
    int16_t out[4] = {0};
    size_t n = thinkpack_audio_pitch_shift(in, 16, -12, out, 4);
    TEST_ASSERT_EQUAL(4, n);
}

static void test_pitch_shift_clamps_extreme_shift(void)
{
    int16_t in[4] = {1, 2, 3, 4};
    int16_t out1[4] = {0};
    int16_t out2[4] = {0};
    size_t a = thinkpack_audio_pitch_shift(in, 4, +40, out1, 4);
    size_t b = thinkpack_audio_pitch_shift(in, 4, +12, out2, 4);
    TEST_ASSERT_EQUAL(b, a);
    TEST_ASSERT_EQUAL_MEMORY(out2, out1, sizeof(out1));
}

static void test_pitch_shift_handles_empty_inputs(void)
{
    int16_t out[4] = {0};
    TEST_ASSERT_EQUAL(0, thinkpack_audio_pitch_shift(NULL, 0, 0, out, 4));

    int16_t in[1] = {1};
    TEST_ASSERT_EQUAL(0, thinkpack_audio_pitch_shift(in, 1, 0, NULL, 0));
    TEST_ASSERT_EQUAL(0, thinkpack_audio_pitch_shift(in, 0, 0, out, 4));
    TEST_ASSERT_EQUAL(0, thinkpack_audio_pitch_shift(in, 1, 0, out, 0));
}

/* ------------------------------------------------------------------ */
/* State machine                                                       */
/* ------------------------------------------------------------------ */

static void test_sm_initial_state(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_IDLE, sm.state);
    TEST_ASSERT_EQUAL(0, sm.recorded_samples);
}

static void test_sm_happy_path_record_play_done(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);

    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_RECORDING,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START));
    thinkpack_audio_sm_add_samples(&sm, 8000);
    TEST_ASSERT_EQUAL(8000, sm.recorded_samples);

    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_PLAYING,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_END));
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_IDLE,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_CLIP_DONE));
}

static void test_sm_empty_recording_returns_to_idle(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);

    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START);
    /* No samples recorded */
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_IDLE,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_END));
}

static void test_sm_limit_reached_forces_playback(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START);
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_PLAYING,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_LIMIT_REACHED));
}

static void test_sm_cancel_during_recording(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START);
    thinkpack_audio_sm_add_samples(&sm, 4000);
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_IDLE,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_CANCEL));
    TEST_ASSERT_EQUAL(0, sm.recorded_samples);
}

static void test_sm_retrigger_during_playback(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START);
    thinkpack_audio_sm_add_samples(&sm, 1234);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_END);
    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_PLAYING, sm.state);

    TEST_ASSERT_EQUAL(THINKPACK_AUDIO_STATE_RECORDING,
                      thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START));
    TEST_ASSERT_EQUAL(0, sm.recorded_samples);
}

static void test_sm_add_samples_only_in_recording(void)
{
    thinkpack_audio_sm_t sm;
    thinkpack_audio_sm_init(&sm);
    /* IDLE: add_samples is a no-op */
    thinkpack_audio_sm_add_samples(&sm, 999);
    TEST_ASSERT_EQUAL(0, sm.recorded_samples);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_TOUCH_START);
    thinkpack_audio_sm_add_samples(&sm, 100);
    thinkpack_audio_sm_handle(&sm, THINKPACK_AUDIO_EVENT_LIMIT_REACHED);
    /* PLAYING: add_samples is a no-op */
    thinkpack_audio_sm_add_samples(&sm, 999);
    TEST_ASSERT_EQUAL(100, sm.recorded_samples);
}

/* ------------------------------------------------------------------ */
/* Test runner                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_volume_cap_scales_samples);
    RUN_TEST(test_volume_cap_handles_null_and_zero);

    RUN_TEST(test_ring_basic_write_read);
    RUN_TEST(test_ring_write_full_rejects_overflow);
    RUN_TEST(test_ring_wraparound);
    RUN_TEST(test_ring_reset_drops_content);

    RUN_TEST(test_pitch_shift_zero_passes_through);
    RUN_TEST(test_pitch_shift_octave_up_halves_output);
    RUN_TEST(test_pitch_shift_octave_down_doubles_output);
    RUN_TEST(test_pitch_shift_respects_out_capacity);
    RUN_TEST(test_pitch_shift_clamps_extreme_shift);
    RUN_TEST(test_pitch_shift_handles_empty_inputs);

    RUN_TEST(test_sm_initial_state);
    RUN_TEST(test_sm_happy_path_record_play_done);
    RUN_TEST(test_sm_empty_recording_returns_to_idle);
    RUN_TEST(test_sm_limit_reached_forces_playback);
    RUN_TEST(test_sm_cancel_during_recording);
    RUN_TEST(test_sm_retrigger_during_playback);
    RUN_TEST(test_sm_add_samples_only_in_recording);

    int failures = UNITY_END();
    return failures == 0 ? 0 : 1;
}
