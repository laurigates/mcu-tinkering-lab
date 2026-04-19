/**
 * @file thinkpack_audio.c
 * @brief Implementation of thinkpack-audio (pure logic: no ESP-IDF deps).
 */

#include "thinkpack_audio.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Volume cap                                                          */
/* ------------------------------------------------------------------ */

void thinkpack_audio_apply_volume_cap(int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0) {
        return;
    }
    for (size_t i = 0; i < sample_count; i++) {
        int32_t scaled = (int32_t)samples[i] * THINKPACK_AUDIO_VOLUME_SCALE_NUM /
                         THINKPACK_AUDIO_VOLUME_SCALE_DEN;
        if (scaled > INT16_MAX) {
            scaled = INT16_MAX;
        } else if (scaled < INT16_MIN) {
            scaled = INT16_MIN;
        }
        samples[i] = (int16_t)scaled;
    }
}

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */

void thinkpack_audio_ring_init(thinkpack_audio_ring_t *ring, int16_t *buffer, size_t capacity)
{
    ring->buffer = buffer;
    ring->capacity = capacity;
    ring->head = 0;
    ring->tail = 0;
    ring->count = 0;
}

void thinkpack_audio_ring_reset(thinkpack_audio_ring_t *ring)
{
    ring->head = 0;
    ring->tail = 0;
    ring->count = 0;
}

size_t thinkpack_audio_ring_used(const thinkpack_audio_ring_t *ring)
{
    return ring->count;
}

size_t thinkpack_audio_ring_free(const thinkpack_audio_ring_t *ring)
{
    return ring->capacity - ring->count;
}

size_t thinkpack_audio_ring_write(thinkpack_audio_ring_t *ring, const int16_t *samples,
                                  size_t count)
{
    if (ring->buffer == NULL || ring->capacity == 0) {
        return 0;
    }
    size_t free_space = ring->capacity - ring->count;
    size_t to_write = (count < free_space) ? count : free_space;
    for (size_t i = 0; i < to_write; i++) {
        ring->buffer[ring->head] = samples[i];
        ring->head = (ring->head + 1) % ring->capacity;
    }
    ring->count += to_write;
    return to_write;
}

size_t thinkpack_audio_ring_read(thinkpack_audio_ring_t *ring, int16_t *out, size_t count)
{
    if (ring->buffer == NULL || ring->capacity == 0) {
        return 0;
    }
    size_t to_read = (count < ring->count) ? count : ring->count;
    for (size_t i = 0; i < to_read; i++) {
        out[i] = ring->buffer[ring->tail];
        ring->tail = (ring->tail + 1) % ring->capacity;
    }
    ring->count -= to_read;
    return to_read;
}

/* ------------------------------------------------------------------ */
/* Pitch-shift resampler                                               */
/* ------------------------------------------------------------------ */

/* Q16.16 ratio table for semitones -12..+12.
 * ratio = 2^(n/12), stored as round(ratio * 65536).
 * For positive shift, output advances through input faster → pitch up.
 */
static const uint32_t k_pitch_ratio_q16[] = {
    32768,  /* -12 */
    34716,  /* -11 */
    36780,  /* -10 */
    38967,  /* -9  */
    41285,  /* -8  */
    43740,  /* -7  */
    46341,  /* -6  */
    49097,  /* -5  */
    52016,  /* -4  */
    55109,  /* -3  */
    58386,  /* -2  */
    61858,  /* -1  */
    65536,  /* 0   */
    69433,  /* +1  */
    73562,  /* +2  */
    77936,  /* +3  */
    82570,  /* +4  */
    87480,  /* +5  */
    92682,  /* +6  */
    98193,  /* +7  */
    104032, /* +8  */
    110218, /* +9  */
    116772, /* +10 */
    123715, /* +11 */
    131072, /* +12 */
};

size_t thinkpack_audio_pitch_shift(const int16_t *in, size_t in_samples, int8_t semitone_shift,
                                   int16_t *out, size_t out_capacity)
{
    if (in == NULL || out == NULL || in_samples == 0 || out_capacity == 0) {
        return 0;
    }
    if (semitone_shift < THINKPACK_AUDIO_PITCH_MIN_SEMITONES) {
        semitone_shift = THINKPACK_AUDIO_PITCH_MIN_SEMITONES;
    }
    if (semitone_shift > THINKPACK_AUDIO_PITCH_MAX_SEMITONES) {
        semitone_shift = THINKPACK_AUDIO_PITCH_MAX_SEMITONES;
    }
    int index = (int)semitone_shift - THINKPACK_AUDIO_PITCH_MIN_SEMITONES;
    uint32_t ratio_q16 = k_pitch_ratio_q16[index];

    /* accumulator counts input-sample offsets in Q16.16.
     * For each output sample we read input[acc >> 16] then advance by ratio_q16. */
    uint32_t acc = 0;
    size_t produced = 0;
    while (produced < out_capacity) {
        size_t idx = (size_t)(acc >> 16);
        if (idx >= in_samples) {
            break;
        }
        out[produced++] = in[idx];
        acc += ratio_q16;
    }
    return produced;
}

/* ------------------------------------------------------------------ */
/* Record/playback state machine                                       */
/* ------------------------------------------------------------------ */

void thinkpack_audio_sm_init(thinkpack_audio_sm_t *sm)
{
    sm->state = THINKPACK_AUDIO_STATE_IDLE;
    sm->recorded_samples = 0;
}

thinkpack_audio_state_t thinkpack_audio_sm_handle(thinkpack_audio_sm_t *sm,
                                                  thinkpack_audio_event_t ev)
{
    switch (sm->state) {
        case THINKPACK_AUDIO_STATE_IDLE:
            if (ev == THINKPACK_AUDIO_EVENT_TOUCH_START) {
                sm->state = THINKPACK_AUDIO_STATE_RECORDING;
                sm->recorded_samples = 0;
            }
            break;

        case THINKPACK_AUDIO_STATE_RECORDING:
            if (ev == THINKPACK_AUDIO_EVENT_TOUCH_END) {
                sm->state = (sm->recorded_samples > 0) ? THINKPACK_AUDIO_STATE_PLAYING
                                                       : THINKPACK_AUDIO_STATE_IDLE;
            } else if (ev == THINKPACK_AUDIO_EVENT_LIMIT_REACHED) {
                sm->state = THINKPACK_AUDIO_STATE_PLAYING;
            } else if (ev == THINKPACK_AUDIO_EVENT_CANCEL) {
                sm->state = THINKPACK_AUDIO_STATE_IDLE;
                sm->recorded_samples = 0;
            }
            break;

        case THINKPACK_AUDIO_STATE_PLAYING:
            if (ev == THINKPACK_AUDIO_EVENT_CLIP_DONE || ev == THINKPACK_AUDIO_EVENT_CANCEL) {
                sm->state = THINKPACK_AUDIO_STATE_IDLE;
            } else if (ev == THINKPACK_AUDIO_EVENT_TOUCH_START) {
                sm->state = THINKPACK_AUDIO_STATE_RECORDING;
                sm->recorded_samples = 0;
            }
            break;
    }
    return sm->state;
}

void thinkpack_audio_sm_add_samples(thinkpack_audio_sm_t *sm, uint32_t n)
{
    if (sm->state == THINKPACK_AUDIO_STATE_RECORDING) {
        sm->recorded_samples += n;
    }
}
