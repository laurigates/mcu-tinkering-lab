/**
 * @file thinkpack_audio.h
 * @brief Pure-logic audio helpers for ThinkPack Chatterbox and Finderbox.
 *
 * This component packages four reusable, host-testable pieces:
 *
 *  - Toddler-safe volume cap (in-place gain reduction).
 *  - A lock-free single-producer/single-consumer ring buffer of int16 PCM.
 *  - A nearest-neighbor pitch-shift resampler driven by semitone shift.
 *  - A record/playback state machine that models a hold-to-record toggle.
 *
 * No ESP-IDF dependencies; safe to compile on a host with gcc/clang.
 */

#ifndef THINKPACK_AUDIO_H
#define THINKPACK_AUDIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Audio constants                                                     */
/* ------------------------------------------------------------------ */

/** 16 kHz mono — matches INMP441 / MAX98357A target for Chatterbox. */
#define THINKPACK_AUDIO_SAMPLE_RATE 16000

/** Longest clip we ever keep in PSRAM (2 s). */
#define THINKPACK_AUDIO_CLIP_MAX_MS 2000

/** Maximum clip length in samples (32000 @ 16 kHz). */
#define THINKPACK_AUDIO_CLIP_MAX_SAMPLES \
    ((THINKPACK_AUDIO_SAMPLE_RATE * THINKPACK_AUDIO_CLIP_MAX_MS) / 1000)

/** Toddler-safe attenuation: 60 % of full-scale int16 range. */
#define THINKPACK_AUDIO_VOLUME_SCALE_NUM 60
#define THINKPACK_AUDIO_VOLUME_SCALE_DEN 100

/** Clamp for semitone shift — two octaves is more than enough for a toy. */
#define THINKPACK_AUDIO_PITCH_MIN_SEMITONES (-12)
#define THINKPACK_AUDIO_PITCH_MAX_SEMITONES (+12)

/* ------------------------------------------------------------------ */
/* Volume cap                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Apply the toddler-safe volume cap in place.
 *
 * Scales every sample by THINKPACK_AUDIO_VOLUME_SCALE_NUM /
 * THINKPACK_AUDIO_VOLUME_SCALE_DEN.  Samples are clamped to the int16
 * range on overflow, which can happen only if an upstream stage has
 * already amplified beyond full scale.
 *
 * @param samples      PCM buffer to modify.  Safe to pass NULL when
 *                     sample_count is 0.
 * @param sample_count Number of samples in @p samples.
 */
void thinkpack_audio_apply_volume_cap(int16_t *samples, size_t sample_count);

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Single-producer/single-consumer ring buffer of int16 samples.
 *
 * The buffer is supplied by the caller; this struct only tracks indices.
 * Pass the same @p buffer and @p capacity to init() that backs the
 * storage for the lifetime of the ring.
 */
typedef struct {
    int16_t *buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
} thinkpack_audio_ring_t;

/** Initialise the ring. @p capacity must be > 0. */
void thinkpack_audio_ring_init(thinkpack_audio_ring_t *ring, int16_t *buffer, size_t capacity);

/** Drop all samples without freeing the storage. */
void thinkpack_audio_ring_reset(thinkpack_audio_ring_t *ring);

/** Number of samples currently stored. */
size_t thinkpack_audio_ring_used(const thinkpack_audio_ring_t *ring);

/** Free space available for writes. */
size_t thinkpack_audio_ring_free(const thinkpack_audio_ring_t *ring);

/**
 * @brief Copy up to @p count samples from @p samples into the ring.
 *
 * Writes stop early once the ring is full.
 *
 * @return Number of samples actually written (<= @p count).
 */
size_t thinkpack_audio_ring_write(thinkpack_audio_ring_t *ring, const int16_t *samples,
                                  size_t count);

/**
 * @brief Read up to @p count samples from the ring into @p out.
 *
 * Reads stop early once the ring is empty.
 *
 * @return Number of samples actually read (<= @p count).
 */
size_t thinkpack_audio_ring_read(thinkpack_audio_ring_t *ring, int16_t *out, size_t count);

/* ------------------------------------------------------------------ */
/* Pitch-shift resampler                                               */
/* ------------------------------------------------------------------ */

/**
 * @brief Nearest-neighbor pitch-shift via integer resampling.
 *
 * @p semitone_shift is clamped to
 * [THINKPACK_AUDIO_PITCH_MIN_SEMITONES, THINKPACK_AUDIO_PITCH_MAX_SEMITONES].
 * Positive shift raises pitch (reads input faster, so fewer output
 * samples per input sample).  The resampler is intentionally simple —
 * it is good enough for 2 s toy clips but does not implement any
 * anti-alias filtering.
 *
 * @param in            Input PCM samples.
 * @param in_samples    Number of samples in @p in.
 * @param semitone_shift Signed semitone offset.
 * @param out           Destination buffer.
 * @param out_capacity  Size of @p out in samples.
 * @return              Number of output samples produced (<= @p out_capacity).
 */
size_t thinkpack_audio_pitch_shift(const int16_t *in, size_t in_samples, int8_t semitone_shift,
                                   int16_t *out, size_t out_capacity);

/* ------------------------------------------------------------------ */
/* Record/playback state machine                                       */
/* ------------------------------------------------------------------ */

typedef enum {
    THINKPACK_AUDIO_STATE_IDLE = 0,
    THINKPACK_AUDIO_STATE_RECORDING,
    THINKPACK_AUDIO_STATE_PLAYING,
} thinkpack_audio_state_t;

typedef enum {
    THINKPACK_AUDIO_EVENT_TOUCH_START = 0,
    THINKPACK_AUDIO_EVENT_TOUCH_END,
    THINKPACK_AUDIO_EVENT_CLIP_DONE,
    THINKPACK_AUDIO_EVENT_LIMIT_REACHED,
    THINKPACK_AUDIO_EVENT_CANCEL,
} thinkpack_audio_event_t;

typedef struct {
    thinkpack_audio_state_t state;
    uint32_t recorded_samples;
} thinkpack_audio_sm_t;

/** Reset to IDLE with zero recorded samples. */
void thinkpack_audio_sm_init(thinkpack_audio_sm_t *sm);

/**
 * @brief Advance the state machine by one event.
 *
 * Transitions:
 *   IDLE      + TOUCH_START    -> RECORDING (recorded_samples := 0)
 *   RECORDING + TOUCH_END      -> PLAYING if recorded_samples > 0 else IDLE
 *   RECORDING + LIMIT_REACHED  -> PLAYING
 *   RECORDING + CANCEL         -> IDLE (recorded_samples := 0)
 *   PLAYING   + CLIP_DONE      -> IDLE
 *   PLAYING   + TOUCH_START    -> RECORDING (recorded_samples := 0)
 *   PLAYING   + CANCEL         -> IDLE
 *   anything else              -> no state change
 *
 * @return the new state.
 */
thinkpack_audio_state_t thinkpack_audio_sm_handle(thinkpack_audio_sm_t *sm,
                                                  thinkpack_audio_event_t ev);

/**
 * @brief Account for @p n additional recorded samples while in RECORDING.
 *
 * No-op if the state machine is not in RECORDING.
 */
void thinkpack_audio_sm_add_samples(thinkpack_audio_sm_t *sm, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif /* THINKPACK_AUDIO_H */
