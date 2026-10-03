/**
 * @file voice_preroll.c
 * @brief Pre-roll ring for hands-free voice turns. See voice_preroll.h for what
 *        may enter and why taking empties it.
 *
 * Pure C — test/test_voice_preroll.c builds it on the host.
 */

#include "voice_preroll.h"

#include <string.h>

void voice_preroll_init(voice_preroll_t *rb, int16_t *storage, size_t cap)
{
    if (rb == NULL) {
        return;
    }
    rb->buf = storage;
    rb->cap = (storage != NULL) ? cap : 0u;
    rb->head = 0u;
    rb->count = 0u;
}

void voice_preroll_reset(voice_preroll_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0u;
    rb->count = 0u;
}

/** Append @p n samples (or zeros when @p pcm is NULL), keeping the newest cap. */
static void append(voice_preroll_t *rb, const int16_t *pcm, size_t n)
{
    /* Only the last cap samples of an oversized frame can survive, so skip the
     * rest rather than writing them only to overwrite them. */
    if (n > rb->cap) {
        if (pcm != NULL) {
            pcm += n - rb->cap;
        }
        n = rb->cap;
    }

    size_t done = 0u;
    while (done < n) {
        size_t run = rb->cap - rb->head;
        if (run > n - done) {
            run = n - done;
        }
        if (pcm != NULL) {
            memcpy(&rb->buf[rb->head], &pcm[done], run * sizeof(int16_t));
        } else {
            memset(&rb->buf[rb->head], 0, run * sizeof(int16_t));
        }
        rb->head = (rb->head + run) % rb->cap;
        done += run;
    }

    rb->count += n;
    if (rb->count > rb->cap) {
        rb->count = rb->cap;
    }
}

void voice_preroll_offer(voice_preroll_t *rb, const int16_t *pcm, size_t n, bool capture_allowed,
                         bool cue_active)
{
    if (rb == NULL) {
        return;
    }
    /* Quarantine first, and it empties rather than skips: nothing from before the
     * robot's own voice may be joined to what comes after it. */
    if (!capture_allowed) {
        voice_preroll_reset(rb);
        return;
    }
    if (rb->cap == 0u || n == 0u) {
        return;
    }
    if (cue_active) {
        append(rb, NULL, n); /* the beep becomes silence of the same length */
        return;
    }
    if (pcm == NULL) {
        return;
    }
    append(rb, pcm, n);
}

size_t voice_preroll_count(const voice_preroll_t *rb)
{
    return (rb != NULL) ? rb->count : 0u;
}

size_t voice_preroll_take(voice_preroll_t *rb, int16_t *dst, size_t max)
{
    if (rb == NULL) {
        return 0u;
    }
    size_t n = rb->count;
    if (n > max) {
        n = max; /* drop the oldest; the newest join the recording that follows */
    }
    if (dst == NULL) {
        n = 0u;
    }

    if (n > 0u) {
        /* The newest sample is just behind head; the n-th newest is n behind it. */
        size_t start = (rb->head + rb->cap - n) % rb->cap;
        size_t done = 0u;
        while (done < n) {
            size_t run = rb->cap - start;
            if (run > n - done) {
                run = n - done;
            }
            memcpy(&dst[done], &rb->buf[start], run * sizeof(int16_t));
            start = (start + run) % rb->cap;
            done += run;
        }
    }

    voice_preroll_reset(rb);
    return n;
}
