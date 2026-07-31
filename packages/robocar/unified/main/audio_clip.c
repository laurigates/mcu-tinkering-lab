/**
 * @file audio_clip.c
 * @brief Sizing, WAV framing and normalisation for a recorded microphone clip.
 *
 * Pure C — no ESP-IDF, no FreeRTOS — so the arithmetic and the header layout are
 * exercised by test_audio_clip.c on the host. See audio_clip.h for why both are
 * worth isolating.
 */

#include "audio_clip.h"

#include <string.h>

/** Bytes per sample. Mono 16-bit throughout this firmware; the mic offers
 *  nothing else and Gemini is happy with it. */
#define BYTES_PER_SAMPLE 2u

/** Little-endian stores, written out rather than memcpy'd from a struct because
 *  a struct would need packing attributes and would still not fix byte order on
 *  a big-endian host test runner. */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

size_t audio_clip_pcm_bytes(uint32_t window_ms, uint32_t sample_rate_hz)
{
    if (window_ms == 0u || sample_rate_hz == 0u) {
        return 0u;
    }

    /* Check the multiplication BEFORE performing it. Doing it the other way —
     * multiply, then notice the result looks small — is exactly the bug this
     * function exists to make impossible: a wrapped product is a perfectly
     * ordinary-looking small number. */
    const uint64_t samples = ((uint64_t)window_ms * (uint64_t)sample_rate_hz) / 1000u;
    const uint64_t bytes = samples * (uint64_t)BYTES_PER_SAMPLE;
    if (bytes == 0u || bytes > (uint64_t)AUDIO_CLIP_MAX_BYTES) {
        return 0u;
    }
    return (size_t)bytes;
}

size_t audio_clip_b64_length(size_t pcm_bytes)
{
    if (pcm_bytes == 0u || pcm_bytes > AUDIO_CLIP_MAX_BYTES) {
        return 0u;
    }
    /* The header is part of what gets encoded. Omitting it here under-allocates
     * by 60 characters and corrupts only the tail of the audio — which presents
     * as a microphone fault, not as an arithmetic one. */
    const uint64_t total = (uint64_t)pcm_bytes + (uint64_t)AUDIO_CLIP_WAV_HEADER_BYTES;
    /* 4 chars per 3 bytes, rounded up, plus the NUL. */
    const uint64_t b64 = ((total + 2u) / 3u) * 4u + 1u;
    if (b64 > (uint64_t)SIZE_MAX) {
        return 0u;
    }
    return (size_t)b64;
}

bool audio_clip_wav_header(uint8_t *out, size_t pcm_bytes, uint32_t sample_rate_hz,
                           uint16_t channels, uint16_t bits)
{
    if (!out || pcm_bytes == 0u || sample_rate_hz == 0u || channels == 0u || bits == 0u ||
        (bits % 8u) != 0u) {
        return false;
    }

    const uint16_t block_align = (uint16_t)(channels * (bits / 8u));
    const uint32_t byte_rate = sample_rate_hz * block_align;

    memcpy(out + 0, "RIFF", 4);
    put_u32(out + 4, (uint32_t)(36u + pcm_bytes)); /* file size minus the 8-byte RIFF preamble */
    memcpy(out + 8, "WAVE", 4);
    memcpy(out + 12, "fmt ", 4);
    put_u32(out + 16, 16u); /* PCM fmt chunk length */
    put_u16(out + 20, 1u);  /* audio format: 1 = PCM, uncompressed */
    put_u16(out + 22, channels);
    put_u32(out + 24, sample_rate_hz);
    put_u32(out + 28, byte_rate);
    put_u16(out + 32, block_align);
    put_u16(out + 34, bits);
    memcpy(out + 36, "data", 4);
    put_u32(out + 40, (uint32_t)pcm_bytes);
    return true;
}

void audio_clip_normalise(int16_t *pcm, size_t samples, audio_clip_stats_t *out_stats)
{
    audio_clip_stats_t st = {0};
    if (!pcm || samples == 0u) {
        if (out_stats) {
            *out_stats = st;
        }
        return;
    }
    st.samples = (uint32_t)samples;

    int64_t sum = 0;
    for (size_t i = 0; i < samples; ++i) {
        sum += pcm[i];
    }
    const int32_t dc = (int32_t)(sum / (int64_t)samples);
    st.dc = dc;

    for (size_t i = 0; i < samples; ++i) {
        int32_t v = (int32_t)pcm[i] - dc;
        /* Saturate rather than wrap. A wrapped sample flips sign at full scale,
         * which is an audible click AND destroys the clipped-count's meaning —
         * the one number that says the gain is too high. */
        if (v > 32767) {
            v = 32767;
            st.clipped++;
        } else if (v < -32768) {
            v = -32768;
            st.clipped++;
        }
        pcm[i] = (int16_t)v;

        const int32_t mag = (v < 0) ? -v : v;
        if (mag > st.peak) {
            st.peak = (int16_t)((mag > 32767) ? 32767 : mag);
        }
    }

    if (out_stats) {
        *out_stats = st;
    }
}
