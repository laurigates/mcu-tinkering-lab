/**
 * @file twin_signals.c
 * @brief Deterministic synthetic sensor data. See twin_signals.h.
 */

#include "twin_signals.h"

#include <math.h>

static uint32_t fnv1a(const char *s)
{
    uint32_t h = 2166136261u;
    for (; s && *s; s++) {
        h ^= (uint8_t)*s;
        h *= 16777619u;
    }
    return h;
}

static uint32_t xorshift(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static uint32_t nonzero(uint32_t s)
{
    return s != 0u ? s : 0x9E3779B9u; /* xorshift is absorbing at 0 */
}

void twin_scene_luma(const char *scene, uint8_t luma[TWIN_THUMB_W * TWIN_THUMB_H])
{
    enum { BX = 8, BY = 6, BW = TWIN_THUMB_W / BX, BH = TWIN_THUMB_H / BY };
    uint32_t st = nonzero(fnv1a(scene));
    uint8_t block[BX * BY];
    for (int i = 0; i < BX * BY; i++) {
        block[i] = (uint8_t)(40u + xorshift(&st) % 176u);
    }
    for (int y = 0; y < TWIN_THUMB_H; y++) {
        for (int x = 0; x < TWIN_THUMB_W; x++) {
            luma[y * TWIN_THUMB_W + x] = block[(y / BH) * BX + (x / BW)];
        }
    }
}

/** Uniform noise in [-a, a]; RMS a / sqrt(3). */
static void noise(uint32_t *st, double rms, int16_t *pcm, size_t n, int add)
{
    const double a = rms * sqrt(3.0);
    for (size_t i = 0; i < n; i++) {
        const double u = (double)xorshift(st) / 4294967295.0; /* [0, 1] */
        double v = (2.0 * u - 1.0) * a + (add ? (double)pcm[i] : 0.0);
        if (v > 32767.0) {
            v = 32767.0;
        } else if (v < -32768.0) {
            v = -32768.0;
        }
        pcm[i] = (int16_t)lrint(v);
    }
}

void twin_pcm_room(uint32_t seed, int16_t *pcm, size_t n)
{
    uint32_t st = nonzero(seed ^ 0xA5A5F00Du);
    noise(&st, TWIN_ROOM_RMS_LSB, pcm, n, 0);
}

void twin_pcm_burst(uint32_t seed, uint32_t frame_index, int peak_db, int16_t *pcm, size_t n)
{
    twin_pcm_room(seed, pcm, n);
    uint32_t st = nonzero(seed * 2654435761u ^ (frame_index + 1u) * 40503u);
    noise(&st, TWIN_ROOM_RMS_LSB * pow(10.0, (double)peak_db / 20.0), pcm, n, 1);
}
