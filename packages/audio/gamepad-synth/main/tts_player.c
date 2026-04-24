/**
 * @file tts_player.c
 * @brief 24 kHz → 44.1 kHz linear-interp playback + overlay mixing.
 *
 * The source clips are spoken words (bandwidth well under 12 kHz), so linear
 * interpolation is audibly transparent — no anti-imaging filter needed.
 *
 * STEP = 24000 / 44100 ≈ 0.5442 source-samples per output-sample. We hold
 * `pos` (the fractional source index) as a float, take `idx = floor(pos)` and
 * `frac = pos - idx`, and interpolate between `samples[idx]` and
 * `samples[idx + 1]`. When `idx + 1` reaches `num_samples`, the clip is done.
 *
 * State lives in file scope rather than the caller so the audio task can
 * drive rendering without needing to pass the player-state struct across the
 * control/audio boundary.
 */

#include "tts_player.h"

#include <stddef.h>

#define TTS_SRC_RATE 24000
#define TTS_DST_RATE 44100
#define TTS_STEP ((float)TTS_SRC_RATE / (float)TTS_DST_RATE)

typedef struct {
    const int16_t *samples;
    int num_samples;
    float pos;
    bool playing;
} tts_state_t;

static volatile tts_state_t s_tts = {0};

static inline int16_t clip_i16(float x)
{
    if (x > 32767.0f)
        return 32767;
    if (x < -32768.0f)
        return -32768;
    return (int16_t)x;
}

void tts_player_start(const int16_t *samples, int num_samples)
{
    if (samples == NULL || num_samples <= 1) {
        s_tts.playing = false;
        return;
    }
    s_tts.samples = samples;
    s_tts.num_samples = num_samples;
    s_tts.pos = 0.0f;
    s_tts.playing = true;
}

bool tts_player_is_playing(void)
{
    return s_tts.playing;
}

void tts_player_render_block(int16_t *stereo_buf, int block_size)
{
    if (stereo_buf == NULL || block_size <= 0)
        return;
    if (!s_tts.playing)
        return;

    const int16_t *samples = s_tts.samples;
    int num_samples = s_tts.num_samples;
    float pos = s_tts.pos;

    for (int i = 0; i < block_size; i++) {
        int idx = (int)pos;
        if (idx + 1 >= num_samples) {
            s_tts.playing = false;
            break;
        }
        float frac = pos - (float)idx;
        float s0 = (float)samples[idx];
        float s1 = (float)samples[idx + 1];
        int16_t out = clip_i16(s0 + (s1 - s0) * frac);

        int idx_l = i * 2;
        int idx_r = idx_l + 1;
        stereo_buf[idx_l] = clip_i16((float)stereo_buf[idx_l] + (float)out);
        stereo_buf[idx_r] = clip_i16((float)stereo_buf[idx_r] + (float)out);

        pos += TTS_STEP;
    }

    s_tts.pos = pos;
}
