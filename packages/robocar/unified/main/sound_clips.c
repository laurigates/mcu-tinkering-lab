/**
 * @file sound_clips.c
 * @brief Embedded clip PCM (EMBED_FILES in CMakeLists.txt) and the ring write.
 */

#include "sound_clips.h"

#include <stddef.h>

#include "audio_player.h"
#include "pin_config.h"

/* EMBED_FILES "sounds/<tag>.pcm" emits _binary_<tag>_pcm_start/_end. */
#define CLIP_SYMBOLS(tag)                                                               \
    extern const uint8_t _binary_##tag##_pcm_start[] asm("_binary_" #tag "_pcm_start"); \
    extern const uint8_t _binary_##tag##_pcm_end[] asm("_binary_" #tag "_pcm_end")

CLIP_SYMBOLS(horn);
CLIP_SYMBOLS(aoogah);
CLIP_SYMBOLS(bell);
CLIP_SYMBOLS(boing);

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
} clip_pcm_t;

/* Indexed by name, so reordering sound_clip_t cannot hand a tag the wrong
 * clip (positional-writes-to-named-channels.md). */
static const clip_pcm_t s_clips[SOUND_CLIP_COUNT] = {
    [SOUND_CLIP_HORN] = {_binary_horn_pcm_start, _binary_horn_pcm_end},
    [SOUND_CLIP_AOOGAH] = {_binary_aoogah_pcm_start, _binary_aoogah_pcm_end},
    [SOUND_CLIP_BELL] = {_binary_bell_pcm_start, _binary_bell_pcm_end},
    [SOUND_CLIP_BOING] = {_binary_boing_pcm_start, _binary_boing_pcm_end},
};

esp_err_t sound_clips_write(sound_clip_t clip, uint32_t timeout_ms)
{
    if (clip <= SOUND_CLIP_NONE || clip >= SOUND_CLIP_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    const clip_pcm_t *c = &s_clips[clip];
    return audio_player_write(c->start, (size_t)(c->end - c->start), timeout_ms);
}

/* Written by the executor (Core 0), taken by the TTS task (Core 1). An atomic
 * exchange on take means a request written between a read and a clear cannot
 * be lost. */
static int s_request = SOUND_CLIP_NONE;

void sound_clips_request(sound_clip_t clip)
{
    if (clip <= SOUND_CLIP_NONE || clip >= SOUND_CLIP_COUNT) {
        return;
    }
    __atomic_store_n(&s_request, (int)clip, __ATOMIC_RELEASE);
}

sound_clip_t sound_clips_take_request(void)
{
    return (sound_clip_t)__atomic_exchange_n(&s_request, (int)SOUND_CLIP_NONE, __ATOMIC_ACQ_REL);
}

uint32_t sound_clips_duration_ms(sound_clip_t clip)
{
    if (clip <= SOUND_CLIP_NONE || clip >= SOUND_CLIP_COUNT) {
        return 0;
    }
    const size_t bytes = (size_t)(s_clips[clip].end - s_clips[clip].start);
    return (uint32_t)(bytes * 1000u / (AUDIO_SAMPLE_RATE_HZ * sizeof(int16_t)));
}
