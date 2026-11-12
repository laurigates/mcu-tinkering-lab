/**
 * @file voice_endpoint.c
 * @brief End-of-speech detection for a hands-free voice turn. See
 *        voice_endpoint.h for the rule and why begin() counts as speech.
 *
 * Pure C — test/test_voice_endpoint.c builds it on the host.
 */

#include "voice_endpoint.h"

#include <stddef.h>

voice_endpoint_cfg_t voice_endpoint_default_cfg(uint32_t max_ms)
{
    const voice_endpoint_cfg_t c = {
        .min_ms = VOICE_ENDPOINT_MIN_MS_DEFAULT,
        .max_ms = max_ms,
        .silence_ms = VOICE_ENDPOINT_SILENCE_MS_DEFAULT,
        .margin_db = VOICE_ENDPOINT_MARGIN_DB_DEFAULT,
    };
    return c;
}

bool voice_endpoint_is_speech(int16_t level_db, int16_t floor_db, uint8_t margin_db)
{
    /* 0 opts out of endpointing: every frame is speech, so only the ceiling
     * ends the turn — including frames below the floor, which the floor's fast
     * fall can produce. */
    if (margin_db == 0u) {
        return true;
    }
    return ((int32_t)level_db - (int32_t)floor_db) >= (int32_t)margin_db;
}

void voice_endpoint_begin(voice_endpoint_t *ep, const voice_endpoint_cfg_t *cfg, uint32_t now_ms)
{
    if (ep == NULL || cfg == NULL) {
        return;
    }
    ep->cfg = *cfg;
    ep->start_ms = now_ms;
    ep->last_speech_ms = now_ms; /* the trigger was speech */
    ep->speech_frames = 0u;
}

voice_endpoint_verdict_t voice_endpoint_update(voice_endpoint_t *ep, int16_t level_db,
                                               int16_t floor_db, uint32_t now_ms)
{
    if (ep == NULL) {
        return VOICE_ENDPOINT_END_MAX;
    }
    if (voice_endpoint_is_speech(level_db, floor_db, ep->cfg.margin_db)) {
        ep->last_speech_ms = now_ms;
        ep->speech_frames++;
    }

    /* Unsigned differences: correct across the uint32 wrap at day 49. */
    const uint32_t elapsed = now_ms - ep->start_ms;
    const uint32_t quiet = now_ms - ep->last_speech_ms;

    if (elapsed >= ep->cfg.max_ms) {
        return VOICE_ENDPOINT_END_MAX;
    }
    if (elapsed >= ep->cfg.min_ms && quiet >= ep->cfg.silence_ms) {
        return VOICE_ENDPOINT_END_SILENCE;
    }
    return VOICE_ENDPOINT_CONTINUE;
}

const char *voice_endpoint_verdict_name(voice_endpoint_verdict_t v)
{
    switch (v) {
        case VOICE_ENDPOINT_END_SILENCE:
            return "silence";
        case VOICE_ENDPOINT_END_MAX:
            return "max";
        case VOICE_ENDPOINT_CONTINUE:
        default:
            return "continue";
    }
}
