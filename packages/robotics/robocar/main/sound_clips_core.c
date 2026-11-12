/**
 * @file sound_clips_core.c
 * @brief Clip names, tag matching and line splitting. See sound_clips_core.h.
 */

#include "sound_clips_core.h"

#include <string.h>

const char *const SOUND_CLIP_NAMES[SOUND_CLIP_COUNT] = {
    [SOUND_CLIP_HORN] = "horn",
    [SOUND_CLIP_AOOGAH] = "aoogah",
    [SOUND_CLIP_BELL] = "bell",
    [SOUND_CLIP_BOING] = "boing",
};

const char *sound_clip_name(sound_clip_t clip)
{
    if (clip <= SOUND_CLIP_NONE || clip >= SOUND_CLIP_COUNT) {
        return NULL;
    }
    return SOUND_CLIP_NAMES[clip];
}

sound_clip_t sound_clip_from_name(const char *name)
{
    if (!name) {
        return SOUND_CLIP_NONE;
    }
    for (int i = 0; i < SOUND_CLIP_COUNT; i++) {
        if (strcmp(name, SOUND_CLIP_NAMES[i]) == 0) {
            return (sound_clip_t)i;
        }
    }
    return SOUND_CLIP_NONE;
}

sound_clip_t sound_clip_from_tag(const char *p, size_t len)
{
    if (!p || len < 3 || p[0] != '[' || p[len - 1] != ']') {
        return SOUND_CLIP_NONE;
    }
    for (int i = 0; i < SOUND_CLIP_COUNT; i++) {
        const size_t n = strlen(SOUND_CLIP_NAMES[i]);
        if (n == len - 2 && strncmp(p + 1, SOUND_CLIP_NAMES[i], n) == 0) {
            return (sound_clip_t)i;
        }
    }
    return SOUND_CLIP_NONE;
}

sound_split_t sound_clips_split(char *text)
{
    sound_split_t split = {SOUND_CLIP_NONE, SOUND_CLIP_NONE};
    if (!text) {
        return split;
    }

    /* One pass with a separate write cursor: removals only shorten the
     * string, so writing behind the read cursor is safe. */
    bool words_seen = false; /* anything other than space or a bracketed run */
    sound_clip_t first = SOUND_CLIP_NONE;
    bool first_leads = false;
    size_t w = 0;

    for (size_t r = 0; text[r] != '\0';) {
        if (text[r] == '[') {
            const char *close = strchr(text + r, ']');
            if (close) {
                const size_t run = (size_t)(close - (text + r)) + 1;
                const sound_clip_t clip = sound_clip_from_tag(text + r, run);
                if (clip != SOUND_CLIP_NONE) {
                    if (first == SOUND_CLIP_NONE) {
                        first = clip;
                        first_leads = !words_seen;
                    }
                    r += run; /* dropped; the collapse below tidies the gap */
                    continue;
                }
                /* A delivery tag: kept, and not words. */
                memmove(text + w, text + r, run);
                w += run;
                r += run;
                continue;
            }
        }
        if (text[r] != ' ') {
            words_seen = true;
        }
        text[w++] = text[r++];
    }
    text[w] = '\0';

    /* Collapse runs of spaces and trim both ends. */
    size_t c = 0;
    bool prev_space = true; /* drops leading spaces */
    for (size_t i = 0; i < w; i++) {
        const bool sp = (text[i] == ' ');
        if (sp && prev_space) {
            continue;
        }
        text[c++] = text[i];
        prev_space = sp;
    }
    while (c > 0 && text[c - 1] == ' ') {
        c--;
    }
    text[c] = '\0';

    if (first != SOUND_CLIP_NONE) {
        if (first_leads) {
            split.lead = first;
        } else {
            split.trail = first;
        }
    }
    return split;
}

bool sound_clips_reflex_due(uint32_t now_ms, uint32_t last_ms, bool honked, bool blocking)
{
    if (!blocking) {
        return false;
    }
    if (!honked) {
        return true;
    }
    /* Unsigned subtraction is the elapsed time even across the wrap. */
    return (uint32_t)(now_ms - last_ms) >= SOUND_REFLEX_COOLDOWN_MS;
}
