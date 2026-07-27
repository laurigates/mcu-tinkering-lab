/**
 * @file speech_tags.c
 * @brief Allow-list filter for inline TTS delivery tags. See speech_tags.h.
 */

#include "speech_tags.h"

#include <stdbool.h>
#include <string.h>

const char *const SPEECH_TAG_ALLOWED[] = {
    "[whispers]", "[sighs]", "[laughs]", "[excited]", "[bored]", "[gasp]",
};

const size_t SPEECH_TAG_ALLOWED_COUNT = sizeof(SPEECH_TAG_ALLOWED) / sizeof(SPEECH_TAG_ALLOWED[0]);

/**
 * Length of the bracketed run starting at @p p (which must point at '['),
 * including both brackets. Returns 0 when the run never closes — the caller
 * treats that as "remove to end of string".
 */
static size_t bracket_run_len(const char *p)
{
    const char *close = strchr(p, ']');
    return close ? (size_t)(close - p) + 1 : 0;
}

/** True when the @p len bytes at @p p are exactly one allowed tag. */
static bool is_allowed(const char *p, size_t len)
{
    for (size_t i = 0; i < SPEECH_TAG_ALLOWED_COUNT; i++) {
        const char *tag = SPEECH_TAG_ALLOWED[i];
        if (strlen(tag) == len && strncmp(p, tag, len) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * Shared scanner for both public entry points.
 *
 * Walks @p in copying to @p out, deciding per bracketed run whether to keep it.
 * @p keep_allowed selects the two behaviours: sanitize keeps recognised tags up
 * to the per-line cap, strip keeps none. Writing through a separate write cursor
 * (rather than memmove per removal) keeps this a single pass, which matters
 * because sanitize runs on the queue-post path.
 */
static size_t filter(const char *in, char *out, size_t out_len, bool keep_allowed, size_t *removed)
{
    size_t w = 0;
    size_t kept = 0;
    size_t drops = 0;

    if (out_len == 0) {
        return 0;
    }

    for (const char *r = in; *r != '\0';) {
        if (*r != '[') {
            if (w + 1 < out_len) {
                out[w++] = *r;
            }
            r++;
            continue;
        }

        const size_t run = bracket_run_len(r);
        if (run == 0) {
            /* Unclosed '[' — a truncated tag. Dropping the remainder is the
             * safe reading: the alternative is speaking "[whis". */
            drops++;
            break;
        }

        const bool keep = keep_allowed && kept < SPEECH_TAG_MAX_PER_LINE && is_allowed(r, run);
        if (keep) {
            for (size_t i = 0; i < run && w + 1 < out_len; i++) {
                out[w++] = r[i];
            }
            kept++;
        } else {
            drops++;
            /* Swallow one following space so "a [foo] b" does not become
             * "a  b". A space *before* the run is handled by the collapse
             * below, which is simpler than looking backwards here. */
            if (r[run] == ' ') {
                r++;
            }
        }
        r += run;
    }

    out[w] = '\0';

    /* Collapse any double space and trim, which is only reachable when a
     * removal sat between two spaces or at the start. */
    if (drops > 0) {
        size_t c = 0;
        bool prev_space = true; /* true so a leading space is dropped */
        for (size_t i = 0; i < w; i++) {
            const bool sp = (out[i] == ' ');
            if (sp && prev_space) {
                continue;
            }
            out[c++] = out[i];
            prev_space = sp;
        }
        while (c > 0 && out[c - 1] == ' ') {
            c--;
        }
        out[c] = '\0';
        w = c;
    }

    if (removed) {
        *removed = drops;
    }
    return w;
}

size_t speech_tags_sanitize(char *text)
{
    if (!text) {
        return 0;
    }

    /* Filtering only ever shortens or preserves length, but it is not safe to
     * write through the same pointer being read once a kept tag is copied
     * forward, so the scan runs into a scratch copy sized to the input. The
     * caller's buffer is SPEECH_TEXT_MAX (320 B); this lives on the posting
     * task's stack for the duration of one call. */
    char scratch[512];
    const size_t len = strlen(text);
    if (len >= sizeof(scratch)) {
        return 0; /* longer than any legitimate utterance — leave it alone */
    }

    size_t removed = 0;
    filter(text, scratch, sizeof(scratch), true, &removed);
    if (removed > 0 || strcmp(scratch, text) != 0) {
        memcpy(text, scratch, strlen(scratch) + 1);
    }
    return removed;
}

size_t speech_tags_strip(const char *in, char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return 0;
    }
    if (!in) {
        out[0] = '\0';
        return 0;
    }
    return filter(in, out, out_len, false, NULL);
}
