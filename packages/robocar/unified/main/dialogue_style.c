/**
 * @file dialogue_style.c
 * @brief Per-utterance variation. See the header for why this exists.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_dialogue_style.c
 * builds it on the host with no shims.
 */

#include "dialogue_style.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* PRNG                                                                        */
/* -------------------------------------------------------------------------- */

/* xorshift32: a handful of instructions and no state beyond one word. Nothing
 * here is security-relevant — it only has to keep the robot from sounding like
 * a tape loop, and a fixed default seed keeps host tests reproducible. */
static uint32_t s_rng = 0x2545F491u;

void dialogue_style_seed(uint32_t seed)
{
    s_rng = (seed != 0u) ? seed : 0x2545F491u; /* xorshift is absorbing at 0 */
}

static uint32_t next_rand(void)
{
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = x;
    return x;
}

/* -------------------------------------------------------------------------- */
/* Draw state                                                                  */
/* -------------------------------------------------------------------------- */

#define SLOT_NONE 0xFFu

static uint8_t s_last_pick[DIALOGUE_SLOT_COUNT] = {SLOT_NONE, SLOT_NONE, SLOT_NONE};

const char *dialogue_style_pick(const dialogue_pool_t *pool, dialogue_slot_t slot)
{
    if (pool == NULL || pool->items == NULL || pool->count == 0u) {
        return NULL;
    }
    if (pool->count == 1u) {
        return pool->items[0];
    }

    size_t idx = (size_t)(next_rand() % (uint32_t)pool->count);

    if (slot < DIALOGUE_SLOT_COUNT && pool->count <= SLOT_NONE) {
        if ((size_t)s_last_pick[slot] == idx) {
            /* Re-draw uniformly among the other count-1 entries. Shifting by a
             * fixed +1 instead would make the sequence a predictable cycle,
             * which is the failure mode this module exists to avoid. */
            idx = (idx + 1u + (size_t)(next_rand() % (uint32_t)(pool->count - 1u))) % pool->count;
        }
        s_last_pick[slot] = (uint8_t)idx;
    }

    return pool->items[idx];
}

/* -------------------------------------------------------------------------- */
/* Recently spoken openings                                                    */
/* -------------------------------------------------------------------------- */

/* Ring of the last few openings, newest at (head - 1). Every slot keeps its
 * final byte NUL — see the concurrency note in the header: that invariant is
 * what makes an unlocked read from another task bounded and terminated. */
static char s_recent[DIALOGUE_RECENT_SLOTS][DIALOGUE_OPENING_MAX];
static size_t s_recent_head;
static size_t s_recent_used;

void dialogue_style_reset_recent(void)
{
    memset(s_recent, 0, sizeof(s_recent));
    s_recent_head = 0;
    s_recent_used = 0;
}

/** True for characters that end a clause and should not survive in an opening. */
static int is_trailing_punct(char c)
{
    return c == ',' || c == '.' || c == ';' || c == ':' || c == '!' || c == '?' || c == '-' ||
           c == ' ';
}

/**
 * Copy the first DIALOGUE_OPENING_WORDS words of @p line into @p out.
 *
 * Byte-oriented on purpose: the input is UTF-8 and the cap could land mid
 * sequence, so the copy stops at the last completed *word* rather than at the
 * byte cap whenever a boundary is available. A single word longer than the cap
 * is dropped entirely rather than cut into invalid UTF-8 — a mangled fragment
 * in the avoid-list would be worse than no entry at all.
 */
static void extract_opening(const char *line, char *out, size_t out_len)
{
    out[0] = '\0';
    if (line == NULL || out_len < 2u) {
        return;
    }

    const size_t cap = out_len - 1u; /* never write the final byte: see above */
    size_t i = 0;
    while (line[i] == ' ') {
        ++i;
    }

    size_t words = 0;
    size_t written = 0;
    size_t last_boundary = 0; /* length of the longest prefix ending a word */

    while (line[i] != '\0' && written < cap && words < DIALOGUE_OPENING_WORDS) {
        if (line[i] == ' ') {
            last_boundary = written;
            ++words;
            if (words >= DIALOGUE_OPENING_WORDS) {
                break;
            }
            out[written++] = ' ';
            while (line[i] == ' ') {
                ++i;
            }
            continue;
        }
        out[written++] = line[i++];
    }

    /* Ran out of room mid-word: fall back to the last complete word. (Stopping
     * on '\0' or on a space means what was copied already ends a word.) */
    if (written >= cap && line[i] != '\0' && line[i] != ' ') {
        written = last_boundary;
    }

    while (written > 0u && is_trailing_punct(out[written - 1u])) {
        --written;
    }
    out[written] = '\0';
}

void dialogue_style_note_spoken(const char *line)
{
    char opening[DIALOGUE_OPENING_MAX];
    extract_opening(line, opening, sizeof(opening));
    if (opening[0] == '\0') {
        return;
    }

    /* A phrase the model keeps returning to would otherwise fill every slot and
     * evict the older openings that still need avoiding. */
    if (s_recent_used > 0u) {
        const size_t newest = (s_recent_head + DIALOGUE_RECENT_SLOTS - 1u) % DIALOGUE_RECENT_SLOTS;
        if (strcmp(s_recent[newest], opening) == 0) {
            return;
        }
    }

    memcpy(s_recent[s_recent_head], opening, strlen(opening) + 1u);
    s_recent_head = (s_recent_head + 1u) % DIALOGUE_RECENT_SLOTS;
    if (s_recent_used < DIALOGUE_RECENT_SLOTS) {
        ++s_recent_used;
    }
}

size_t dialogue_style_recent_openings(char *out, size_t out_len)
{
    if (out == NULL || out_len == 0u) {
        return 0;
    }
    out[0] = '\0';

    size_t written = 0;
    for (size_t n = 0; n < s_recent_used; ++n) {
        /* Newest first: the most recent opening is the one most worth avoiding
         * if the list has to be truncated to fit. */
        const size_t idx = (s_recent_head + DIALOGUE_RECENT_SLOTS - 1u - n) % DIALOGUE_RECENT_SLOTS;
        const char *item = s_recent[idx];
        if (item[0] == '\0') {
            continue;
        }

        const size_t need = (written > 0u ? 2u : 0u) + 2u + strlen(item);
        if (written + need >= out_len) {
            break;
        }
        if (written > 0u) {
            memcpy(out + written, ", ", 2u);
            written += 2u;
        }
        out[written++] = '"';
        memcpy(out + written, item, strlen(item));
        written += strlen(item);
        out[written++] = '"';
        out[written] = '\0';
    }
    return written;
}

/* -------------------------------------------------------------------------- */
/* Composed directive                                                          */
/* -------------------------------------------------------------------------- */

/** Append @p s to @p out, returning the new length. No-op once full. */
static size_t append(char *out, size_t out_len, size_t written, const char *s)
{
    if (s == NULL || s[0] == '\0') {
        return written;
    }
    const size_t n = strlen(s);
    if (written + n >= out_len) {
        return written; /* leave the buffer as-is rather than half-append */
    }
    memcpy(out + written, s, n + 1u);
    return written + n;
}

size_t dialogue_style_directive(const dialogue_pool_t *openers, const dialogue_pool_t *shapes,
                                const char *avoid_lead, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0u) {
        return 0;
    }
    out[0] = '\0';

    size_t written = 0;
    const char *opener = dialogue_style_pick(openers, DIALOGUE_SLOT_OPENER);
    const char *shape = dialogue_style_pick(shapes, DIALOGUE_SLOT_SHAPE);

    written = append(out, out_len, written, opener);
    if (shape != NULL && shape[0] != '\0') {
        if (written > 0u) {
            written = append(out, out_len, written, " ");
        }
        written = append(out, out_len, written, shape);
    }

    if (avoid_lead != NULL && avoid_lead[0] != '\0') {
        char recent[DIALOGUE_RECENT_SLOTS * (DIALOGUE_OPENING_MAX + 4u)];
        if (dialogue_style_recent_openings(recent, sizeof(recent)) > 0u) {
            const size_t before = written;
            if (written > 0u) {
                written = append(out, out_len, written, " ");
            }
            written = append(out, out_len, written, avoid_lead);
            written = append(out, out_len, written, " ");
            written = append(out, out_len, written, recent);
            written = append(out, out_len, written, ".");
            /* All-or-nothing: a clause cut off after "Do not begin with:" reads
             * as a broken instruction, so drop it wholesale if it did not fit.
             * The closing full stop is the last thing appended, so its presence
             * is exactly the "every part fitted" test. */
            if (written == before || out[written - 1u] != '.') {
                written = before;
                out[written] = '\0';
            }
        }
    }

    return written;
}
