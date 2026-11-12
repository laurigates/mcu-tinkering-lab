/**
 * @file dialogue_style.c
 * @brief Per-utterance variation. See the header for why this exists.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF — so test/test_dialogue_style.c
 * builds it on the host with no shims.
 */

#include "dialogue_style.h"

#include <string.h>

#include "speech_tags.h"

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

/* Ring of the last few whole lines, same discipline. */
static char s_recall[DIALOGUE_RECALL_SLOTS][DIALOGUE_RECALL_MAX];
static size_t s_recall_head;
static size_t s_recall_used;

void dialogue_style_reset_recent(void)
{
    memset(s_recent, 0, sizeof(s_recent));
    s_recent_head = 0;
    s_recent_used = 0;
    memset(s_recall, 0, sizeof(s_recall));
    s_recall_head = 0;
    s_recall_used = 0;
}

/** True for characters that end a clause and should not survive in an opening. */
static int is_trailing_punct(char c)
{
    return c == ',' || c == '.' || c == ';' || c == ':' || c == '!' || c == '?' || c == '-' ||
           c == ' ';
}

/**
 * Copy at most @p max_words words of @p line into @p out (0 = no word limit).
 *
 * Byte-oriented on purpose: the input is UTF-8 and the cap could land mid
 * sequence, so the copy stops at the last completed *word* rather than at the
 * byte cap whenever a boundary is available. A single word longer than the cap
 * is dropped entirely rather than cut into invalid UTF-8 — a mangled fragment
 * in the avoid-list, or in the recalled-lines list, would be worse than no
 * entry at all.
 */
static void extract_words(const char *line, char *out, size_t out_len, size_t max_words)
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

    while (line[i] != '\0' && written < cap && (max_words == 0u || words < max_words)) {
        if (line[i] == ' ') {
            last_boundary = written;
            ++words;
            if (max_words != 0u && words >= max_words) {
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
    /* Delivery tags are stripped before anything is extracted. A line beginning
     * "[sighs] Jaahas" would otherwise be remembered as opening with the tag,
     * and the next prompt would ask the model to avoid *sighing* rather than to
     * avoid the word "Jaahas" — the mechanism would quietly start policing the
     * wrong axis. The recall ring wants the same treatment for the same reason:
     * two identical sentences differing only in their tag are a repeat.
     * See speech_tags.h. */
    char words[DIALOGUE_STYLE_MAX];
    speech_tags_strip(line, words, sizeof(words));

    char opening[DIALOGUE_OPENING_MAX];
    extract_words(words, opening, sizeof(opening), DIALOGUE_OPENING_WORDS);
    if (opening[0] == '\0') {
        return;
    }

    /* The whole line, for the recall ring. Stored unconditionally — unlike the
     * openings ring below, a near-repeat here is exactly what the next
     * repetition check needs to see. */
    char whole[DIALOGUE_RECALL_MAX];
    extract_words(words, whole, sizeof(whole), 0u);
    if (whole[0] != '\0') {
        memcpy(s_recall[s_recall_head], whole, strlen(whole) + 1u);
        s_recall_head = (s_recall_head + 1u) % DIALOGUE_RECALL_SLOTS;
        if (s_recall_used < DIALOGUE_RECALL_SLOTS) {
            ++s_recall_used;
        }
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

/**
 * Render a ring as a quoted, comma-separated list, newest first.
 *
 * Newest first because a buffer too small for the whole ring should keep the
 * most recent entries — those are the ones a listener would notice being
 * repeated. An entry that does not fit whole is skipped rather than cut: half a
 * quoted string in a prompt reads as a mangled instruction (and could split a
 * UTF-8 sequence on its way into a JSON body).
 */
static size_t render_ring(const char *ring, size_t slots, size_t stride, size_t head, size_t used,
                          char *out, size_t out_len)
{
    if (out == NULL || out_len == 0u) {
        return 0;
    }
    out[0] = '\0';

    size_t written = 0;
    for (size_t n = 0; n < used; ++n) {
        const size_t idx = (head + slots - 1u - n) % slots;
        const char *item = ring + (idx * stride);
        if (item[0] == '\0') {
            continue;
        }

        const size_t item_len = strlen(item);
        const size_t need = (written > 0u ? 2u : 0u) + 2u + item_len;
        if (written + need >= out_len) {
            break;
        }
        if (written > 0u) {
            memcpy(out + written, ", ", 2u);
            written += 2u;
        }
        out[written++] = '"';
        memcpy(out + written, item, item_len);
        written += item_len;
        out[written++] = '"';
        out[written] = '\0';
    }
    return written;
}

size_t dialogue_style_recent_openings(char *out, size_t out_len)
{
    return render_ring(&s_recent[0][0], DIALOGUE_RECENT_SLOTS, DIALOGUE_OPENING_MAX, s_recent_head,
                       s_recent_used, out, out_len);
}

size_t dialogue_style_recent_lines(char *out, size_t out_len)
{
    return render_ring(&s_recall[0][0], DIALOGUE_RECALL_SLOTS, DIALOGUE_RECALL_MAX, s_recall_head,
                       s_recall_used, out, out_len);
}

/* -------------------------------------------------------------------------- */
/* Repetition check                                                            */
/* -------------------------------------------------------------------------- */

static uint8_t s_repeat_pct = DIALOGUE_REPEAT_PCT_DEFAULT;

void dialogue_style_set_repeat_pct(uint8_t pct)
{
    if (pct >= 1u && pct <= 100u) {
        s_repeat_pct = pct;
    }
}

uint8_t dialogue_style_repeat_pct(void)
{
    return s_repeat_pct;
}

/** Anything that separates words. Bytes >= 0x80 are word characters, so UTF-8
 *  sequences stay whole and are compared bytewise. */
static int is_word_sep(char c)
{
    const unsigned char u = (unsigned char)c;
    return u <= (unsigned char)' ' || u == ',' || u == '.' || u == ';' || u == ':' || u == '!' ||
           u == '?' || u == '-' || u == '"' || u == '\'' || u == '(' || u == ')';
}

/**
 * Advance @p cursor to the next word. @return its start, or NULL at end.
 */
static const char *next_word(const char **cursor, size_t *len)
{
    const char *p = *cursor;
    while (*p != '\0' && is_word_sep(*p)) {
        ++p;
    }
    if (*p == '\0') {
        *cursor = p;
        return NULL;
    }
    const char *start = p;
    while (*p != '\0' && !is_word_sep(*p)) {
        ++p;
    }
    *cursor = p;
    *len = (size_t)(p - start);
    return start;
}

/** ASCII-case-insensitive word equality. See the header for why only ASCII. */
static int word_eq(const char *a, size_t alen, const char *b, size_t blen)
{
    if (alen != blen) {
        return 0;
    }
    for (size_t i = 0; i < alen; ++i) {
        unsigned char ca = (unsigned char)a[i];
        unsigned char cb = (unsigned char)b[i];
        if (ca >= 'A' && ca <= 'Z') {
            ca = (unsigned char)(ca + 32);
        }
        if (cb >= 'A' && cb <= 'Z') {
            cb = (unsigned char)(cb + 32);
        }
        if (ca != cb) {
            return 0;
        }
    }
    return 1;
}

/** True when @p w occurs in @p s before @p limit (NULL = whole string). */
static int contains_word(const char *s, const char *limit, const char *w, size_t wlen)
{
    const char *cur = s;
    const char *tok = NULL;
    size_t tlen = 0;
    while ((tok = next_word(&cur, &tlen)) != NULL) {
        if (limit != NULL && tok >= limit) {
            break;
        }
        if (word_eq(tok, tlen, w, wlen)) {
            return 1;
        }
    }
    return 0;
}

unsigned dialogue_style_similarity(const char *a, const char *b)
{
    if (a == NULL || b == NULL) {
        return 0;
    }

    /* Sørensen–Dice over the two *sets* of words: 2|A∩B| / (|A|+|B|). Sets
     * rather than multisets so a filler word repeated inside one sentence does
     * not inflate the score. O(n*m) with n,m ≈ 20 — nothing worth indexing. */
    unsigned uniq_a = 0;
    unsigned uniq_b = 0;
    unsigned common = 0;

    const char *cur = a;
    const char *tok = NULL;
    size_t tlen = 0;
    while ((tok = next_word(&cur, &tlen)) != NULL) {
        if (contains_word(a, tok, tok, tlen)) {
            continue; /* already counted this word */
        }
        ++uniq_a;
        if (contains_word(b, NULL, tok, tlen)) {
            ++common;
        }
    }

    cur = b;
    while ((tok = next_word(&cur, &tlen)) != NULL) {
        if (!contains_word(b, tok, tok, tlen)) {
            ++uniq_b;
        }
    }

    if (uniq_a + uniq_b == 0u) {
        return 0;
    }
    return (unsigned)((200u * common) / (uniq_a + uniq_b));
}

bool dialogue_style_is_repetitive(const char *line)
{
    if (line == NULL || line[0] == '\0') {
        return false;
    }

    char words[DIALOGUE_STYLE_MAX];
    speech_tags_strip(line, words, sizeof(words));
    if (words[0] == '\0') {
        return false;
    }

    for (size_t n = 0; n < s_recall_used; ++n) {
        const size_t idx = (s_recall_head + DIALOGUE_RECALL_SLOTS - 1u - n) % DIALOGUE_RECALL_SLOTS;
        if (s_recall[idx][0] == '\0') {
            continue;
        }
        if (dialogue_style_similarity(words, s_recall[idx]) >= (unsigned)s_repeat_pct) {
            return true;
        }
    }
    return false;
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
