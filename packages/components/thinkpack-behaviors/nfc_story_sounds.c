/**
 * @file nfc_story_sounds.c
 * @brief Hand-rolled JSON parser + dispatcher for NFC story sound sequences.
 *
 * See nfc_story_sounds.h for the accepted grammar.  The parser walks the
 * input character-by-character, keeping a small cursor; no allocations and
 * no external JSON library.
 */

#include "nfc_story_sounds.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* Parser state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *buf;
    size_t len;
    size_t pos;
} parser_t;

static bool at_end(const parser_t *p)
{
    return p->pos >= p->len;
}

static void skip_ws(parser_t *p)
{
    while (!at_end(p)) {
        char c = p->buf[p->pos];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            p->pos++;
            continue;
        }
        break;
    }
}

static bool consume_char(parser_t *p, char expected)
{
    skip_ws(p);
    if (at_end(p) || p->buf[p->pos] != expected) {
        return false;
    }
    p->pos++;
    return true;
}

/* Match a double-quoted literal — e.g. consume_literal_string(p, "sequence")
 * matches `"sequence"` exactly.  Leading whitespace is skipped.  No escape
 * processing; we only compare ASCII runs. */
static bool consume_literal_string(parser_t *p, const char *literal)
{
    skip_ws(p);
    size_t lit_len = strlen(literal);
    if (p->len - p->pos < lit_len + 2) {
        return false;
    }
    if (p->buf[p->pos] != '"') {
        return false;
    }
    if (memcmp(p->buf + p->pos + 1, literal, lit_len) != 0) {
        return false;
    }
    if (p->buf[p->pos + 1 + lit_len] != '"') {
        return false;
    }
    p->pos += lit_len + 2;
    return true;
}

/* Read a quoted string into out_buf (NUL-terminated, at most out_cap bytes
 * including terminator).  No escape handling — rejects backslashes outright
 * to keep the parser strict and short. */
static bool consume_quoted_string(parser_t *p, char *out_buf, size_t out_cap)
{
    skip_ws(p);
    if (at_end(p) || p->buf[p->pos] != '"') {
        return false;
    }
    p->pos++;
    size_t written = 0;
    while (!at_end(p) && p->buf[p->pos] != '"') {
        char c = p->buf[p->pos];
        if (c == '\\') {
            return false;
        }
        if (written + 1 >= out_cap) {
            return false;
        }
        out_buf[written++] = c;
        p->pos++;
    }
    if (at_end(p)) {
        return false;
    }
    p->pos++; /* closing quote */
    out_buf[written] = '\0';
    return true;
}

/* Parse a signed decimal integer into *out.  Rejects overflow beyond int32. */
static bool consume_integer(parser_t *p, int32_t *out)
{
    skip_ws(p);
    if (at_end(p)) {
        return false;
    }
    size_t start = p->pos;
    int sign = 1;
    if (p->buf[p->pos] == '-') {
        sign = -1;
        p->pos++;
    } else if (p->buf[p->pos] == '+') {
        p->pos++;
    }
    if (at_end(p) || p->buf[p->pos] < '0' || p->buf[p->pos] > '9') {
        p->pos = start;
        return false;
    }
    int64_t acc = 0;
    while (!at_end(p) && p->buf[p->pos] >= '0' && p->buf[p->pos] <= '9') {
        acc = acc * 10 + (p->buf[p->pos] - '0');
        if (acc > (int64_t)2147483647LL + 1) {
            return false;
        }
        p->pos++;
    }
    acc *= sign;
    if (acc > 2147483647LL || acc < -2147483648LL) {
        return false;
    }
    *out = (int32_t)acc;
    return true;
}

/* ------------------------------------------------------------------ */
/* Higher-level parse helpers                                          */
/* ------------------------------------------------------------------ */

static bool kind_from_string(const char *s, uint8_t *out)
{
    if (strcmp(s, "tone") == 0) {
        *out = (uint8_t)NFC_STORY_KIND_TONE;
        return true;
    }
    if (strcmp(s, "clip") == 0) {
        *out = (uint8_t)NFC_STORY_KIND_CLIP;
        return true;
    }
    if (strcmp(s, "wait") == 0) {
        *out = (uint8_t)NFC_STORY_KIND_WAIT;
        return true;
    }
    return false;
}

static bool parse_step(parser_t *p, story_step_t *step)
{
    if (!consume_char(p, '{')) {
        return false;
    }

    bool have_kind = false;
    bool have_param = false;
    uint8_t kind = 0;
    int32_t param = 0;

    for (int field = 0; field < 2; field++) {
        if (field > 0 && !consume_char(p, ',')) {
            return false;
        }

        /* Key name — we accept "kind" or "param" in any order. */
        skip_ws(p);
        if (at_end(p) || p->buf[p->pos] != '"') {
            return false;
        }
        char key[16];
        if (!consume_quoted_string(p, key, sizeof(key))) {
            return false;
        }
        if (!consume_char(p, ':')) {
            return false;
        }

        if (strcmp(key, "kind") == 0) {
            char kind_str[16];
            if (!consume_quoted_string(p, kind_str, sizeof(kind_str))) {
                return false;
            }
            if (!kind_from_string(kind_str, &kind)) {
                return false;
            }
            have_kind = true;
        } else if (strcmp(key, "param") == 0) {
            if (!consume_integer(p, &param)) {
                return false;
            }
            have_param = true;
        } else {
            return false;
        }
    }

    if (!consume_char(p, '}')) {
        return false;
    }
    if (!have_kind || !have_param) {
        return false;
    }

    step->kind = kind;
    step->param = param;
    return true;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool nfc_story_sounds_parse(const char *json, size_t len, story_sequence_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    if (json == NULL || len == 0) {
        return false;
    }

    parser_t p = {.buf = json, .len = len, .pos = 0};

    if (!consume_char(&p, '{')) {
        return false;
    }
    if (!consume_literal_string(&p, "sequence")) {
        return false;
    }
    if (!consume_char(&p, ':')) {
        return false;
    }
    if (!consume_char(&p, '[')) {
        return false;
    }

    skip_ws(&p);
    if (!at_end(&p) && p.buf[p.pos] == ']') {
        /* Empty sequence — reject to keep the LLM contract simple. */
        memset(out, 0, sizeof(*out));
        return false;
    }

    uint8_t count = 0;
    for (;;) {
        if (count >= NFC_STORY_SOUNDS_MAX_STEPS) {
            memset(out, 0, sizeof(*out));
            return false;
        }
        story_step_t step;
        if (!parse_step(&p, &step)) {
            memset(out, 0, sizeof(*out));
            return false;
        }
        out->steps[count++] = step;

        skip_ws(&p);
        if (!at_end(&p) && p.buf[p.pos] == ',') {
            p.pos++;
            continue;
        }
        break;
    }

    if (!consume_char(&p, ']')) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    if (!consume_char(&p, '}')) {
        memset(out, 0, sizeof(*out));
        return false;
    }

    /* Trailing whitespace after the close-brace is permitted; anything
     * else indicates malformed input. */
    skip_ws(&p);
    if (!at_end(&p)) {
        memset(out, 0, sizeof(*out));
        return false;
    }

    out->count = count;
    return true;
}

int nfc_story_sounds_execute(const story_sequence_t *seq, audio_dispatch_fn dispatch,
                             void *user_ctx)
{
    if (seq == NULL || dispatch == NULL) {
        return -1;
    }
    for (uint8_t i = 0; i < seq->count; i++) {
        int rc = dispatch(seq->steps[i].kind, seq->steps[i].param, user_ctx);
        if (rc != 0) {
            return rc;
        }
    }
    return 0;
}
