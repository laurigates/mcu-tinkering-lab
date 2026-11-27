/**
 * @file quest_log_core.c
 * @brief Teuvo's quest log records, caps, pruning and file format (ADR-039 §1).
 */

#include "quest_log_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dialogue_style.h"

/* The longest line the parser accepts: every field at its cap plus the margin. */
#define QUEST_LOG_LINE_BUF \
    (QUEST_LOG_LINE_MARGIN + QUEST_LOG_TITLE_MAX + QUEST_LOG_WHY_MAX + QUEST_LOG_OUTCOME_MAX + 1)

/* Fields in the longest record (Q). */
#define QUEST_LOG_FIELDS_MAX 12

static const char *const k_kind_names[QUEST_KIND_COUNT] = {"explore", "find", "revisit", "ask"};
static const char *const k_state_names[QUEST_STATE_COUNT] = {"open", "active", "done", "dropped"};

void quest_log_init(quest_log_t *log)
{
    memset(log, 0, sizeof(*log));
    log->next_id = 1;
}

void quest_log_clear(quest_log_t *log)
{
    quest_log_init(log);
}

quest_time_t quest_time_from(int64_t epoch_s, int64_t uptime_s)
{
    quest_time_t t;
    if (epoch_s >= QUEST_LOG_EPOCH_VALID_S) {
        t.s = epoch_s;
        t.synced = true;
    } else {
        t.s = uptime_s;
        t.synced = false;
    }
    return t;
}

static bool is_continuation(unsigned char c)
{
    return (c & 0xC0u) == 0x80u;
}

size_t quest_log_clean_text(const char *in, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return 0;
    }
    out[0] = '\0';
    if (in == NULL) {
        return 0;
    }

    const size_t cap = out_len - 1;
    size_t len = 0;
    bool pending_space = false;
    const unsigned char *p = (const unsigned char *)in;

    for (; *p != '\0'; p++) {
        const unsigned char c = (*p < 0x20u || *p == 0x7Fu) ? (unsigned char)' ' : *p;
        if (c == ' ') {
            pending_space = (len > 0);
            continue;
        }
        const size_t need = (pending_space ? 1u : 0u) + 1u;
        if (len + need > cap) {
            /* Out of room. If this byte continues a character, the bytes of
             * that character already written are an incomplete sequence: take
             * them back, and the lead byte with them. */
            if (is_continuation(c)) {
                while (len > 0 && is_continuation((unsigned char)out[len - 1])) {
                    len--;
                }
                if (len > 0 && ((unsigned char)out[len - 1] & 0x80u) != 0) {
                    len--;
                }
            }
            break;
        }
        if (pending_space) {
            out[len++] = ' ';
            pending_space = false;
        }
        out[len++] = (char)c;
    }

    while (len > 0 && out[len - 1] == ' ') {
        len--;
    }
    out[len] = '\0';
    return len;
}

/* ---- Record bookkeeping ------------------------------------------------- */

static int quest_index(const quest_log_t *log, uint16_t id)
{
    if (id == 0) {
        return -1;
    }
    for (size_t i = 0; i < log->quest_count; i++) {
        if (log->quests[i].id == id) {
            return (int)i;
        }
    }
    return -1;
}

static bool state_is_open(quest_state_t s)
{
    return s == QUEST_STATE_OPEN || s == QUEST_STATE_ACTIVE;
}

static void remove_note_at(quest_log_t *log, size_t idx)
{
    memmove(&log->notes[idx], &log->notes[idx + 1],
            (log->note_count - idx - 1) * sizeof(log->notes[0]));
    log->note_count--;
    memset(&log->notes[log->note_count], 0, sizeof(log->notes[0]));
}

static void remove_quest_at(quest_log_t *log, size_t idx)
{
    const uint16_t id = log->quests[idx].id;
    for (size_t i = log->note_count; i > 0; i--) {
        if (log->notes[i - 1].quest_id == id) {
            remove_note_at(log, i - 1);
        }
    }
    memmove(&log->quests[idx], &log->quests[idx + 1],
            (log->quest_count - idx - 1) * sizeof(log->quests[0]));
    log->quest_count--;
    memset(&log->quests[log->quest_count], 0, sizeof(log->quests[0]));
}

/* Remove the oldest closed quest and its notes. @return false if none is closed. */
static bool prune_oldest_closed(quest_log_t *log)
{
    for (size_t i = 0; i < log->quest_count; i++) {
        if (!state_is_open(log->quests[i].state)) {
            remove_quest_at(log, i);
            return true;
        }
    }
    return false;
}

size_t quest_log_open_count(const quest_log_t *log)
{
    size_t n = 0;
    for (size_t i = 0; i < log->quest_count; i++) {
        if (state_is_open(log->quests[i].state)) {
            n++;
        }
    }
    return n;
}

size_t quest_log_note_count(const quest_log_t *log, uint16_t quest_id)
{
    size_t n = 0;
    for (size_t i = 0; i < log->note_count; i++) {
        if (log->notes[i].quest_id == quest_id) {
            n++;
        }
    }
    return n;
}

const quest_t *quest_log_find(const quest_log_t *log, uint16_t id)
{
    const int i = quest_index(log, id);
    return i < 0 ? NULL : &log->quests[i];
}

const quest_t *quest_log_active(const quest_log_t *log)
{
    for (size_t i = 0; i < log->quest_count; i++) {
        if (log->quests[i].state == QUEST_STATE_ACTIVE) {
            return &log->quests[i];
        }
    }
    return NULL;
}

/* Insert a quest record whose fields are already clean. Makes room by pruning
 * the oldest closed quest. @return the stored record, or NULL when full. */
static quest_t *append_quest(quest_log_t *log, const quest_t *q)
{
    if (state_is_open(q->state) && quest_log_open_count(log) >= QUEST_LOG_OPEN_MAX) {
        return NULL;
    }
    if (log->quest_count >= QUEST_LOG_QUESTS_MAX && !prune_oldest_closed(log)) {
        return NULL;
    }
    quest_t *slot = &log->quests[log->quest_count++];
    *slot = *q;
    return slot;
}

/* Insert a note whose fields are already clean, applying the per-quest and
 * total caps. */
static void append_note(quest_log_t *log, const quest_note_t *n)
{
    if (quest_log_note_count(log, n->quest_id) >= QUEST_LOG_NOTES_PER_QUEST) {
        for (size_t i = 0; i < log->note_count; i++) {
            if (log->notes[i].quest_id == n->quest_id) {
                remove_note_at(log, i);
                break;
            }
        }
    }
    if (log->note_count >= QUEST_LOG_NOTES_MAX && !prune_oldest_closed(log)) {
        remove_note_at(log, 0);
    }
    log->notes[log->note_count++] = *n;
}

/* ---- Mutations ----------------------------------------------------------- */

quest_err_t quest_log_open(quest_log_t *log, quest_kind_t kind, const char *title, const char *why,
                           quest_time_t now, uint16_t *id_out)
{
    if (log == NULL || (unsigned)kind >= QUEST_KIND_COUNT) {
        return QUEST_ERR_ARG;
    }
    quest_t q;
    memset(&q, 0, sizeof(q));
    if (quest_log_clean_text(title, q.title, sizeof(q.title)) == 0) {
        return QUEST_ERR_ARG;
    }
    quest_log_clean_text(why, q.why, sizeof(q.why));
    if (quest_log_open_count(log) >= QUEST_LOG_OPEN_MAX) {
        return QUEST_ERR_FULL;
    }

    /* 65535 quests later the ids wrap: skip 0 and any id still kept. */
    while (log->next_id == 0 || quest_index(log, log->next_id) >= 0) {
        log->next_id++;
    }
    q.id = log->next_id;
    q.kind = kind;
    q.state = QUEST_STATE_OPEN;
    q.opened = now;
    if (append_quest(log, &q) == NULL) {
        return QUEST_ERR_FULL;
    }
    log->next_id++;
    if (id_out != NULL) {
        *id_out = q.id;
    }
    return QUEST_OK;
}

quest_err_t quest_log_set_active(quest_log_t *log, uint16_t id)
{
    const int idx = quest_index(log, id);
    if (idx < 0) {
        return QUEST_ERR_NOT_FOUND;
    }
    if (!state_is_open(log->quests[idx].state)) {
        return QUEST_ERR_STATE;
    }
    for (size_t i = 0; i < log->quest_count; i++) {
        if (log->quests[i].state == QUEST_STATE_ACTIVE) {
            log->quests[i].state = QUEST_STATE_OPEN;
        }
    }
    log->quests[idx].state = QUEST_STATE_ACTIVE;
    return QUEST_OK;
}

quest_err_t quest_log_close(quest_log_t *log, uint16_t id, quest_state_t state, const char *outcome,
                            quest_time_t now)
{
    if (state != QUEST_STATE_DONE && state != QUEST_STATE_DROPPED) {
        return QUEST_ERR_ARG;
    }
    const int idx = quest_index(log, id);
    if (idx < 0) {
        return QUEST_ERR_NOT_FOUND;
    }
    quest_t *q = &log->quests[idx];
    if (!state_is_open(q->state)) {
        return QUEST_ERR_STATE;
    }
    q->state = state;
    q->closed = now;
    quest_log_clean_text(outcome, q->outcome, sizeof(q->outcome));
    return QUEST_OK;
}

static char clean_pose(char pose)
{
    return (pose == 'C' || pose == 'L' || pose == 'R' || pose == 'F') ? pose : '-';
}

quest_err_t quest_log_note(quest_log_t *log, uint16_t quest_id, char pose, const char *text,
                           quest_time_t now)
{
    const int idx = quest_index(log, quest_id);
    if (idx < 0) {
        return QUEST_ERR_NOT_FOUND;
    }
    if (!state_is_open(log->quests[idx].state)) {
        return QUEST_ERR_STATE;
    }
    quest_note_t n;
    memset(&n, 0, sizeof(n));
    if (quest_log_clean_text(text, n.text, sizeof(n.text)) == 0) {
        return QUEST_ERR_ARG;
    }
    for (size_t i = 0; i < log->note_count; i++) {
        if (log->notes[i].quest_id == quest_id &&
            dialogue_style_similarity(log->notes[i].text, n.text) >= QUEST_LOG_REPEAT_PCT) {
            return QUEST_ERR_REPEAT;
        }
    }
    n.quest_id = quest_id;
    n.at = now;
    n.pose = clean_pose(pose);
    append_note(log, &n);
    return QUEST_OK;
}

/* ---- Names --------------------------------------------------------------- */

const char *quest_kind_name(quest_kind_t kind)
{
    return (unsigned)kind < QUEST_KIND_COUNT ? k_kind_names[kind] : "?";
}

const char *quest_state_name(quest_state_t state)
{
    return (unsigned)state < QUEST_STATE_COUNT ? k_state_names[state] : "?";
}

bool quest_kind_from_name(const char *name, quest_kind_t *out)
{
    if (name == NULL) {
        return false;
    }
    for (int k = 0; k < QUEST_KIND_COUNT; k++) {
        if (strcmp(name, k_kind_names[k]) == 0) {
            if (out != NULL) {
                *out = (quest_kind_t)k;
            }
            return true;
        }
    }
    return false;
}

const char *quest_err_name(quest_err_t err)
{
    switch (err) {
        case QUEST_OK:
            return "ok";
        case QUEST_ERR_ARG:
            return "bad argument";
        case QUEST_ERR_FULL:
            return "too many open quests";
        case QUEST_ERR_NOT_FOUND:
            return "no such quest";
        case QUEST_ERR_STATE:
            return "quest is closed";
        case QUEST_ERR_REPEAT:
            return "near-repeat of an earlier note";
    }
    return "?";
}

/* ---- Serialise ----------------------------------------------------------- */

static void appendf(char *out, size_t out_len, size_t *pos, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n;
    if (*pos < out_len) {
        n = vsnprintf(out + *pos, out_len - *pos, fmt, ap);
    } else {
        n = vsnprintf(NULL, 0, fmt, ap);
    }
    va_end(ap);
    if (n > 0) {
        *pos += (size_t)n;
    }
}

size_t quest_log_serialize(const quest_log_t *log, char *out, size_t out_len)
{
    size_t pos = 0;
    const int v = QUEST_LOG_FORMAT_VERSION;

    appendf(out, out_len, &pos, "%d\tM\t%u\n", v, (unsigned)log->next_id);
    for (size_t i = 0; i < log->quest_count; i++) {
        const quest_t *q = &log->quests[i];
        appendf(out, out_len, &pos, "%d\tQ\t%u\t%d\t%d\t%lld\t%d\t%lld\t%d\t%s\t%s\t%s\n", v,
                (unsigned)q->id, (int)q->kind, (int)q->state, (long long)q->opened.s,
                q->opened.synced ? 1 : 0, (long long)q->closed.s, q->closed.synced ? 1 : 0,
                q->title, q->why, q->outcome);
    }
    for (size_t i = 0; i < log->note_count; i++) {
        const quest_note_t *n = &log->notes[i];
        appendf(out, out_len, &pos, "%d\tN\t%u\t%lld\t%d\t%c\t%s\n", v, (unsigned)n->quest_id,
                (long long)n->at.s, n->at.synced ? 1 : 0, n->pose, n->text);
    }
    if (pos >= out_len && out_len > 0) {
        out[0] = '\0';
    }
    return pos;
}

/* ---- Parse --------------------------------------------------------------- */

static bool parse_ll(const char *s, long long min, long long max, long long *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    char *end = NULL;
    const long long v = strtoll(s, &end, 10);
    if (end == NULL || *end != '\0' || v < min || v > max) {
        return false;
    }
    *out = v;
    return true;
}

static bool parse_flag(const char *s, bool *out)
{
    long long v;
    if (!parse_ll(s, 0, 1, &v)) {
        return false;
    }
    *out = (v == 1);
    return true;
}

/* Split @p line in place on tabs. Empty fields are kept (an empty `why`).
 * @return the field count, or 0 when there are more than @p max. */
static size_t split_tabs(char *line, char **fields, size_t max)
{
    size_t n = 0;
    char *p = line;
    for (;;) {
        if (n == max) {
            return 0;
        }
        fields[n++] = p;
        char *tab = strchr(p, '\t');
        if (tab == NULL) {
            return n;
        }
        *tab = '\0';
        p = tab + 1;
    }
}

static bool parse_quest(quest_log_t *log, char **f, size_t nf)
{
    if (nf != 12) {
        return false;
    }
    long long id, kind, state, opened, closed;
    quest_t q;
    memset(&q, 0, sizeof(q));
    if (!parse_ll(f[2], 1, UINT16_MAX, &id) || !parse_ll(f[3], 0, QUEST_KIND_COUNT - 1, &kind) ||
        !parse_ll(f[4], 0, QUEST_STATE_COUNT - 1, &state) ||
        !parse_ll(f[5], INT64_MIN, INT64_MAX, &opened) || !parse_flag(f[6], &q.opened.synced) ||
        !parse_ll(f[7], INT64_MIN, INT64_MAX, &closed) || !parse_flag(f[8], &q.closed.synced)) {
        return false;
    }
    if (quest_index(log, (uint16_t)id) >= 0 ||
        quest_log_clean_text(f[9], q.title, sizeof(q.title)) == 0) {
        return false;
    }
    q.id = (uint16_t)id;
    q.kind = (quest_kind_t)kind;
    q.state = (quest_state_t)state;
    if (q.state == QUEST_STATE_ACTIVE && quest_log_active(log) != NULL) {
        q.state = QUEST_STATE_OPEN; /* at most one active */
    }
    q.opened.s = opened;
    q.closed.s = closed;
    quest_log_clean_text(f[10], q.why, sizeof(q.why));
    quest_log_clean_text(f[11], q.outcome, sizeof(q.outcome));
    return append_quest(log, &q) != NULL;
}

static bool parse_note(quest_log_t *log, char **f, size_t nf)
{
    if (nf != 7) {
        return false;
    }
    long long qid, at;
    quest_note_t n;
    memset(&n, 0, sizeof(n));
    if (!parse_ll(f[2], 1, UINT16_MAX, &qid) || !parse_ll(f[3], INT64_MIN, INT64_MAX, &at) ||
        !parse_flag(f[4], &n.at.synced) || strlen(f[5]) != 1) {
        return false;
    }
    if (quest_index(log, (uint16_t)qid) < 0 ||
        quest_log_clean_text(f[6], n.text, sizeof(n.text)) == 0) {
        return false;
    }
    n.quest_id = (uint16_t)qid;
    n.at.s = at;
    n.pose = clean_pose(f[5][0]);
    append_note(log, &n);
    return true;
}

size_t quest_log_parse(quest_log_t *log, const char *buf, size_t len)
{
    quest_log_init(log);
    if (buf == NULL) {
        return 0;
    }

    size_t loaded = 0;
    long long stored_next = 1;
    char line[QUEST_LOG_LINE_BUF];
    char *fields[QUEST_LOG_FIELDS_MAX];
    size_t start = 0;

    while (start < len) {
        const char *nl = memchr(buf + start, '\n', len - start);
        if (nl == NULL) {
            break; /* last line incomplete */
        }
        const size_t line_len = (size_t)(nl - (buf + start));
        const char *src = buf + start;
        start += line_len + 1;

        if (line_len == 0 || line_len >= sizeof(line) || memchr(src, '\0', line_len) != NULL) {
            continue;
        }
        memcpy(line, src, line_len);
        line[line_len] = '\0';

        const size_t nf = split_tabs(line, fields, QUEST_LOG_FIELDS_MAX);
        long long version;
        if (nf < 2 || !parse_ll(fields[0], 0, 1000, &version) ||
            version != QUEST_LOG_FORMAT_VERSION) {
            continue;
        }
        if (strcmp(fields[1], "M") == 0) {
            long long next;
            if (nf == 3 && parse_ll(fields[2], 1, UINT16_MAX, &next)) {
                stored_next = next;
            }
        } else if (strcmp(fields[1], "Q") == 0) {
            loaded += parse_quest(log, fields, nf) ? 1u : 0u;
        } else if (strcmp(fields[1], "N") == 0) {
            loaded += parse_note(log, fields, nf) ? 1u : 0u;
        }
    }

    /* Never hand out an id that is still in the log, whatever M said. */
    long long next = stored_next;
    for (size_t i = 0; i < log->quest_count; i++) {
        if ((long long)log->quests[i].id >= next) {
            next = (long long)log->quests[i].id + 1;
        }
    }
    log->next_id = (next > UINT16_MAX) ? 1 : (uint16_t)next;
    return loaded;
}
