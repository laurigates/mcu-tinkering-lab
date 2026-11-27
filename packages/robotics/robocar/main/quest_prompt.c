/**
 * @file quest_prompt.c
 * @brief The planner's quest clause, its two tools, and applying the reply
 *        (ADR-039 §3). See quest_prompt.h.
 */

#include "quest_prompt.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Every sentence ends ". " so a clause cut between sentences still reads as
 * whole ones, and the planner prompt can append it with no separator. */
static const char k_discovery_fmt[] =
    "You may ALSO call 'log_discovery' once, IN ADDITION to a movement function, with one line "
    "on what you see that bears on the quest and what is near it. It is offered because %s. Do "
    "not restate a note above; if nothing here bears on the quest, do not call it. ";
static const char k_reason_view[] = "the view has changed since your last look in this direction";
static const char k_reason_track[] = "you chose to track something on your last turn";
static const char k_done[] =
    "You may ALSO call 'quest_done' once, IN ADDITION to a movement function: achieved=true when "
    "what you see shows the quest is done, achieved=false when it cannot be done from here, with "
    "a one-line outcome. Otherwise do not call it. ";

static const char *kind_hint(quest_kind_t kind)
{
    switch (kind) {
        case QUEST_KIND_EXPLORE:
            return "Look around, turning or tracking to see more. ";
        case QUEST_KIND_FIND:
            return "Look for it, turning or tracking to search. ";
        case QUEST_KIND_REVISIT:
            return "Look at it again and see whether anything has changed. ";
        case QUEST_KIND_ASK:
        case QUEST_KIND_COUNT:
        default:
            return "";
    }
}

/** snprintf-append, saturating. @return the new length, always < @p cap. */
__attribute__((format(printf, 4, 5))) static size_t append(char *buf, size_t cap, size_t pos,
                                                           const char *fmt, ...)
{
    if (pos + 1u >= cap) {
        return pos;
    }
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf + pos, cap - pos, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return pos;
    }
    return (pos + (size_t)n >= cap) ? (cap - 1u) : (pos + (size_t)n);
}

void quest_reply_clear(quest_reply_t *reply)
{
    memset(reply, 0, sizeof(*reply));
}

/** Indices of the newest @p max notes on @p quest_id that fit in @p room bytes
 *  of `"<text>"; ` each, oldest first. @return how many. */
static size_t pick_notes(const quest_log_t *log, uint16_t quest_id, size_t room, size_t *idx,
                         size_t max)
{
    size_t picked = 0;
    size_t used = 0;
    /* Walk newest to oldest; stop at the first that does not fit, so the notes
     * quoted are always the newest ones and never a gap. */
    for (size_t i = log->note_count; i > 0 && picked < max; i--) {
        const quest_note_t *n = &log->notes[i - 1];
        if (n->quest_id != quest_id) {
            continue;
        }
        const size_t need = strlen(n->text) + 4u; /* two quotes, "; " or ". " */
        if (used + need > room) {
            break;
        }
        used += need;
        idx[picked++] = i - 1;
    }
    /* Reverse into oldest first. */
    for (size_t a = 0, b = picked; a + 1 < b; a++, b--) {
        const size_t t = idx[a];
        idx[a] = idx[b - 1];
        idx[b - 1] = t;
    }
    return picked;
}

void quest_prompt_build(const quest_log_t *log, const quest_prompt_inputs_t *in,
                        quest_ration_t *ration, quest_offer_t *out)
{
    memset(out, 0, sizeof(*out));
    const quest_t *q = quest_log_active(log);
    if (q == NULL || q->kind == QUEST_KIND_ASK) {
        return;
    }

    if (ration->quest_id != q->id) {
        ration->quest_id = q->id;
        ration->requests = 0;
    }
    const bool has_note = quest_log_note_count(log, q->id) > 0;
    out->quest_id = q->id;
    out->offer_discovery = in->view_changed || in->tracked_last_cycle;
    out->offer_done =
        out->offer_discovery || has_note || ration->requests >= QUEST_DONE_AFTER_REQUESTS;
    ration->requests++;

    /* Head and tool sentences first, into their own buffers, so the notes are
     * sized against the room actually left. */
    char head[QUEST_LOG_TITLE_MAX + QUEST_LOG_WHY_MAX + 192];
    size_t hl = append(head, sizeof(head), 0, "You have an active quest, kind '%s': \"%s\". ",
                       quest_kind_name(q->kind), q->title);
    if (q->why[0] != '\0') {
        hl = append(head, sizeof(head), hl, "Why: %s. ", q->why);
    }
    hl = append(head, sizeof(head), hl, "%s", kind_hint(q->kind));

    char tools[sizeof(k_discovery_fmt) + sizeof(k_reason_view) + sizeof(k_done)];
    size_t tl = 0;
    tools[0] = '\0';
    if (out->offer_discovery) {
        tl = append(tools, sizeof(tools), tl, k_discovery_fmt,
                    in->view_changed ? k_reason_view : k_reason_track);
    }
    if (out->offer_done) {
        tl = append(tools, sizeof(tools), tl, "%s", k_done);
    }

    static const char k_notes_lead[] = "Your latest notes on it: ";
    const size_t fixed = hl + tl + (sizeof(k_notes_lead) - 1u);
    const size_t room = (fixed < QUEST_CLAUSE_MAX) ? (QUEST_CLAUSE_MAX - fixed) : 0u;
    size_t idx[QUEST_CLAUSE_NOTES];
    const size_t picked = pick_notes(log, q->id, room, idx, QUEST_CLAUSE_NOTES);

    const size_t cap = sizeof(out->clause);
    size_t pos = append(out->clause, cap, 0, "%s", head);
    if (picked > 0) {
        pos = append(out->clause, cap, pos, "%s", k_notes_lead);
        for (size_t i = 0; i < picked; i++) {
            pos = append(out->clause, cap, pos, "\"%s\"%s", log->notes[idx[i]].text,
                         (i + 1 < picked) ? "; " : ". ");
        }
    }
    (void)append(out->clause, cap, pos, "%s", tools);
}

bool quest_prompt_apply(quest_log_t *log, const quest_offer_t *offer, const quest_reply_t *reply,
                        char pose, quest_time_t now, quest_apply_result_t *res)
{
    memset(res, 0, sizeof(*res));
    res->note_err = QUEST_OK;

    const bool wants_note = reply->discovery[0] != '\0';
    const bool wants_close = reply->done;
    res->unoffered = (wants_note && !offer->offer_discovery) || (wants_close && !offer->offer_done);
    if (offer->quest_id == 0 || (!wants_note && !wants_close)) {
        return false;
    }

    const quest_t *active = quest_log_active(log);
    if (active == NULL || active->id != offer->quest_id) {
        res->stale = true;
        return false;
    }

    bool changed = false;
    if (wants_note && offer->offer_discovery) {
        res->note_err = quest_log_note(log, offer->quest_id, pose, reply->discovery, now);
        res->noted = (res->note_err == QUEST_OK);
        changed |= res->noted;
    }
    if (wants_close && offer->offer_done) {
        const quest_state_t as = reply->achieved ? QUEST_STATE_DONE : QUEST_STATE_DROPPED;
        if (quest_log_close(log, offer->quest_id, as, reply->outcome, now) == QUEST_OK) {
            res->closed = true;
            res->closed_as = as;
            changed = true;
        }
    }
    return changed;
}
