/**
 * @file note_gate.c
 * @brief Planner note gate. See note_gate.h.
 */

#include "note_gate.h"

#include <string.h>

static uint32_t s_pending;
static uint32_t s_offers;
static uint32_t s_written;
static bool s_have_offer;
static uint32_t s_last_offer_ms;

/* The window as it was before the last take, for note_gate_untake(). */
static bool s_can_untake;
static bool s_prev_have_offer;
static uint32_t s_prev_last_offer_ms;

void note_gate_init(void)
{
    s_pending = 0u;
    s_offers = 0u;
    s_written = 0u;
    s_have_offer = false;
    s_last_offer_ms = 0u;
    s_can_untake = false;
}

void note_gate_raise(note_trigger_t trigger)
{
    if (trigger <= NOTE_TRIGGER_NONE || trigger >= NOTE_TRIGGER_COUNT) {
        return;
    }
    s_pending |= 1u << (unsigned)trigger;
}

uint32_t note_gate_wait_ms(uint32_t now_ms)
{
    if (!s_have_offer) {
        return 0u;
    }
    /* Unsigned difference: correct across the day-49 millisecond wrap. */
    const uint32_t since = now_ms - s_last_offer_ms;
    return since >= NOTE_GATE_MIN_INTERVAL_MS ? 0u : NOTE_GATE_MIN_INTERVAL_MS - since;
}

bool note_gate_spent(void)
{
    return s_offers >= NOTE_GATE_MAX_PER_BOOT;
}

note_trigger_t note_gate_take(uint32_t now_ms)
{
    if (s_pending == 0u || note_gate_spent() || note_gate_wait_ms(now_ms) > 0u) {
        return NOTE_TRIGGER_NONE;
    }
    note_trigger_t chosen = NOTE_TRIGGER_NONE;
    for (int t = NOTE_TRIGGER_NONE + 1; t < NOTE_TRIGGER_COUNT; t++) {
        if (s_pending & (1u << (unsigned)t)) {
            chosen = (note_trigger_t)t;
            break;
        }
    }
    s_prev_have_offer = s_have_offer;
    s_prev_last_offer_ms = s_last_offer_ms;
    s_can_untake = true;
    s_pending = 0u;
    s_offers++;
    s_have_offer = true;
    s_last_offer_ms = now_ms;
    return chosen;
}

void note_gate_untake(note_trigger_t trigger)
{
    if (!s_can_untake || trigger <= NOTE_TRIGGER_NONE || trigger >= NOTE_TRIGGER_COUNT) {
        return;
    }
    s_can_untake = false;
    if (s_offers > 0u) {
        s_offers--;
    }
    s_have_offer = s_prev_have_offer;
    s_last_offer_ms = s_prev_last_offer_ms;
    note_gate_raise(trigger);
}

void note_gate_noted(void)
{
    if (s_written < UINT32_MAX) {
        s_written++;
    }
}

void note_gate_resume(void)
{
    s_offers = 0u;
    s_written = 0u;
    s_have_offer = false;
    s_can_untake = false;
}

uint32_t note_gate_offers(void)
{
    return s_offers;
}

uint32_t note_gate_written(void)
{
    return s_written;
}

uint32_t note_gate_pending(void)
{
    return s_pending;
}

const char *note_trigger_name(note_trigger_t trigger)
{
    switch (trigger) {
        case NOTE_TRIGGER_REFLEX:
            return "reflex";
        case NOTE_TRIGGER_BLIND:
            return "blind";
        case NOTE_TRIGGER_TRACK_STUCK:
            return "track_stuck";
        case NOTE_TRIGGER_REPEATS:
            return "repeats";
        case NOTE_TRIGGER_REFLECT:
            return "reflect";
        default:
            return "none";
    }
}

const char *note_trigger_reason(note_trigger_t trigger)
{
    switch (trigger) {
        case NOTE_TRIGGER_REFLEX:
            return "the robot's obstacle reflex braked it since your last note, because its "
                   "front rangefinder measured something very close ahead";
        case NOTE_TRIGGER_BLIND:
            return "no camera frame has decoded on the robot since it started, so the image "
                   "you are given may be blank or corrupt";
        case NOTE_TRIGGER_TRACK_STUCK:
            return "you have chosen 'track' on each of your last few plans, so whatever you are "
                   "tracking has not been reached";
        case NOTE_TRIGGER_REPEATS:
            return "several of your recent spoken lines were withheld because they repeated "
                   "things you had already said";
        case NOTE_TRIGGER_REFLECT:
            return "it has been a while since your last note, and this is a periodic chance to "
                   "reflect on how the robot is doing";
        default:
            return "";
    }
}

static const char *const k_kind_names[NOTE_KIND_COUNT] = {
    [NOTE_KIND_NONE] = "",
    [NOTE_KIND_DIFFICULTY] = "difficulty",
    [NOTE_KIND_BEHAVIOUR] = "behaviour",
    [NOTE_KIND_SUGGESTION] = "suggestion",
};

const char *note_kind_name(note_kind_t kind)
{
    if (kind <= NOTE_KIND_NONE || kind >= NOTE_KIND_COUNT) {
        return "";
    }
    return k_kind_names[kind];
}

note_kind_t note_kind_from_name(const char *name)
{
    if (name == NULL) {
        return NOTE_KIND_NONE;
    }
    for (int k = NOTE_KIND_NONE + 1; k < NOTE_KIND_COUNT; k++) {
        if (strcmp(name, k_kind_names[k]) == 0) {
            return (note_kind_t)k;
        }
    }
    return NOTE_KIND_NONE;
}

size_t note_text_clean(char *out, size_t cap, const char *text)
{
    if (out == NULL || cap == 0u) {
        return 0u;
    }
    out[0] = '\0';
    if (text == NULL) {
        return 0u;
    }
    const size_t limit = (cap - 1u < NOTE_TEXT_MAX) ? cap - 1u : NOTE_TEXT_MAX;
    size_t o = 0u;
    bool pending_space = false;
    for (const unsigned char *p = (const unsigned char *)text; *p != '\0'; p++) {
        const unsigned char c = *p;
        if (c < 0x20u || c == 0x7fu || c == ' ') {
            pending_space = (o > 0u); /* never a leading space */
            continue;
        }
        const size_t need = (pending_space ? 1u : 0u) + 1u;
        if (o + need > limit) {
            break;
        }
        if (pending_space) {
            out[o++] = ' ';
            pending_space = false;
        }
        out[o++] = (char)c;
    }
    /* A cut may have landed inside a multi-byte sequence: drop the partial
     * sequence, so the JSON on its way to GlitchTip stays valid UTF-8. */
    if (o > 0u && ((unsigned char)out[o - 1u] & 0x80u) != 0u) {
        size_t start = o - 1u;
        while (start > 0u && ((unsigned char)out[start] & 0xC0u) == 0x80u) {
            start--;
        }
        const unsigned char lead = (unsigned char)out[start];
        const size_t want = (lead & 0xE0u) == 0xC0u   ? 2u
                            : (lead & 0xF0u) == 0xE0u ? 3u
                            : (lead & 0xF8u) == 0xF0u ? 4u
                                                      : 1u;
        if (o - start < want) {
            o = start;
        }
    }
    while (o > 0u && out[o - 1u] == ' ') {
        o--;
    }
    out[o] = '\0';
    return o;
}
