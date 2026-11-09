/**
 * @file note_gate.h
 * @brief When the planner may write a note to GlitchTip, and how many (ADR-033).
 *
 * The planner can call a side-channel tool, `note(kind, text)`, to tell the
 * firmware's developers about its own behaviour or a difficulty it ran into.
 * The note goes to GlitchTip's `robocar-ai` project through event_report, and
 * is never spoken.
 *
 * Like `speak`, the tool is withheld rather than discouraged
 * (.claude/rules/stateless-model-gating.md): it is declared only on a request
 * where a deterministic trigger has fired since the last offer, and the prompt
 * names that trigger, because the stateless model cannot know it otherwise.
 * Most requests carry neither the declaration nor the clause, so the feature
 * costs no prompt tokens while nothing is happening.
 *
 * Triggers are raised by planner_cycle_core.c from evidence the robot already
 * has (reflex count, BLIND, goal and speech streaks, a request count). This
 * module holds them, chooses which to offer, and rations the offers:
 *
 *   - at most one offer per NOTE_GATE_MIN_INTERVAL_MS (5 min);
 *   - at most NOTE_GATE_MAX_PER_BOOT (20) offers, after which the gate stays
 *     shut until `plan resume` or a reboot, like plan_budget's fuse.
 *
 * Offers are counted, not notes: an offer the model declines still carried the
 * declaration and the clause, which is the cost being bounded. Taking an offer
 * clears every pending trigger, because the clause has told the model what
 * happened; a trigger raised while the window is shut waits for it to open.
 *
 * Pure C, no ESP-IDF: host-tested in test/test_note_gate.c and compiled into
 * the behaviour twin (ADR-030).
 */

#ifndef NOTE_GATE_H
#define NOTE_GATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Shortest gap between two offers. */
#define NOTE_GATE_MIN_INTERVAL_MS (5u * 60u * 1000u)
/** Offers per boot (and per `plan resume`). */
#define NOTE_GATE_MAX_PER_BOOT 20u
/** Consecutive `track` goals that count as a track that is not converging. */
#define NOTE_GATE_TRACK_STUCK_CYCLES 4u
/** Consecutive spoken lines withheld as repeats before it is worth a note. */
#define NOTE_GATE_REPEAT_STREAK 3u
/** Requests between two periodic reflections (~10 min at the 15 s cadence). */
#define NOTE_GATE_REFLECT_EVERY 40u
/** Longest note text the firmware keeps, in bytes (NUL not counted). */
#define NOTE_TEXT_MAX 280u

/** Why the note tool was offered. Lower values win when several are pending. */
typedef enum {
    NOTE_TRIGGER_NONE = 0,
    NOTE_TRIGGER_REFLEX,      /**< the obstacle reflex braked since the last offer */
    NOTE_TRIGGER_BLIND,       /**< no camera frame has decoded */
    NOTE_TRIGGER_TRACK_STUCK, /**< `track` chosen NOTE_GATE_TRACK_STUCK_CYCLES times running */
    NOTE_TRIGGER_REPEATS,     /**< NOTE_GATE_REPEAT_STREAK lines withheld as repeats */
    NOTE_TRIGGER_REFLECT,     /**< NOTE_GATE_REFLECT_EVERY requests since the last one */
    NOTE_TRIGGER_COUNT,
} note_trigger_t;

/** What the model says the note is about. */
typedef enum {
    NOTE_KIND_NONE = 0,
    NOTE_KIND_DIFFICULTY,
    NOTE_KIND_BEHAVIOUR,
    NOTE_KIND_SUGGESTION,
    NOTE_KIND_COUNT,
} note_kind_t;

/** Clear everything: counters, window and pending triggers. Call at boot. */
void note_gate_init(void);

/** Mark @p trigger pending. NONE and out-of-range values are ignored. */
void note_gate_raise(note_trigger_t trigger);

/**
 * The trigger to offer on the request about to be made, or NONE.
 *
 * Returns the highest-priority pending trigger when the window is open and the
 * per-boot count is not spent; that charges one offer, starts a new window at
 * @p now_ms and clears every pending trigger. Otherwise returns NONE and leaves
 * the pending triggers where they are.
 */
note_trigger_t note_gate_take(uint32_t now_ms);

/**
 * Undo the last note_gate_take() that returned @p trigger: its request was
 * never sent (no WiFi link or no key), so the model never saw the offer. The
 * count and the window go back to what they were, and @p trigger is pending
 * again. Only the most recent take can be undone; a second call does nothing.
 */
void note_gate_untake(note_trigger_t trigger);

/** Count a note the model actually wrote (for the `plan` status line). */
void note_gate_noted(void);

/** `plan resume`: re-arm the per-boot count and the window. Pending
 *  triggers are kept. */
void note_gate_resume(void);

uint32_t note_gate_offers(void);
uint32_t note_gate_written(void);
/** True once NOTE_GATE_MAX_PER_BOOT offers have been made. */
bool note_gate_spent(void);
/** Bit (1 << trigger) for each pending trigger. */
uint32_t note_gate_pending(void);
/** Milliseconds until the window opens at @p now_ms; 0 when it is open. */
uint32_t note_gate_wait_ms(uint32_t now_ms);

/** Short name for logs, tags and the twin trace ("reflex", "blind", ...). */
const char *note_trigger_name(note_trigger_t trigger);

/**
 * The reason the planner prompt gives for offering the tool: only what the
 * robot measured, so the model has nothing to invent (stateless-model-gating.md
 * §4). Never NULL; "" for NONE.
 */
const char *note_trigger_reason(note_trigger_t trigger);

/** "difficulty", "behaviour", "suggestion", or "" for NONE. */
const char *note_kind_name(note_kind_t kind);

/** Inverse of note_kind_name(); NONE for anything else, including NULL. */
note_kind_t note_kind_from_name(const char *name);

/**
 * Copy @p text into @p out as one line of at most NOTE_TEXT_MAX bytes: control
 * characters become spaces, runs of spaces collapse, the ends are trimmed, and
 * a cut never splits a UTF-8 sequence. @p cap must be at least 1.
 *
 * @return the length written; 0 when nothing printable remains.
 */
size_t note_text_clean(char *out, size_t cap, const char *text);

#ifdef __cplusplus
}
#endif

#endif  // NOTE_GATE_H
