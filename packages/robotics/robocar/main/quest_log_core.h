/**
 * @file quest_log_core.h
 * @brief Teuvo's quest log: the records, caps, pruning and file format (ADR-039 §1).
 *
 * Pure C with no ESP-IDF dependency, so the host suite and the behaviour twin
 * run the same code the firmware does. The caller owns the clock and the
 * storage: every mutation takes a `quest_time_t`, and the log is turned into
 * a byte buffer (quest_log_serialize()) and back (quest_log_parse()) rather
 * than touching a file. `quest_log.c` is the firmware glue (mutex, SPIFFS,
 * save task).
 *
 * ## File format
 *
 * One record per line, tab-separated, every line starting with the format
 * version. Text fields never hold a tab or a newline: every string is cleaned
 * on the way in (quest_log_clean_text()), so no escaping is needed and a line
 * is always one record.
 *
 *     1  M  <next_id>
 *     1  Q  <id>  <kind>  <state>  <opened_s>  <opened_synced>  <closed_s>
 *           <closed_synced>  <title>  <why>  <outcome>
 *     1  N  <quest_id>  <at_s>  <at_synced>  <pose>  <text>
 *
 * The file is written whole and renamed into place, so a reader normally sees
 * a complete file. quest_log_parse() still assumes it may not: a last line
 * without its newline, a line from another version and a malformed line are
 * skipped, and the parse never fails as a whole (`test_quest_log_core.c` cuts
 * a log at every byte offset).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Format version written at the start of every line. A line with any other
 *  version is skipped, so a downgrade reads what it understands. */
#define QUEST_LOG_FORMAT_VERSION 1

/** Text caps in bytes, excluding the NUL. Cut at a UTF-8 character boundary. */
#define QUEST_LOG_TITLE_MAX 60
#define QUEST_LOG_WHY_MAX 120
#define QUEST_LOG_OUTCOME_MAX 120
#define QUEST_LOG_NOTE_MAX 160

/** At most this many quests are open or active at once (ADR-039 §1). */
#define QUEST_LOG_OPEN_MAX 8
/** Quest records kept, open and closed. Past this the oldest closed quest and
 *  its notes are pruned. */
#define QUEST_LOG_QUESTS_MAX 24
/** Notes kept per quest; a new note past this drops the quest's oldest. */
#define QUEST_LOG_NOTES_PER_QUEST 12
/** Notes kept in total. A new note past this prunes the oldest closed quest
 *  first, then the oldest note of any quest. Sized so the open quests alone
 *  never need more than this. */
#define QUEST_LOG_NOTES_MAX (QUEST_LOG_OPEN_MAX * QUEST_LOG_NOTES_PER_QUEST)

/** A note at or above this Sørensen–Dice word similarity (percent) to one of
 *  its quest's notes is a near-repeat and is dropped (ADR-039 §3). */
#define QUEST_LOG_REPEAT_PCT 70

/** Wall time before this (2026-01-01T00:00:00Z) means the clock has not synced. */
#define QUEST_LOG_EPOCH_VALID_S 1767225600LL

/** The largest file quest_log_serialize() can produce, for sizing its buffer.
 *  Each line is bounded by its fields' caps plus a margin for the numbers. */
#define QUEST_LOG_LINE_MARGIN 96
#define QUEST_LOG_SERIALIZED_MAX                                                               \
    (QUEST_LOG_LINE_MARGIN +                                                                   \
     QUEST_LOG_QUESTS_MAX * (QUEST_LOG_LINE_MARGIN + QUEST_LOG_TITLE_MAX + QUEST_LOG_WHY_MAX + \
                             QUEST_LOG_OUTCOME_MAX) +                                          \
     QUEST_LOG_NOTES_MAX * (QUEST_LOG_LINE_MARGIN + QUEST_LOG_NOTE_MAX))

typedef enum {
    QUEST_KIND_EXPLORE = 0, /**< look around somewhere and note what is there */
    QUEST_KIND_FIND,        /**< look for a named thing */
    QUEST_KIND_REVISIT,     /**< check later on something noted */
    QUEST_KIND_ASK,         /**< a question for a person, only when one talks to him */
    QUEST_KIND_COUNT
} quest_kind_t;

typedef enum {
    QUEST_STATE_OPEN = 0,
    QUEST_STATE_ACTIVE, /**< at most one */
    QUEST_STATE_DONE,
    QUEST_STATE_DROPPED,
    QUEST_STATE_COUNT
} quest_state_t;

typedef enum {
    QUEST_OK = 0,
    QUEST_ERR_ARG,  /**< bad kind, empty text, NULL */
    QUEST_ERR_FULL, /**< QUEST_LOG_OPEN_MAX quests already open */
    QUEST_ERR_NOT_FOUND,
    QUEST_ERR_STATE,  /**< the quest is closed */
    QUEST_ERR_REPEAT, /**< a near-repeat of one of the quest's notes; not stored */
} quest_err_t;

/** A point in time: Unix seconds once the clock has synced, seconds since boot
 *  before. A boot-relative time from an earlier boot means only "earlier". */
typedef struct {
    int64_t s;
    bool synced;
} quest_time_t;

typedef struct {
    uint16_t id; /**< 0 marks an unused slot */
    quest_kind_t kind;
    quest_state_t state;
    quest_time_t opened;
    quest_time_t closed; /**< s = 0 while open */
    char title[QUEST_LOG_TITLE_MAX + 1];
    char why[QUEST_LOG_WHY_MAX + 1];
    char outcome[QUEST_LOG_OUTCOME_MAX + 1];
} quest_t;

typedef struct {
    uint16_t quest_id; /**< 0 marks an unused slot */
    quest_time_t at;
    char pose; /**< dwell pose at capture: 'C', 'L', 'R', 'F', or '-' for none */
    char text[QUEST_LOG_NOTE_MAX + 1];
} quest_note_t;

/** The whole log. About 25 kB; the firmware keeps it in PSRAM. Quests and notes
 *  are each kept oldest first, packed at the front of their arrays. */
typedef struct {
    quest_t quests[QUEST_LOG_QUESTS_MAX];
    size_t quest_count;
    quest_note_t notes[QUEST_LOG_NOTES_MAX];
    size_t note_count;
    uint16_t next_id;
} quest_log_t;

void quest_log_init(quest_log_t *log);

/** Build a quest_time_t from the wall clock and the uptime: synced when
 *  @p epoch_s is at or past QUEST_LOG_EPOCH_VALID_S. */
quest_time_t quest_time_from(int64_t epoch_s, int64_t uptime_s);

/** Copy @p in to @p out (size @p out_len) with tabs, newlines and other
 *  control bytes turned into spaces, runs of spaces collapsed, leading and
 *  trailing spaces trimmed, and the result cut at a UTF-8 character boundary
 *  to fit. @return the length written. */
size_t quest_log_clean_text(const char *in, char *out, size_t out_len);

/** Open a quest. It starts `open`, not active. @p why may be NULL or empty.
 *  Prunes the oldest closed quest when QUEST_LOG_QUESTS_MAX are kept. */
quest_err_t quest_log_open(quest_log_t *log, quest_kind_t kind, const char *title, const char *why,
                           quest_time_t now, uint16_t *id_out);

/** Make @p id the active quest; the previously active one goes back to open. */
quest_err_t quest_log_set_active(quest_log_t *log, uint16_t id);

/** Close @p id as QUEST_STATE_DONE or QUEST_STATE_DROPPED. */
quest_err_t quest_log_close(quest_log_t *log, uint16_t id, quest_state_t state, const char *outcome,
                            quest_time_t now);

/** Add a note to an open or active quest. QUEST_ERR_REPEAT when the cleaned
 *  text is a near-repeat of one of that quest's notes. */
quest_err_t quest_log_note(quest_log_t *log, uint16_t quest_id, char pose, const char *text,
                           quest_time_t now);

/** Delete everything and start the ids from 1. */
void quest_log_clear(quest_log_t *log);

const quest_t *quest_log_find(const quest_log_t *log, uint16_t id);
/** The active quest, or NULL. */
const quest_t *quest_log_active(const quest_log_t *log);
/** Quests open or active. */
size_t quest_log_open_count(const quest_log_t *log);
/** Notes on @p quest_id. */
size_t quest_log_note_count(const quest_log_t *log, uint16_t quest_id);

const char *quest_kind_name(quest_kind_t kind);
const char *quest_state_name(quest_state_t state);
/** Parse a kind name ("explore", "find", "revisit", "ask"). */
bool quest_kind_from_name(const char *name, quest_kind_t *out);
const char *quest_err_name(quest_err_t err);

/** Write the log in the file format. @return the length the whole file needs,
 *  excluding the NUL; when that is >= @p out_len, @p out holds nothing usable
 *  (an empty string when @p out_len > 0). */
size_t quest_log_serialize(const quest_log_t *log, char *out, size_t out_len);

/** Replace @p log with what @p buf (length @p len, no NUL needed) holds.
 *  Skips incomplete, malformed and other-version lines, notes of quests it has
 *  not read, and every active quest after the first. Re-applies the caps.
 *  @return the number of records (quests plus notes) loaded. */
size_t quest_log_parse(quest_log_t *log, const char *buf, size_t len);

#ifdef __cplusplus
}
#endif
