/**
 * @file test_quest_log_core.c
 * @brief Host tests for Teuvo's quest log (ADR-039 §1).
 *
 * What a bench cannot stage: the 9th open quest, the 25th quest record, a
 * log cut at every byte offset by a power loss, a UTF-8 character straddling a
 * text cap, and a file from another format version.
 */

#include "quest_log_core.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        fflush(stdout);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static quest_log_t s_log;
static quest_log_t s_copy;
static char s_buf[QUEST_LOG_SERIALIZED_MAX + 1];

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    quest_log_init(&s_log);
    fn();
    test_pass++;
    printf("     PASS\n");
}

static quest_time_t at(int64_t s)
{
    return quest_time_from(QUEST_LOG_EPOCH_VALID_S + s, 0);
}

static uint16_t open_quest(const char *title)
{
    uint16_t id = 0;
    ASSERT(quest_log_open(&s_log, QUEST_KIND_EXPLORE, title, "because", at(0), &id) == QUEST_OK);
    return id;
}

/* ---- Time ---------------------------------------------------------------- */

static void time_is_synced_only_past_the_threshold(void)
{
    const quest_time_t before = quest_time_from(1000, 42);
    ASSERT(!before.synced);
    ASSERT(before.s == 42);
    const quest_time_t after = quest_time_from(QUEST_LOG_EPOCH_VALID_S, 42);
    ASSERT(after.synced);
    ASSERT(after.s == QUEST_LOG_EPOCH_VALID_S);
}

/* ---- Text cleaning ------------------------------------------------------- */

static void control_bytes_become_single_spaces(void)
{
    char out[64];
    const size_t n = quest_log_clean_text("  a\tb\n\nc\r d  ", out, sizeof(out));
    ASSERT(strcmp(out, "a b c d") == 0);
    ASSERT(n == 7);
}

static void a_cap_never_splits_a_utf8_character(void)
{
    /* "aä": 'a' then the two bytes of ä. A 3-byte buffer holds 2 bytes, which
     * would end mid-character, so only "a" may remain. */
    char out[3];
    quest_log_clean_text("a\xc3\xa4", out, sizeof(out));
    ASSERT(strcmp(out, "a") == 0);

    /* With room for all three bytes it is kept whole. */
    char out4[4];
    quest_log_clean_text("a\xc3\xa4", out4, sizeof(out4));
    ASSERT(strcmp(out4, "a\xc3\xa4") == 0);

    /* A three-byte character (€) cut after its first and second byte. */
    char out_e[4];
    quest_log_clean_text("ab\xe2\x82\xac", out_e, sizeof(out_e));
    ASSERT(strcmp(out_e, "ab") == 0);
}

static void a_cap_does_not_leave_a_trailing_space(void)
{
    char out[5];
    quest_log_clean_text("abc defgh", out, sizeof(out));
    ASSERT(strcmp(out, "abc") == 0);
}

static void stored_text_is_capped(void)
{
    char long_title[200];
    memset(long_title, 'x', sizeof(long_title) - 1);
    long_title[sizeof(long_title) - 1] = '\0';
    const uint16_t id = open_quest(long_title);
    ASSERT(strlen(quest_log_find(&s_log, id)->title) == QUEST_LOG_TITLE_MAX);
}

/* ---- Quests -------------------------------------------------------------- */

static void ids_start_at_one_and_increase(void)
{
    ASSERT(open_quest("first") == 1);
    ASSERT(open_quest("second") == 2);
    ASSERT(s_log.quest_count == 2);
    ASSERT(quest_log_find(&s_log, 1)->state == QUEST_STATE_OPEN);
    ASSERT(quest_log_active(&s_log) == NULL);
}

static void an_empty_title_or_bad_kind_is_refused(void)
{
    uint16_t id = 0;
    ASSERT(quest_log_open(&s_log, QUEST_KIND_FIND, " \t ", NULL, at(0), &id) == QUEST_ERR_ARG);
    ASSERT(quest_log_open(&s_log, QUEST_KIND_COUNT, "x", NULL, at(0), &id) == QUEST_ERR_ARG);
    ASSERT(s_log.quest_count == 0);
}

static void the_ninth_open_quest_is_refused(void)
{
    for (int i = 0; i < QUEST_LOG_OPEN_MAX; i++) {
        char t[16];
        snprintf(t, sizeof(t), "quest %d", i);
        open_quest(t);
    }
    uint16_t id = 0;
    ASSERT(quest_log_open(&s_log, QUEST_KIND_ASK, "one more", NULL, at(0), &id) == QUEST_ERR_FULL);
    /* Closing one makes room. */
    ASSERT(quest_log_close(&s_log, 3, QUEST_STATE_DONE, "found it", at(5)) == QUEST_OK);
    ASSERT(quest_log_open(&s_log, QUEST_KIND_ASK, "one more", NULL, at(0), &id) == QUEST_OK);
    ASSERT(quest_log_open_count(&s_log) == QUEST_LOG_OPEN_MAX);
}

static void only_one_quest_is_active(void)
{
    const uint16_t a = open_quest("a");
    const uint16_t b = open_quest("b");
    ASSERT(quest_log_set_active(&s_log, a) == QUEST_OK);
    ASSERT(quest_log_active(&s_log)->id == a);
    ASSERT(quest_log_set_active(&s_log, b) == QUEST_OK);
    ASSERT(quest_log_active(&s_log)->id == b);
    ASSERT(quest_log_find(&s_log, a)->state == QUEST_STATE_OPEN);
    ASSERT(quest_log_set_active(&s_log, 99) == QUEST_ERR_NOT_FOUND);
}

static void closing_the_active_quest_leaves_none_active(void)
{
    const uint16_t a = open_quest("a");
    quest_log_set_active(&s_log, a);
    ASSERT(quest_log_close(&s_log, a, QUEST_STATE_DROPPED, "gave up", at(9)) == QUEST_OK);
    ASSERT(quest_log_active(&s_log) == NULL);
    const quest_t *q = quest_log_find(&s_log, a);
    ASSERT(q->state == QUEST_STATE_DROPPED);
    ASSERT(q->closed.s == at(9).s);
    ASSERT(strcmp(q->outcome, "gave up") == 0);
    /* A closed quest cannot be closed again, activated or noted. */
    ASSERT(quest_log_close(&s_log, a, QUEST_STATE_DONE, NULL, at(10)) == QUEST_ERR_STATE);
    ASSERT(quest_log_set_active(&s_log, a) == QUEST_ERR_STATE);
    ASSERT(quest_log_note(&s_log, a, 'C', "late", at(11)) == QUEST_ERR_STATE);
    ASSERT(quest_log_close(&s_log, a, QUEST_STATE_OPEN, NULL, at(10)) == QUEST_ERR_ARG);
}

static void the_oldest_closed_quest_is_pruned_with_its_notes(void)
{
    /* Fill every record slot: close each quest so the open cap never binds. */
    for (int i = 0; i < QUEST_LOG_QUESTS_MAX; i++) {
        char t[16];
        snprintf(t, sizeof(t), "quest %d", i);
        const uint16_t id = open_quest(t);
        char note[32];
        snprintf(note, sizeof(note), "note for quest number %d", i);
        ASSERT(quest_log_note(&s_log, id, 'L', note, at(i)) == QUEST_OK);
        ASSERT(quest_log_close(&s_log, id, QUEST_STATE_DONE, "ok", at(i)) == QUEST_OK);
    }
    ASSERT(s_log.quest_count == QUEST_LOG_QUESTS_MAX);
    ASSERT(quest_log_find(&s_log, 1) != NULL);

    const uint16_t fresh = open_quest("fresh");
    ASSERT(fresh == QUEST_LOG_QUESTS_MAX + 1);
    ASSERT(s_log.quest_count == QUEST_LOG_QUESTS_MAX);
    ASSERT(quest_log_find(&s_log, 1) == NULL);
    ASSERT(quest_log_note_count(&s_log, 1) == 0);
    ASSERT(quest_log_find(&s_log, 2) != NULL);
    ASSERT(quest_log_note_count(&s_log, 2) == 1);
}

static void an_open_quest_is_never_pruned(void)
{
    const uint16_t keep = open_quest("keep me");
    for (int i = 0; i < 3 * QUEST_LOG_QUESTS_MAX; i++) {
        char t[16];
        snprintf(t, sizeof(t), "q %d", i);
        const uint16_t id = open_quest(t);
        quest_log_close(&s_log, id, QUEST_STATE_DONE, NULL, at(i));
    }
    ASSERT(quest_log_find(&s_log, keep) != NULL);
    ASSERT(quest_log_find(&s_log, keep)->state == QUEST_STATE_OPEN);
}

/* ---- Notes --------------------------------------------------------------- */

static void a_near_repeat_note_is_dropped(void)
{
    const uint16_t id = open_quest("find the ball");
    ASSERT(quest_log_note(&s_log, id, 'C', "A red ball under the sofa", at(1)) == QUEST_OK);
    ASSERT(quest_log_note(&s_log, id, 'C', "a red ball, under the sofa.", at(2)) ==
           QUEST_ERR_REPEAT);
    ASSERT(quest_log_note(&s_log, id, 'R', "A blue slipper by the door", at(3)) == QUEST_OK);
    ASSERT(quest_log_note_count(&s_log, id) == 2);

    /* The same words on another quest are not a repeat there. */
    const uint16_t other = open_quest("other");
    ASSERT(quest_log_note(&s_log, other, 'C', "A red ball under the sofa", at(4)) == QUEST_OK);
}

static void a_note_needs_text_and_a_known_quest(void)
{
    const uint16_t id = open_quest("q");
    ASSERT(quest_log_note(&s_log, id, 'C', "\n\t ", at(1)) == QUEST_ERR_ARG);
    ASSERT(quest_log_note(&s_log, 77, 'C', "something", at(1)) == QUEST_ERR_NOT_FOUND);
}

static void a_bad_pose_is_stored_as_none(void)
{
    const uint16_t id = open_quest("q");
    ASSERT(quest_log_note(&s_log, id, 'X', "something", at(1)) == QUEST_OK);
    ASSERT(s_log.notes[0].pose == '-');
    ASSERT(quest_log_note(&s_log, id, 'F', "the floor in front", at(1)) == QUEST_OK);
    ASSERT(s_log.notes[1].pose == 'F');
}

static void a_full_quest_drops_its_oldest_note(void)
{
    const uint16_t id = open_quest("busy");
    /* Distinct words each time so none is a near-repeat. */
    static const char *const words[] = {"alpha",   "bravo", "charlie", "delta",   "echo",
                                        "foxtrot", "golf",  "hotel",   "india",   "juliet",
                                        "kilo",    "lima",  "mike",    "november"};
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
        ASSERT(quest_log_note(&s_log, id, 'C', words[i], at((int64_t)i)) == QUEST_OK);
    }
    ASSERT(quest_log_note_count(&s_log, id) == QUEST_LOG_NOTES_PER_QUEST);
    ASSERT(strcmp(s_log.notes[0].text, "charlie") == 0);
    ASSERT(strcmp(s_log.notes[s_log.note_count - 1].text, "november") == 0);
}

static void the_open_quests_notes_always_fit(void)
{
    /* Every open slot filled with a full set of notes, plus closed history. */
    for (int c = 0; c < 4; c++) {
        char t[16];
        snprintf(t, sizeof(t), "old %d", c);
        const uint16_t id = open_quest(t);
        quest_log_note(&s_log, id, 'C', "old note", at(0));
        quest_log_close(&s_log, id, QUEST_STATE_DONE, NULL, at(0));
    }
    for (int q = 0; q < QUEST_LOG_OPEN_MAX; q++) {
        char t[16];
        snprintf(t, sizeof(t), "open %d", q);
        const uint16_t id = open_quest(t);
        for (int n = 0; n < QUEST_LOG_NOTES_PER_QUEST; n++) {
            char text[48];
            snprintf(text, sizeof(text), "quest%d observation%d", q, n);
            ASSERT(quest_log_note(&s_log, id, 'C', text, at(n)) == QUEST_OK);
        }
    }
    ASSERT(s_log.note_count == QUEST_LOG_NOTES_MAX);
    for (size_t i = 0; i < s_log.quest_count; i++) {
        const quest_t *q = &s_log.quests[i];
        if (q->state == QUEST_STATE_OPEN) {
            ASSERT(quest_log_note_count(&s_log, q->id) == QUEST_LOG_NOTES_PER_QUEST);
        }
    }
}

static void clear_empties_the_log_and_restarts_ids(void)
{
    open_quest("a");
    open_quest("b");
    quest_log_clear(&s_log);
    ASSERT(s_log.quest_count == 0);
    ASSERT(s_log.note_count == 0);
    ASSERT(open_quest("c") == 1);
}

/* ---- File format --------------------------------------------------------- */

static void build_sample_log(void)
{
    const uint16_t a = open_quest("What is behind the sofa?");
    const uint16_t b = open_quest("Whose are the blue slippers? \xc3\xa4\xc3\xb6");
    uint16_t c = 0;
    quest_log_open(&s_log, QUEST_KIND_FIND, "Find the red ball", "", quest_time_from(0, 77), &c);
    quest_log_set_active(&s_log, b);
    quest_log_note(&s_log, a, 'L', "A cable and a sock", at(10));
    quest_log_note(&s_log, b, 'C', "Slippers by the door", quest_time_from(5, 120));
    quest_log_note(&s_log, c, 'F', "Nothing on the floor", at(30));
    quest_log_close(&s_log, a, QUEST_STATE_DONE, "Only dust bunnies", at(40));
}

static int quests_equal(const quest_t *p, const quest_t *q)
{
    return p->id == q->id && p->kind == q->kind && p->state == q->state &&
           p->opened.s == q->opened.s && p->opened.synced == q->opened.synced &&
           p->closed.s == q->closed.s && p->closed.synced == q->closed.synced &&
           strcmp(p->title, q->title) == 0 && strcmp(p->why, q->why) == 0 &&
           strcmp(p->outcome, q->outcome) == 0;
}

static int notes_equal(const quest_note_t *p, const quest_note_t *q)
{
    return p->quest_id == q->quest_id && p->at.s == q->at.s && p->at.synced == q->at.synced &&
           p->pose == q->pose && strcmp(p->text, q->text) == 0;
}

static int logs_equal(const quest_log_t *x, const quest_log_t *y)
{
    if (x->quest_count != y->quest_count || x->note_count != y->note_count ||
        x->next_id != y->next_id) {
        return 0;
    }
    for (size_t i = 0; i < x->quest_count; i++) {
        if (!quests_equal(&x->quests[i], &y->quests[i])) {
            return 0;
        }
    }
    for (size_t i = 0; i < x->note_count; i++) {
        if (!notes_equal(&x->notes[i], &y->notes[i])) {
            return 0;
        }
    }
    return 1;
}

static void a_log_survives_a_round_trip(void)
{
    build_sample_log();
    const size_t n = quest_log_serialize(&s_log, s_buf, sizeof(s_buf));
    ASSERT(n > 0 && n < sizeof(s_buf));
    ASSERT(quest_log_parse(&s_copy, s_buf, n) == 6);
    ASSERT(logs_equal(&s_log, &s_copy));
    ASSERT(quest_log_active(&s_copy)->id == 2);
    ASSERT(!quest_log_find(&s_copy, 3)->opened.synced);
}

static void an_empty_log_round_trips(void)
{
    const size_t n = quest_log_serialize(&s_log, s_buf, sizeof(s_buf));
    ASSERT(quest_log_parse(&s_copy, s_buf, n) == 0);
    ASSERT(logs_equal(&s_log, &s_copy));
}

static void a_short_buffer_reports_the_needed_length(void)
{
    build_sample_log();
    const size_t need = quest_log_serialize(&s_log, NULL, 0);
    char small[16];
    ASSERT(quest_log_serialize(&s_log, small, sizeof(small)) == need);
    ASSERT(small[0] == '\0');
    ASSERT(quest_log_serialize(&s_log, s_buf, need + 1) == need);
    ASSERT(strlen(s_buf) == need);
}

static void a_full_log_fits_the_serialized_max(void)
{
    /* Every field at its cap and every time at its widest decimal form. */
    const quest_time_t wide_unsynced = quest_time_from(0, INT64_MIN);
    const quest_time_t wide_synced = quest_time_from(INT64_MAX, 0);
    const int closed = QUEST_LOG_QUESTS_MAX - QUEST_LOG_OPEN_MAX;
    for (int i = 0; i < QUEST_LOG_QUESTS_MAX; i++) {
        char t[QUEST_LOG_TITLE_MAX + 1];
        memset(t, 'T', sizeof(t) - 1);
        t[sizeof(t) - 1] = '\0';
        char w[QUEST_LOG_WHY_MAX + 1];
        memset(w, 'W', sizeof(w) - 1);
        w[sizeof(w) - 1] = '\0';
        uint16_t id = 0;
        ASSERT(quest_log_open(&s_log, QUEST_KIND_REVISIT, t, w, wide_unsynced, &id) == QUEST_OK);
        if (i < closed) {
            char o[QUEST_LOG_OUTCOME_MAX + 1];
            memset(o, 'O', sizeof(o) - 1);
            o[sizeof(o) - 1] = '\0';
            ASSERT(quest_log_close(&s_log, id, QUEST_STATE_DROPPED, o, wide_synced) == QUEST_OK);
        }
    }
    for (int q = closed + 1; q <= QUEST_LOG_QUESTS_MAX; q++) {
        for (int n = 0; n < QUEST_LOG_NOTES_PER_QUEST; n++) {
            char text[QUEST_LOG_NOTE_MAX + 1];
            memset(text, 'a' + n, sizeof(text) - 1);
            text[sizeof(text) - 1] = '\0';
            ASSERT(quest_log_note(&s_log, (uint16_t)q, 'C', text, wide_unsynced) == QUEST_OK);
        }
    }
    ASSERT(s_log.note_count == QUEST_LOG_NOTES_MAX);
    const size_t need = quest_log_serialize(&s_log, NULL, 0);
    ASSERT(need < QUEST_LOG_SERIALIZED_MAX);
    ASSERT(quest_log_serialize(&s_log, s_buf, sizeof(s_buf)) == need);
    ASSERT(quest_log_parse(&s_copy, s_buf, need) == QUEST_LOG_QUESTS_MAX + QUEST_LOG_NOTES_MAX);
    ASSERT(logs_equal(&s_log, &s_copy));
}

static void a_log_cut_at_any_byte_loads_only_whole_lines(void)
{
    build_sample_log();
    const size_t n = quest_log_serialize(&s_log, s_buf, sizeof(s_buf));
    size_t prev = 0;
    for (size_t cut = 0; cut <= n; cut++) {
        const size_t loaded = quest_log_parse(&s_copy, s_buf, cut);
        /* Monotonic: more bytes never load fewer records. */
        ASSERT(loaded >= prev);
        ASSERT(loaded <= 6);
        prev = loaded;
        /* What was loaded is a prefix of the original, field for field: a
         * cut inside a line's last field must not load a shortened text. */
        ASSERT(s_copy.quest_count <= s_log.quest_count);
        ASSERT(s_copy.note_count <= s_log.note_count);
        for (size_t i = 0; i < s_copy.quest_count; i++) {
            ASSERT(quests_equal(&s_copy.quests[i], &s_log.quests[i]));
        }
        for (size_t i = 0; i < s_copy.note_count; i++) {
            ASSERT(notes_equal(&s_copy.notes[i], &s_log.notes[i]));
        }
        /* An id in the log is never handed out again. */
        ASSERT(quest_log_find(&s_copy, s_copy.next_id) == NULL);
    }
    ASSERT(prev == 6);
}

static void other_versions_and_garbage_are_skipped(void)
{
    const char *file = "1\tM\t5\n"
                       "2\tQ\t9\t0\t0\t0\t0\t0\t0\tfuture\t\t\n" /* another version */
                       "garbage line\n"
                       "\n"
                       "1\tQ\t3\t1\t0\t100\t0\t0\t0\tfind it\twhy\t\n"
                       "1\tQ\t4\t7\t0\t100\t0\t0\t0\tbad kind\t\t\n"
                       "1\tQ\t3\t0\t0\t100\t0\t0\t0\tduplicate id\t\t\n"
                       "1\tQ\tx\t0\t0\t100\t0\t0\t0\tbad id\t\t\n"
                       "1\tN\t3\t5\t0\tC\tseen it\n"
                       "1\tN\t8\t5\t0\tC\tnote of an unknown quest\n"
                       "1\tN\t3\t5\t0\tCC\tbad pose\n"
                       "1\tN\t3\t5\t1\tC\textra\tfield\n";
    const size_t loaded = quest_log_parse(&s_copy, file, strlen(file));
    ASSERT(loaded == 2);
    ASSERT(s_copy.quest_count == 1);
    ASSERT(s_copy.note_count == 1);
    ASSERT(strcmp(s_copy.quests[0].title, "find it") == 0);
    ASSERT(s_copy.next_id == 5);
}

static void next_id_never_reuses_a_kept_id(void)
{
    const char *file = "1\tM\t2\n"
                       "1\tQ\t40\t0\t0\t0\t0\t0\t0\tlate\t\t\n";
    quest_log_parse(&s_copy, file, strlen(file));
    ASSERT(s_copy.next_id == 41);
}

static void a_second_active_quest_reads_as_open(void)
{
    const char *file = "1\tQ\t1\t0\t1\t0\t0\t0\t0\tfirst\t\t\n"
                       "1\tQ\t2\t0\t1\t0\t0\t0\t0\tsecond\t\t\n";
    ASSERT(quest_log_parse(&s_copy, file, strlen(file)) == 2);
    ASSERT(quest_log_active(&s_copy)->id == 1);
    ASSERT(quest_log_find(&s_copy, 2)->state == QUEST_STATE_OPEN);
}

static void a_file_over_the_caps_is_cut_to_them(void)
{
    /* Ten open quests in the file: the caps hold on load too. */
    size_t pos = 0;
    for (int i = 1; i <= 10; i++) {
        pos += (size_t)snprintf(s_buf + pos, sizeof(s_buf) - pos,
                                "1\tQ\t%d\t0\t0\t0\t0\t0\t0\tq%d\t\t\n", i, i);
    }
    quest_log_parse(&s_copy, s_buf, pos);
    ASSERT(quest_log_open_count(&s_copy) == QUEST_LOG_OPEN_MAX);
    ASSERT(quest_log_find(&s_copy, 9) == NULL);
}

static void names_round_trip(void)
{
    for (int k = 0; k < QUEST_KIND_COUNT; k++) {
        quest_kind_t out;
        ASSERT(quest_kind_from_name(quest_kind_name((quest_kind_t)k), &out));
        ASSERT(out == (quest_kind_t)k);
    }
    ASSERT(!quest_kind_from_name("wander", NULL));
    ASSERT(strcmp(quest_state_name(QUEST_STATE_DROPPED), "dropped") == 0);
    ASSERT(strcmp(quest_err_name(QUEST_ERR_REPEAT), "near-repeat of an earlier note") == 0);
}

int main(void)
{
    printf("=== quest_log_core host tests ===\n");
    test_run("time_is_synced_only_past_the_threshold", time_is_synced_only_past_the_threshold);
    test_run("control_bytes_become_single_spaces", control_bytes_become_single_spaces);
    test_run("a_cap_never_splits_a_utf8_character", a_cap_never_splits_a_utf8_character);
    test_run("a_cap_does_not_leave_a_trailing_space", a_cap_does_not_leave_a_trailing_space);
    test_run("stored_text_is_capped", stored_text_is_capped);
    test_run("ids_start_at_one_and_increase", ids_start_at_one_and_increase);
    test_run("an_empty_title_or_bad_kind_is_refused", an_empty_title_or_bad_kind_is_refused);
    test_run("the_ninth_open_quest_is_refused", the_ninth_open_quest_is_refused);
    test_run("only_one_quest_is_active", only_one_quest_is_active);
    test_run("closing_the_active_quest_leaves_none_active",
             closing_the_active_quest_leaves_none_active);
    test_run("the_oldest_closed_quest_is_pruned_with_its_notes",
             the_oldest_closed_quest_is_pruned_with_its_notes);
    test_run("an_open_quest_is_never_pruned", an_open_quest_is_never_pruned);
    test_run("a_near_repeat_note_is_dropped", a_near_repeat_note_is_dropped);
    test_run("a_note_needs_text_and_a_known_quest", a_note_needs_text_and_a_known_quest);
    test_run("a_bad_pose_is_stored_as_none", a_bad_pose_is_stored_as_none);
    test_run("a_full_quest_drops_its_oldest_note", a_full_quest_drops_its_oldest_note);
    test_run("the_open_quests_notes_always_fit", the_open_quests_notes_always_fit);
    test_run("clear_empties_the_log_and_restarts_ids", clear_empties_the_log_and_restarts_ids);
    test_run("a_log_survives_a_round_trip", a_log_survives_a_round_trip);
    test_run("an_empty_log_round_trips", an_empty_log_round_trips);
    test_run("a_short_buffer_reports_the_needed_length", a_short_buffer_reports_the_needed_length);
    test_run("a_full_log_fits_the_serialized_max", a_full_log_fits_the_serialized_max);
    test_run("a_log_cut_at_any_byte_loads_only_whole_lines",
             a_log_cut_at_any_byte_loads_only_whole_lines);
    test_run("other_versions_and_garbage_are_skipped", other_versions_and_garbage_are_skipped);
    test_run("next_id_never_reuses_a_kept_id", next_id_never_reuses_a_kept_id);
    test_run("a_second_active_quest_reads_as_open", a_second_active_quest_reads_as_open);
    test_run("a_file_over_the_caps_is_cut_to_them", a_file_over_the_caps_is_cut_to_them);
    test_run("names_round_trip", names_round_trip);
    printf("\n=== %d/%d tests passed ===\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
