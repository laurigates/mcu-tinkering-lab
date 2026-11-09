/**
 * @file test_event_report_core.c
 * @brief Host tests for the GlitchTip event JSON and the breadcrumb ring
 *        (ADR-033).
 *
 * What a bench cannot stage on demand: RTC memory full of garbage after a
 * power-on, a line cut off mid-write by a reset, a ring that has wrapped many
 * times, and the exact JSON shape the host bridge parses.
 */

#include "event_report_core.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

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

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

static event_crumbs_t s_ring;

static void write_line(uint32_t seq, const char *text)
{
    (void)event_crumbs_write(&s_ring, seq, text, strlen(text));
}

/* ---------------------------------------------------------------- ring --- */

static void garbage_ring_is_not_adopted(void)
{
    memset(&s_ring, 0xA5, sizeof(s_ring));
    ASSERT(!event_crumbs_adopt(&s_ring));
}

static void reset_ring_is_adopted_and_empty(void)
{
    memset(&s_ring, 0xA5, sizeof(s_ring));
    event_crumbs_reset(&s_ring);
    ASSERT(event_crumbs_adopt(&s_ring));
    const char *out[EVENT_CRUMBS_LINES];
    ASSERT(event_crumbs_collect(&s_ring, out, EVENT_CRUMBS_LINES) == 0);
}

static void lines_come_back_oldest_first(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "one");
    write_line(1, "two");
    write_line(2, "three");
    const char *out[EVENT_CRUMBS_LINES];
    const size_t n = event_crumbs_collect(&s_ring, out, EVENT_CRUMBS_LINES);
    ASSERT(n == 3);
    ASSERT(strcmp(out[0], "one") == 0);
    ASSERT(strcmp(out[2], "three") == 0);
}

static void wrapped_ring_keeps_the_newest_lines(void)
{
    event_crumbs_reset(&s_ring);
    char text[16];
    const uint32_t total = EVENT_CRUMBS_LINES * 3 + 5;
    for (uint32_t s = 0; s < total; s++) {
        snprintf(text, sizeof(text), "line %u", (unsigned)s);
        write_line(s, text);
    }
    const char *out[EVENT_CRUMBS_LINES];
    const size_t n = event_crumbs_collect(&s_ring, out, EVENT_CRUMBS_LINES);
    ASSERT(n == EVENT_CRUMBS_LINES);
    snprintf(text, sizeof(text), "line %u", (unsigned)(total - EVENT_CRUMBS_LINES));
    ASSERT(strcmp(out[0], text) == 0);
    snprintf(text, sizeof(text), "line %u", (unsigned)(total - 1));
    ASSERT(strcmp(out[n - 1], text) == 0);
}

static void collect_respects_max(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "a");
    write_line(1, "b");
    write_line(2, "c");
    const char *out[2];
    ASSERT(event_crumbs_collect(&s_ring, out, 2) == 2);
    /* The newest two, since the oldest are the least useful. */
    ASSERT(strcmp(out[0], "b") == 0);
    ASSERT(strcmp(out[1], "c") == 0);
}

static void log_line_is_cleaned(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "\x1b[0;32mI (123) main: ok\x1b[0m\r\n");
    const char *out[1];
    ASSERT(event_crumbs_collect(&s_ring, out, 1) == 1);
    ASSERT(strcmp(out[0], "I (123) main: ok") == 0);
}

static void control_bytes_become_question_marks(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "a\tb\x01"
                  "c");
    const char *out[1];
    ASSERT(event_crumbs_collect(&s_ring, out, 1) == 1);
    ASSERT(strcmp(out[0], "a?b?c") == 0);
}

static void empty_after_cleaning_is_not_stored(void)
{
    event_crumbs_reset(&s_ring);
    ASSERT(!event_crumbs_write(&s_ring, 0, "\r\n", 2));
    ASSERT(!event_crumbs_write(&s_ring, 0, NULL, 4));
    const char *out[1];
    ASSERT(event_crumbs_collect(&s_ring, out, 1) == 0);
}

static void long_line_is_cut_and_terminated(void)
{
    event_crumbs_reset(&s_ring);
    char text[EVENT_CRUMBS_LINE_MAX * 2];
    memset(text, 'x', sizeof(text));
    ASSERT(event_crumbs_write(&s_ring, 0, text, sizeof(text)));
    const char *out[1];
    ASSERT(event_crumbs_collect(&s_ring, out, 1) == 1);
    ASSERT(strlen(out[0]) == EVENT_CRUMBS_LINE_MAX - 1);
}

static void adopt_terminates_a_torn_line(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "fine");
    /* A reset mid-write leaves a slot with no NUL in it. */
    memset(s_ring.lines[0], 'y', EVENT_CRUMBS_LINE_MAX);
    ASSERT(event_crumbs_adopt(&s_ring));
    const char *out[1];
    ASSERT(event_crumbs_collect(&s_ring, out, 1) == 1);
    ASSERT(strlen(out[0]) == EVENT_CRUMBS_LINE_MAX - 1);
}

/* Two cores: the later line's write lands first. Both must come back, in
 * sequence order, and the newest must not be dropped. */
static void racing_writers_keep_sequence_order(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0, "first");
    write_line(2, "third, landed first");
    write_line(1, "second, landed last");
    const char *out[EVENT_CRUMBS_LINES];
    ASSERT(event_crumbs_collect(&s_ring, out, EVENT_CRUMBS_LINES) == 3);
    ASSERT(strcmp(out[0], "first") == 0);
    ASSERT(strcmp(out[1], "second, landed last") == 0);
    ASSERT(strcmp(out[2], "third, landed first") == 0);
}

static void order_survives_the_counter_wrapping(void)
{
    event_crumbs_reset(&s_ring);
    write_line(0xFFFFFFFEu, "before wrap");
    write_line(0xFFFFFFFFu, "at wrap");
    write_line(0u, "after wrap");
    const char *out[3];
    ASSERT(event_crumbs_collect(&s_ring, out, 3) == 3);
    ASSERT(strcmp(out[0], "before wrap") == 0);
    ASSERT(strcmp(out[2], "after wrap") == 0);
}

/* -------------------------------------------------------------- policy --- */

static void deliberate_resets_are_not_reported(void)
{
    const char *quiet[] = {"POWERON", "SW", "EXT", "USB", "JTAG", "DEEPSLEEP"};
    for (size_t i = 0; i < sizeof(quiet) / sizeof(quiet[0]); i++) {
        ASSERT(!event_report_reset_is_reportable(quiet[i]));
    }
}

static void faults_are_reported(void)
{
    const char *loud[] = {"PANIC",      "INT_WDT",    "TASK_WDT", "WDT",    "BROWNOUT",
                          "PWR_GLITCH", "CPU_LOCKUP", "UNKNOWN",  "NEW_ONE"};
    for (size_t i = 0; i < sizeof(loud) / sizeof(loud[0]); i++) {
        ASSERT(event_report_reset_is_reportable(loud[i]));
    }
    ASSERT(event_report_reset_is_reportable(NULL));
}

static void id_is_32_lowercase_hex(void)
{
    const uint32_t words[4] = {0x0123ABCDu, 0, 0xFFFFFFFFu, 0x10u};
    char id[EVENT_REPORT_ID_LEN + 1];
    event_report_format_id(words, id);
    ASSERT(strlen(id) == EVENT_REPORT_ID_LEN);
    ASSERT(strcmp(id, "0123abcd00000000ffffffff00000010") == 0);
}

/* ---------------------------------------------------------------- json --- */

static event_report_t base_event(void)
{
    event_report_t ev = {
        .event_id = "0123abcd00000000ffffffff00000010",
        .timestamp = 0,
        .level = "error",
        .source = "firmware",
        .release = "robocar-unified@1.2.3",
        .message = "Reset: BROWNOUT",
    };
    return ev;
}

static cJSON *render_parse(const event_report_t *ev)
{
    char *json = event_report_render(ev);
    ASSERT(json != NULL);
    cJSON *root = cJSON_Parse(json);
    cJSON_free(json);
    ASSERT(root != NULL);
    return root;
}

static const char *str_of(const cJSON *obj, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}

static void minimal_event_has_the_contract_fields(void)
{
    const event_report_t ev = base_event();
    cJSON *root = render_parse(&ev);
    ASSERT(strcmp(str_of(root, "event_id"), ev.event_id) == 0);
    ASSERT(strcmp(str_of(root, "level"), "error") == 0);
    ASSERT(strcmp(str_of(root, "source"), "firmware") == 0);
    ASSERT(strcmp(str_of(root, "release"), "robocar-unified@1.2.3") == 0);
    ASSERT(strcmp(str_of(root, "message"), "Reset: BROWNOUT") == 0);
    const cJSON *ts = cJSON_GetObjectItemCaseSensitive(root, "timestamp");
    ASSERT(cJSON_IsNumber(ts) && ts->valuedouble == 0);
    /* No exception object unless one was given. */
    ASSERT(cJSON_GetObjectItemCaseSensitive(root, "exception") == NULL);
    cJSON_Delete(root);
}

static void missing_required_field_renders_nothing(void)
{
    event_report_t ev = base_event();
    ev.release = NULL;
    ASSERT(event_report_render(&ev) == NULL);
    ev = base_event();
    ev.event_id = NULL;
    ASSERT(event_report_render(&ev) == NULL);
    ASSERT(event_report_render(NULL) == NULL);
}

static void tags_extras_and_fingerprint_render(void)
{
    event_report_t ev = base_event();
    ev.fingerprint[0] = "reset";
    ev.fingerprint[1] = "BROWNOUT";
    ev.fingerprint_count = 2;
    ev.tags[0] = (event_report_tag_t){"reset_reason", "BROWNOUT"};
    ev.tags[1] = (event_report_tag_t){"build_sha", "abc1234"};
    ev.tags[2] = (event_report_tag_t){NULL, "skipped"};
    ev.tag_count = 3;
    ev.extras[0] = (event_report_extra_t){"uptime_ms", 4321};
    ev.extra_count = 1;
    cJSON *root = render_parse(&ev);

    const cJSON *fp = cJSON_GetObjectItemCaseSensitive(root, "fingerprint");
    ASSERT(cJSON_GetArraySize(fp) == 2);
    ASSERT(strcmp(cJSON_GetArrayItem(fp, 1)->valuestring, "BROWNOUT") == 0);

    const cJSON *tags = cJSON_GetObjectItemCaseSensitive(root, "tags");
    ASSERT(cJSON_GetArraySize(tags) == 2);
    ASSERT(strcmp(str_of(tags, "build_sha"), "abc1234") == 0);

    const cJSON *extra = cJSON_GetObjectItemCaseSensitive(root, "extra");
    const cJSON *up = cJSON_GetObjectItemCaseSensitive(extra, "uptime_ms");
    ASSERT(cJSON_IsNumber(up) && up->valuedouble == 4321);
    cJSON_Delete(root);
}

static void breadcrumbs_render_as_log_category(void)
{
    const char *crumbs[] = {"first", NULL, "second"};
    event_report_t ev = base_event();
    ev.breadcrumbs = crumbs;
    ev.breadcrumb_count = 3;
    cJSON *root = render_parse(&ev);
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "breadcrumbs");
    ASSERT(cJSON_GetArraySize(arr) == 2);
    const cJSON *b1 = cJSON_GetArrayItem(arr, 1);
    ASSERT(strcmp(str_of(b1, "message"), "second") == 0);
    ASSERT(strcmp(str_of(b1, "category"), "log") == 0);
    cJSON_Delete(root);
}

static void breadcrumb_category_can_be_set(void)
{
    const char *goals[] = {"track 40%", "stop"};
    event_report_t ev = base_event();
    ev.source = "planner";
    ev.breadcrumbs = goals;
    ev.breadcrumb_count = 2;
    ev.breadcrumb_category = "goal";
    cJSON *root = render_parse(&ev);
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "breadcrumbs");
    ASSERT(cJSON_GetArraySize(arr) == 2);
    ASSERT(strcmp(str_of(cJSON_GetArrayItem(arr, 0), "category"), "goal") == 0);
    ASSERT(strcmp(str_of(root, "source"), "planner") == 0);
    cJSON_Delete(root);
}

static void exception_frames_render_as_hex_innermost_first(void)
{
    event_report_t ev = base_event();
    ev.exception_type = "LoadProhibited";
    ev.exception_value = "PC 0x42001234";
    ev.frames[0] = 0x42001234u;
    ev.frames[1] = 0x4200abcdu;
    ev.frame_count = 2;
    cJSON *root = render_parse(&ev);
    const cJSON *exc = cJSON_GetObjectItemCaseSensitive(root, "exception");
    ASSERT(strcmp(str_of(exc, "type"), "LoadProhibited") == 0);
    const cJSON *frames = cJSON_GetObjectItemCaseSensitive(exc, "frames");
    ASSERT(cJSON_GetArraySize(frames) == 2);
    ASSERT(strcmp(str_of(cJSON_GetArrayItem(frames, 0), "instruction_addr"), "0x42001234") == 0);
    ASSERT(strcmp(str_of(cJSON_GetArrayItem(frames, 1), "instruction_addr"), "0x4200abcd") == 0);
    cJSON_Delete(root);
}

static void oversized_counts_are_clamped(void)
{
    event_report_t ev = base_event();
    ev.exception_type = "x";
    ev.frame_count = EVENT_REPORT_FRAMES_MAX + 50;
    ev.tag_count = EVENT_REPORT_TAGS_MAX + 50;
    ev.extra_count = EVENT_REPORT_EXTRAS_MAX + 50;
    ev.fingerprint_count = EVENT_REPORT_FINGERPRINT_MAX + 50;
    cJSON *root = render_parse(&ev);
    const cJSON *frames = cJSON_GetObjectItemCaseSensitive(
        cJSON_GetObjectItemCaseSensitive(root, "exception"), "frames");
    ASSERT(cJSON_GetArraySize(frames) == EVENT_REPORT_FRAMES_MAX);
    cJSON_Delete(root);
}

int main(void)
{
    printf("=== event_report_core host tests ===\n");
    test_run("garbage_ring_is_not_adopted", garbage_ring_is_not_adopted);
    test_run("reset_ring_is_adopted_and_empty", reset_ring_is_adopted_and_empty);
    test_run("lines_come_back_oldest_first", lines_come_back_oldest_first);
    test_run("wrapped_ring_keeps_the_newest_lines", wrapped_ring_keeps_the_newest_lines);
    test_run("collect_respects_max", collect_respects_max);
    test_run("log_line_is_cleaned", log_line_is_cleaned);
    test_run("control_bytes_become_question_marks", control_bytes_become_question_marks);
    test_run("empty_after_cleaning_is_not_stored", empty_after_cleaning_is_not_stored);
    test_run("long_line_is_cut_and_terminated", long_line_is_cut_and_terminated);
    test_run("adopt_terminates_a_torn_line", adopt_terminates_a_torn_line);
    test_run("racing_writers_keep_sequence_order", racing_writers_keep_sequence_order);
    test_run("order_survives_the_counter_wrapping", order_survives_the_counter_wrapping);
    test_run("deliberate_resets_are_not_reported", deliberate_resets_are_not_reported);
    test_run("faults_are_reported", faults_are_reported);
    test_run("id_is_32_lowercase_hex", id_is_32_lowercase_hex);
    test_run("minimal_event_has_the_contract_fields", minimal_event_has_the_contract_fields);
    test_run("missing_required_field_renders_nothing", missing_required_field_renders_nothing);
    test_run("tags_extras_and_fingerprint_render", tags_extras_and_fingerprint_render);
    test_run("breadcrumbs_render_as_log_category", breadcrumbs_render_as_log_category);
    test_run("breadcrumb_category_can_be_set", breadcrumb_category_can_be_set);
    test_run("exception_frames_render_as_hex_innermost_first",
             exception_frames_render_as_hex_innermost_first);
    test_run("oversized_counts_are_clamped", oversized_counts_are_clamped);
    printf("\n%d/%d tests passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
