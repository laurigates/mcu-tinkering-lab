/**
 * @file test_mqtt_command.c
 * @brief Host tests for the MQTT command-topic validator/dispatcher (issue #524).
 *
 * mqtt_command.c is pure C with no ESP-IDF/FreeRTOS dependency, so it compiles
 * and runs unmodified here — the properties under test are exactly the ones
 * that matter for an inbound message from an untrusted network boundary:
 *
 *   - a malformed or oversized payload never reaches dispatch (it must not be
 *     able to crash or wedge the MQTT event task);
 *   - a payload fragmented across multiple MQTT_EVENT_DATA callbacks (a
 *     target — the topic — resolved once, with more of the write still
 *     arriving) is rejected rather than acted on as if it were complete;
 *   - an unknown command is rejected rather than silently dropped or, worse,
 *     silently accepted;
 *   - a valid movement command reaches only the injected `movement` callback
 *     — standing in for reactive_controller_manual() via the same queue the
 *     serial console uses — and never the `console_line` callback, which is
 *     the regression this suite exists to pin: this module must not grow a
 *     path that lets a movement word fall through to a generic handler, or
 *     vice versa.
 */

#include "mqtt_command.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
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

/* ------------------------------------------------------------------------
 * Recording ops — stand in for the real callbacks (dispatch_movement() /
 * execute_console_line() in main.c) without pulling in FreeRTOS queues or
 * motor_controller.h. The whole point of mqtt_command.c's ops-callback
 * design is that it never references motor_controller.h at all; this struct
 * proves what the module actually calls, and nothing more.
 * ------------------------------------------------------------------------ */
typedef struct {
    int movement_calls;
    char last_movement_word[32];
    int console_line_calls;
    char last_console_line[MQTT_COMMAND_MAX_LEN + 1];
} recorder_t;

static void record_movement(const char *word, void *ctx)
{
    recorder_t *r = (recorder_t *)ctx;
    r->movement_calls++;
    strncpy(r->last_movement_word, word, sizeof(r->last_movement_word) - 1);
    r->last_movement_word[sizeof(r->last_movement_word) - 1] = '\0';
}

static void record_console_line(const char *line, void *ctx)
{
    recorder_t *r = (recorder_t *)ctx;
    r->console_line_calls++;
    strncpy(r->last_console_line, line, sizeof(r->last_console_line) - 1);
    r->last_console_line[sizeof(r->last_console_line) - 1] = '\0';
}

static void recorder_reset(recorder_t *r)
{
    memset(r, 0, sizeof(*r));
}

static mqtt_command_ops_t ops_for(recorder_t *r)
{
    mqtt_command_ops_t ops = {
        .movement = record_movement,
        .console_line = record_console_line,
        .ctx = r,
    };
    return ops;
}

/* ------------------------------------------------------------------------
 * mqtt_command_extract_line()
 * ------------------------------------------------------------------------ */

/* The straightforward case: a short, complete, printable payload arriving in
 * a single (unfragmented) MQTT_EVENT_DATA event. */
static void test_extract_accepts_a_normal_payload(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    const char *payload = "plan resume";
    const int len = (int)strlen(payload);

    ASSERT(mqtt_command_extract_line(payload, len, len, 0, line, sizeof(line)) == true);
    ASSERT(strcmp(line, "plan resume") == 0);
}

/* A single letter is the smallest legal payload. */
static void test_extract_accepts_single_letter(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    ASSERT(mqtt_command_extract_line("F", 1, 1, 0, line, sizeof(line)) == true);
    ASSERT(strcmp(line, "F") == 0);
}

/* Trailing whitespace some publishers append (mosquitto_pub -m from a shell
 * heredoc, a trailing newline) is trimmed rather than turning a valid command
 * into an unrecognised one. */
static void test_extract_trims_trailing_whitespace(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    const char *payload = "plan resume\n";
    const int len = (int)strlen(payload);

    ASSERT(mqtt_command_extract_line(payload, len, len, 0, line, sizeof(line)) == true);
    ASSERT(strcmp(line, "plan resume") == 0);
}

/* An empty payload must be rejected, not accepted as a zero-length line. */
static void test_extract_rejects_empty_payload(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    ASSERT(mqtt_command_extract_line("x", 0, 0, 0, line, sizeof(line)) == false);
}

/* An oversized payload must never be copied — the primary "cannot crash the
 * MQTT event task" guarantee this module exists to provide. */
static void test_extract_rejects_oversized_payload(void)
{
    char oversized[256];
    memset(oversized, 'A', sizeof(oversized));
    char line[MQTT_COMMAND_MAX_LEN + 1];

    ASSERT(mqtt_command_extract_line(oversized, (int)sizeof(oversized), (int)sizeof(oversized), 0,
                                     line, sizeof(line)) == false);
}

/* A payload with an embedded control byte (including a NUL byte hidden
 * inside the declared length) is rejected outright, rather than silently
 * truncated at the embedded NUL and treated as a shorter, "valid" command. */
static void test_extract_rejects_non_printable_bytes(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    const char with_embedded_nul[] = {'F', '\0', 'B'};
    const char with_control_byte[] = {'p', 'l', 'a', 'n', ' ', 0x07 /* BEL */};

    ASSERT(mqtt_command_extract_line(with_embedded_nul, (int)sizeof(with_embedded_nul),
                                     (int)sizeof(with_embedded_nul), 0, line,
                                     sizeof(line)) == false);
    ASSERT(mqtt_command_extract_line(with_control_byte, (int)sizeof(with_control_byte),
                                     (int)sizeof(with_control_byte), 0, line,
                                     sizeof(line)) == false);
}

/* A payload that is ONLY whitespace trims to nothing and must be rejected,
 * not dispatched as an empty command line. */
static void test_extract_rejects_all_whitespace_payload(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    ASSERT(mqtt_command_extract_line("   ", 3, 3, 0, line, sizeof(line)) == false);
}

/* The fragmentation/ordering hazard: esp-mqtt can deliver one logical message
 * across several MQTT_EVENT_DATA callbacks (event->total_data_len /
 * event->current_data_offset) once a payload exceeds its internal buffer.
 * The topic ("target") is resolved once, on the first callback, while more
 * of the payload ("the write") is still arriving — dispatching on a fragment
 * would act on a truncated command. Both fragmentation signals are checked
 * independently. */
static void test_extract_rejects_fragmented_messages(void)
{
    char line[MQTT_COMMAND_MAX_LEN + 1];
    const char *payload = "plan resume";
    const int len = (int)strlen(payload);

    /* First fragment of a larger message: this chunk looks complete on its
     * own (data_len bytes are all printable) but total_data_len says more is
     * coming. */
    ASSERT(mqtt_command_extract_line(payload, len, len + 10, 0, line, sizeof(line)) == false);

    /* A continuation fragment: non-zero offset into a larger message. */
    ASSERT(mqtt_command_extract_line(payload, len, len + 10, len, line, sizeof(line)) == false);
}

/* Defensive: NULL/undersized destination arguments must not be dereferenced. */
static void test_extract_rejects_bad_output_buffer(void)
{
    char tiny[4];  // smaller than MQTT_COMMAND_MAX_LEN + 1
    ASSERT(mqtt_command_extract_line("F", 1, 1, 0, NULL, MQTT_COMMAND_MAX_LEN + 1) == false);
    ASSERT(mqtt_command_extract_line("F", 1, 1, 0, tiny, sizeof(tiny)) == false);
    ASSERT(mqtt_command_extract_line(NULL, 1, 1, 0, tiny, sizeof(tiny)) == false);
}

/* ------------------------------------------------------------------------
 * mqtt_command_dispatch()
 * ------------------------------------------------------------------------ */

/* An unknown command is rejected: neither callback fires, and the caller
 * (mqtt_logger.c) is told so it can log-and-drop rather than silently
 * ignoring it. */
static void test_dispatch_rejects_unknown_command(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);

    ASSERT(mqtt_command_dispatch("reticulate_splines", &ops) == false);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);

    /* A near-miss on a real prefix (not a whole recognised token) is still
     * unknown — this module does not do fuzzy/partial matching. */
    ASSERT(mqtt_command_dispatch("pla", &ops) == false);
    ASSERT(mqtt_command_dispatch("", &ops) == false);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* The load-bearing case for issue #524: a valid drive command must reach the
 * injected movement callback (standing in for reactive_controller_manual()
 * via the console's existing motor queue) and must NEVER reach console_line
 * — there is no code path here by which a movement word could be forwarded
 * to a generic handler instead of the motor-command route. */
static void test_dispatch_routes_movement_to_movement_callback_only(void)
{
    recorder_t r;

    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);
    ASSERT(mqtt_command_dispatch("F", &ops) == true);
    ASSERT(r.movement_calls == 1);
    ASSERT(strcmp(r.last_movement_word, "forward") == 0);
    ASSERT(r.console_line_calls == 0);

    /* Case-insensitive, matching the console's switch exactly. */
    recorder_reset(&r);
    ASSERT(mqtt_command_dispatch("f", &ops) == true);
    ASSERT(r.movement_calls == 1);
    ASSERT(strcmp(r.last_movement_word, "forward") == 0);

    /* Every single-letter console movement command. */
    static const struct {
        const char *letter;
        const char *word;
    } cases[] = {
        {"B", "backward"},  {"L", "left"},       {"R", "right"},
        {"C", "rotate_cw"}, {"W", "rotate_ccw"}, {"S", "stop"},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        recorder_reset(&r);
        ASSERT(mqtt_command_dispatch(cases[i].letter, &ops) == true);
        ASSERT(r.movement_calls == 1);
        ASSERT(strcmp(r.last_movement_word, cases[i].word) == 0);
        ASSERT(r.console_line_calls == 0);
    }

    /* The full word form (a machine client may prefer this to a bare
     * letter) dispatches identically. */
    recorder_reset(&r);
    ASSERT(mqtt_command_dispatch("rotate_ccw", &ops) == true);
    ASSERT(r.movement_calls == 1);
    ASSERT(strcmp(r.last_movement_word, "rotate_ccw") == 0);
    ASSERT(r.console_line_calls == 0);
}

/* A recognised non-movement command is forwarded verbatim to console_line —
 * the same dispatch the serial console uses (execute_console_line() in
 * main.c) — and never touches the movement callback. */
static void test_dispatch_routes_console_commands_to_console_callback_only(void)
{
    recorder_t r;
    mqtt_command_ops_t ops = ops_for(&r);

    static const char *const lines[] = {
        "plan resume", "voice quiet 30", "trace reset",       "snap 3",
        "listen 5",    "mic dump 10",    "cam gainceiling 4", "servo pan 20",
        "led 255 0 0", "sound beep",     "gpio mode 3 out",
    };
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        recorder_reset(&r);
        ASSERT(mqtt_command_dispatch(lines[i], &ops) == true);
        ASSERT(r.console_line_calls == 1);
        ASSERT(strcmp(r.last_console_line, lines[i]) == 0);
        ASSERT(r.movement_calls == 0);
    }
}

/* A NULL category callback disables that whole category: a movement command
 * with no movement handler is rejected exactly like an unknown command,
 * never silently forwarded to console_line (or vice versa). */
static void test_dispatch_rejects_when_category_disabled(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t movement_only = {
        .movement = record_movement,
        .console_line = NULL,
        .ctx = &r,
    };
    ASSERT(mqtt_command_dispatch("plan resume", &movement_only) == false);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);

    recorder_reset(&r);
    mqtt_command_ops_t console_only = {
        .movement = NULL,
        .console_line = record_console_line,
        .ctx = &r,
    };
    ASSERT(mqtt_command_dispatch("F", &console_only) == false);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* Defensive: NULL line/ops must not be dereferenced. */
static void test_dispatch_rejects_null_args(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);

    ASSERT(mqtt_command_dispatch(NULL, &ops) == false);
    ASSERT(mqtt_command_dispatch("F", NULL) == false);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* ------------------------------------------------------------------------
 * End-to-end: extract, then dispatch — the shape mqtt_logger.c actually
 * uses, run over a raw (non-NUL-terminated) buffer the way esp-mqtt hands
 * event->data to the caller.
 * ------------------------------------------------------------------------ */
static void test_end_to_end_extract_then_dispatch(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);

    const char raw[] = {'F'};  // deliberately not NUL-terminated
    char line[MQTT_COMMAND_MAX_LEN + 1];

    ASSERT(mqtt_command_extract_line(raw, (int)sizeof(raw), (int)sizeof(raw), 0, line,
                                     sizeof(line)) == true);
    ASSERT(mqtt_command_dispatch(line, &ops) == true);
    ASSERT(r.movement_calls == 1);
    ASSERT(r.console_line_calls == 0);

    /* A malformed payload must never reach dispatch at all. */
    recorder_reset(&r);
    const char garbage[] = {0x01, 0x02, 0x03};
    ASSERT(mqtt_command_extract_line(garbage, (int)sizeof(garbage), (int)sizeof(garbage), 0, line,
                                     sizeof(line)) == false);
    /* Caller (mqtt_logger.c) must not call mqtt_command_dispatch() when
     * extraction failed; this asserts the recorder saw nothing, i.e. that
     * discipline is what keeps a malformed payload from ever being acted
     * on. */
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

int main(void)
{
    printf("=== MQTT command dispatcher host tests (issue #524) ===\n\n");

    test_run("extract_accepts_a_normal_payload", test_extract_accepts_a_normal_payload);
    test_run("extract_accepts_single_letter", test_extract_accepts_single_letter);
    test_run("extract_trims_trailing_whitespace", test_extract_trims_trailing_whitespace);
    test_run("extract_rejects_empty_payload", test_extract_rejects_empty_payload);
    test_run("extract_rejects_oversized_payload", test_extract_rejects_oversized_payload);
    test_run("extract_rejects_non_printable_bytes", test_extract_rejects_non_printable_bytes);
    test_run("extract_rejects_all_whitespace_payload", test_extract_rejects_all_whitespace_payload);
    test_run("extract_rejects_fragmented_messages", test_extract_rejects_fragmented_messages);
    test_run("extract_rejects_bad_output_buffer", test_extract_rejects_bad_output_buffer);
    test_run("dispatch_rejects_unknown_command", test_dispatch_rejects_unknown_command);
    test_run("dispatch_routes_movement_to_movement_callback_only",
             test_dispatch_routes_movement_to_movement_callback_only);
    test_run("dispatch_routes_console_commands_to_console_callback_only",
             test_dispatch_routes_console_commands_to_console_callback_only);
    test_run("dispatch_rejects_when_category_disabled",
             test_dispatch_rejects_when_category_disabled);
    test_run("dispatch_rejects_null_args", test_dispatch_rejects_null_args);
    test_run("end_to_end_extract_then_dispatch", test_end_to_end_extract_then_dispatch);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
