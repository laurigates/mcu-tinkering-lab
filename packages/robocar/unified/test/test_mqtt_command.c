/**
 * @file test_mqtt_command.c
 * @brief Host tests for the MQTT command-topic validator/dispatcher (issues #524, #626).
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
        .access = MQTT_COMMAND_ACCESS_FULL,
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

    ASSERT(mqtt_command_dispatch("reticulate_splines", &ops) != MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);

    /* A near-miss on a real prefix (not a whole recognised token) is still
     * unknown — this module does not do fuzzy/partial matching. */
    ASSERT(mqtt_command_dispatch("pla", &ops) != MQTT_COMMAND_OK);
    ASSERT(mqtt_command_dispatch("", &ops) != MQTT_COMMAND_OK);
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
    ASSERT(mqtt_command_dispatch("F", &ops) == MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 1);
    ASSERT(strcmp(r.last_movement_word, "forward") == 0);
    ASSERT(r.console_line_calls == 0);

    /* Case-insensitive, matching the console's switch exactly. */
    recorder_reset(&r);
    ASSERT(mqtt_command_dispatch("f", &ops) == MQTT_COMMAND_OK);
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
        ASSERT(mqtt_command_dispatch(cases[i].letter, &ops) == MQTT_COMMAND_OK);
        ASSERT(r.movement_calls == 1);
        ASSERT(strcmp(r.last_movement_word, cases[i].word) == 0);
        ASSERT(r.console_line_calls == 0);
    }

    /* The full word form (a machine client may prefer this to a bare
     * letter) dispatches identically. */
    recorder_reset(&r);
    ASSERT(mqtt_command_dispatch("rotate_ccw", &ops) == MQTT_COMMAND_OK);
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
        ASSERT(mqtt_command_dispatch(lines[i], &ops) == MQTT_COMMAND_OK);
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
        .access = MQTT_COMMAND_ACCESS_FULL,
    };
    ASSERT(mqtt_command_dispatch("plan resume", &movement_only) != MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);

    recorder_reset(&r);
    mqtt_command_ops_t console_only = {
        .movement = NULL,
        .console_line = record_console_line,
        .ctx = &r,
        .access = MQTT_COMMAND_ACCESS_FULL,
    };
    ASSERT(mqtt_command_dispatch("F", &console_only) != MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* Defensive: NULL line/ops must not be dereferenced. */
static void test_dispatch_rejects_null_args(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);

    ASSERT(mqtt_command_dispatch(NULL, &ops) != MQTT_COMMAND_OK);
    ASSERT(mqtt_command_dispatch("F", NULL) != MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* ------------------------------------------------------------------------
 * Read-only lockout (issue #626)
 *
 * With no broker credentials configured, anything on the LAN that can reach
 * the broker can publish to the command topic. In that mode only the
 * explicit read-only allow-list may reach a handler; movement, every
 * state-changing command, and every command nobody has classified yet are
 * refused. The allow-list is the security boundary, so these cases pin it
 * from both sides.
 * ------------------------------------------------------------------------ */

static mqtt_command_ops_t read_only_ops_for(recorder_t *r)
{
    mqtt_command_ops_t ops = ops_for(r);
    ops.access = MQTT_COMMAND_ACCESS_READ_ONLY;
    return ops;
}

/* Full access needs BOTH halves. A username alone, a password alone, or an
 * empty string standing in for either (the CMake stub's default) leaves the
 * board locked to read-only. */
static void test_access_requires_both_credentials(void)
{
    ASSERT(mqtt_command_access_for_credentials("robocar", "secret") == MQTT_COMMAND_ACCESS_FULL);

    ASSERT(mqtt_command_access_for_credentials(NULL, NULL) == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_access_for_credentials("", "") == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_access_for_credentials("robocar", NULL) == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_access_for_credentials("robocar", "") == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_access_for_credentials(NULL, "secret") == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_access_for_credentials("", "secret") == MQTT_COMMAND_ACCESS_READ_ONLY);
}

/* The boot log and the facts line print these words; pin them so a rename is
 * a deliberate change rather than a silent one. */
static void test_access_names(void)
{
    ASSERT(strcmp(mqtt_command_access_name(MQTT_COMMAND_ACCESS_FULL), "full") == 0);
    ASSERT(strcmp(mqtt_command_access_name(MQTT_COMMAND_ACCESS_READ_ONLY), "read-only") == 0);
}

/* Fail closed: an ops table that never set .access is read-only, because
 * READ_ONLY is the zero value. A future caller that forgets the field gets
 * the locked mode, not the open one. */
static void test_zero_initialised_access_is_read_only(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = {
        .movement = record_movement,
        .console_line = record_console_line,
        .ctx = &r,
    };
    ASSERT(ops.access == MQTT_COMMAND_ACCESS_READ_ONLY);
    ASSERT(mqtt_command_dispatch("F", &ops) == MQTT_COMMAND_REFUSED_READ_ONLY);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* Every movement letter, in both cases, and every movement word. */
static void test_read_only_refuses_every_movement(void)
{
    static const char *const lines[] = {
        "F",       "f",        "B",    "b",     "L",         "l",          "R",
        "r",       "C",        "c",    "W",     "w",         "S",          "s",
        "forward", "backward", "left", "right", "rotate_cw", "rotate_ccw", "stop",
    };
    recorder_t r;
    mqtt_command_ops_t ops = read_only_ops_for(&r);
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        recorder_reset(&r);
        ASSERT(mqtt_command_check(lines[i], MQTT_COMMAND_ACCESS_READ_ONLY) ==
               MQTT_COMMAND_REFUSED_READ_ONLY);
        ASSERT(mqtt_command_dispatch(lines[i], &ops) == MQTT_COMMAND_REFUSED_READ_ONLY);
        ASSERT(r.movement_calls == 0);
        ASSERT(r.console_line_calls == 0);
    }
}

/* Each state-changing form of every forwarded command family, plus lines
 * that share an allow-listed command's prefix without BEING it. The latter
 * matter because the console handlers parse loosely — `trace foo` falls
 * through to the trace report, `voice <slug>` switches the persona and
 * persists it to NVS — so only an exact match is a safe test. */
static void test_read_only_refuses_state_changing_commands(void)
{
    static const char *const lines[] = {
        /* plan */
        "plan resume",
        "plan on",
        "plan off",
        "plan wake",
        "plan sleep",
        "plan scene 3",
        "plan range 10",
        "plan requests 100",
        "plan tokens 5000",
        /* voice */
        "voice quiet 30",
        "voice budget 3 300",
        "voice repeat 60",
        "voice scene 8",
        "voice loud 12",
        "voice sound 6",
        "voice vad on",
        "voice resume",
        "voice ration 6 300",
        "voice turns 50",
        "voice turns",
        "voice volume 50",
        "voice fx",
        "voice fx on",
        "voice fx body 65",
        "voice say hello",
        "voice name Kore",
        "voice fi_1950",
        "voice vary",
        "voice said extra",
        /* trace */
        "trace reset",
        "trace led on",
        "trace led off",
        "trace led",
        "trace foo",
        /* privacy: camera frames and raw microphone audio are not status */
        "snap",
        "snap 3",
        "mic dump",
        "mic dump 10",
        "listen",
        "listen 5",
        "listen clear",
        /* cam */
        "cam gainceiling 4",
        "cam ae 1",
        "cam brightness -1",
        "cam flip on",
        "cam mirror off",
        /* servo */
        "servo pan 20",
        "servo tilt -10",
        "servo on",
        "servo off",
        "servo exercise",
        "servo limit pan -40 40",
        "servo freq 50",
        /* actuators */
        "led 255 0 0",
        "led",
        "sound beep",
        "sound melody",
        /* gpio */
        "gpio mode 3 out",
        "gpio set 3 1",
        "gpio get 3",
        /* near misses of allow-listed lines */
        "planx",
        "servos",
        "cameras",
        "mic ",
        "voice  said",
    };
    recorder_t r;
    mqtt_command_ops_t ops = read_only_ops_for(&r);
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        recorder_reset(&r);
        const mqtt_command_result_t got = mqtt_command_dispatch(lines[i], &ops);
        if (got != MQTT_COMMAND_REFUSED_READ_ONLY) {
            printf("     line \"%s\" -> %s\n", lines[i], mqtt_command_result_reason(got));
        }
        ASSERT(got == MQTT_COMMAND_REFUSED_READ_ONLY);
        ASSERT(r.movement_calls == 0);
        ASSERT(r.console_line_calls == 0);
    }
}

/* The allow-list itself: each entry reaches console_line verbatim, never
 * movement. If a line here stops being read-only in main.c, it must come off
 * the list in mqtt_command.c and off this table together. */
static void test_read_only_allows_status_commands(void)
{
    static const char *const lines[] = {
        "plan", "trace", "mic", "cam", "servo", "gpio", "voice", "voice said",
    };
    recorder_t r;
    mqtt_command_ops_t ops = read_only_ops_for(&r);
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        recorder_reset(&r);
        ASSERT(mqtt_command_is_read_only(lines[i]));
        ASSERT(mqtt_command_check(lines[i], MQTT_COMMAND_ACCESS_READ_ONLY) == MQTT_COMMAND_OK);
        ASSERT(mqtt_command_dispatch(lines[i], &ops) == MQTT_COMMAND_OK);
        ASSERT(r.console_line_calls == 1);
        ASSERT(strcmp(r.last_console_line, lines[i]) == 0);
        ASSERT(r.movement_calls == 0);
    }
}

/* An unknown command is refused in read-only mode too — reported as
 * unrecognised, so the log names the real reason. */
static void test_read_only_refuses_unknown_command(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = read_only_ops_for(&r);

    ASSERT(mqtt_command_dispatch("reticulate_splines", &ops) == MQTT_COMMAND_UNRECOGNISED);
    ASSERT(mqtt_command_dispatch("", &ops) == MQTT_COMMAND_UNRECOGNISED);
    ASSERT(r.movement_calls == 0);
    ASSERT(r.console_line_calls == 0);
}

/* The serial-only `mqtt auth` provisioning command must never be reachable
 * over MQTT, in either mode: otherwise an anonymous publisher could set the
 * credentials that lift its own lockout, and a full-access one could lock the
 * owner out. */
static void test_credential_command_is_never_reachable(void)
{
    static const char *const lines[] = {"mqtt", "mqtt auth robocar secret", "mqtt auth clear"};
    recorder_t r;
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        recorder_reset(&r);
        mqtt_command_ops_t full = ops_for(&r);
        ASSERT(mqtt_command_dispatch(lines[i], &full) == MQTT_COMMAND_UNRECOGNISED);
        mqtt_command_ops_t ro = read_only_ops_for(&r);
        ASSERT(mqtt_command_dispatch(lines[i], &ro) == MQTT_COMMAND_UNRECOGNISED);
        ASSERT(r.movement_calls == 0);
        ASSERT(r.console_line_calls == 0);
    }
}

/* Full access keeps today's behaviour: a movement and a state-changing line
 * still dispatch. Pinned here beside the lockout so the two modes are read
 * as a pair. */
static void test_full_access_dispatches_state_changing_commands(void)
{
    recorder_t r;
    recorder_reset(&r);
    mqtt_command_ops_t ops = ops_for(&r);

    ASSERT(mqtt_command_check("F", MQTT_COMMAND_ACCESS_FULL) == MQTT_COMMAND_OK);
    ASSERT(mqtt_command_dispatch("F", &ops) == MQTT_COMMAND_OK);
    ASSERT(mqtt_command_dispatch("plan resume", &ops) == MQTT_COMMAND_OK);
    ASSERT(r.movement_calls == 1);
    ASSERT(r.console_line_calls == 1);
}

/* Every outcome has a distinct, printable reason for the log line. */
static void test_result_reasons_are_distinct(void)
{
    const char *ok = mqtt_command_result_reason(MQTT_COMMAND_OK);
    const char *unk = mqtt_command_result_reason(MQTT_COMMAND_UNRECOGNISED);
    const char *ro = mqtt_command_result_reason(MQTT_COMMAND_REFUSED_READ_ONLY);
    ASSERT(ok && unk && ro);
    ASSERT(strcmp(ok, unk) != 0 && strcmp(ok, ro) != 0 && strcmp(unk, ro) != 0);
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
    ASSERT(mqtt_command_dispatch(line, &ops) == MQTT_COMMAND_OK);
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
    printf("=== MQTT command dispatcher host tests (issues #524, #626) ===\n\n");

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
    test_run("access_requires_both_credentials", test_access_requires_both_credentials);
    test_run("access_names", test_access_names);
    test_run("zero_initialised_access_is_read_only", test_zero_initialised_access_is_read_only);
    test_run("read_only_refuses_every_movement", test_read_only_refuses_every_movement);
    test_run("read_only_refuses_state_changing_commands",
             test_read_only_refuses_state_changing_commands);
    test_run("read_only_allows_status_commands", test_read_only_allows_status_commands);
    test_run("read_only_refuses_unknown_command", test_read_only_refuses_unknown_command);
    test_run("credential_command_is_never_reachable", test_credential_command_is_never_reachable);
    test_run("full_access_dispatches_state_changing_commands",
             test_full_access_dispatches_state_changing_commands);
    test_run("result_reasons_are_distinct", test_result_reasons_are_distinct);
    test_run("end_to_end_extract_then_dispatch", test_end_to_end_extract_then_dispatch);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
