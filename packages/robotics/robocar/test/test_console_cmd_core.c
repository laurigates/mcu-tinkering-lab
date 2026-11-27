/**
 * @file test_console_cmd_core.c
 * @brief Host tests for the console command table (console_cmd_core.c).
 *
 * One table decides which handler a console line reaches, for the serial
 * console (execute_console_line() in main.c) and for the MQTT command topic
 * (mqtt_command.c's recognition step). Before it, the two kept their own
 * copies of the prefix list "in sync" by hand. These tests pin the table as
 * main.c's strncmp chain had it, so the move is behaviour-preserving.
 */

#include "console_cmd_core.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;

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
    printf("     PASS\n");
}

/* Every prefix main.c's execute_console_line() chain dispatched, with the
 * handler it reached. */
static void test_each_prefix_reaches_its_command(void)
{
    static const struct {
        const char *line;
        console_cmd_t cmd;
    } k_cases[] = {
        {"gpio", CONSOLE_CMD_GPIO},     {"voice", CONSOLE_CMD_VOICE},
        {"snap", CONSOLE_CMD_SNAP},     {"listen", CONSOLE_CMD_LISTEN},
        {"trace", CONSOLE_CMD_TRACE},   {"mic", CONSOLE_CMD_MIC},
        {"cam", CONSOLE_CMD_CAM},       {"plan", CONSOLE_CMD_PLAN},
        {"idle", CONSOLE_CMD_IDLE},     {"cliff", CONSOLE_CMD_CLIFF},
        {"motors", CONSOLE_CMD_MOTORS}, {"ota", CONSOLE_CMD_OTA},
        {"report", CONSOLE_CMD_REPORT}, {"quest", CONSOLE_CMD_QUEST},
        {"sound", CONSOLE_CMD_PERIPH},  {"servo", CONSOLE_CMD_PERIPH},
        {"led", CONSOLE_CMD_PERIPH},
    };
    for (size_t i = 0; i < sizeof(k_cases) / sizeof(k_cases[0]); i++) {
        ASSERT(console_cmd_lookup(k_cases[i].line) == k_cases[i].cmd);
    }
    /* The same count as the table: nothing more is recognised. */
    ASSERT(console_cmd_prefix_count() == sizeof(k_cases) / sizeof(k_cases[0]));
}

/* Matching is by prefix, as the strncmp chain was: arguments follow, and a
 * longer word that starts with a prefix still reaches it. */
static void test_matching_is_by_prefix(void)
{
    ASSERT(console_cmd_lookup("voice live on") == CONSOLE_CMD_VOICE);
    ASSERT(console_cmd_lookup("servo pan 30") == CONSOLE_CMD_PERIPH);
    ASSERT(console_cmd_lookup("quest open find Find the red ball") == CONSOLE_CMD_QUEST);
    ASSERT(console_cmd_lookup("camera") == CONSOLE_CMD_CAM);
}

/* Serial-only commands, movement words and anything else are not in the table:
 * `mqtt` and `gemini` are handled by command_task() before the table, so no
 * MQTT message can reach them (#626). */
static void test_unknown_lines_are_none(void)
{
    ASSERT(console_cmd_lookup("mqtt") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("mqtt auth user pass") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("gemini auth key") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("forward") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("F") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup(NULL) == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup(" voice") == CONSOLE_CMD_NONE);
    ASSERT(console_cmd_lookup("Voice") == CONSOLE_CMD_NONE);
}

/* No prefix starts another, so the table's order cannot change which command
 * a line reaches. A new prefix that breaks this fails here. */
static void test_no_prefix_shadows_another(void)
{
    const size_t n = console_cmd_prefix_count();
    for (size_t i = 0; i < n; i++) {
        const char *a = console_cmd_prefix_at(i);
        ASSERT(a != NULL && a[0] != '\0');
        for (size_t j = 0; j < n; j++) {
            if (i == j) {
                continue;
            }
            const char *b = console_cmd_prefix_at(j);
            ASSERT(strncmp(b, a, strlen(a)) != 0);
        }
    }
    ASSERT(console_cmd_prefix_at(n) == NULL);
}

/* Every command but NONE has a prefix, so no handler is unreachable. */
static void test_every_command_has_a_prefix(void)
{
    for (int c = CONSOLE_CMD_NONE + 1; c < CONSOLE_CMD_COUNT; c++) {
        bool found = false;
        for (size_t i = 0; i < console_cmd_prefix_count(); i++) {
            if (console_cmd_lookup(console_cmd_prefix_at(i)) == (console_cmd_t)c) {
                found = true;
            }
        }
        ASSERT(found);
    }
}

int main(void)
{
    printf("=== console_cmd_core host tests ===\n\n");
    test_run("each_prefix_reaches_its_command", test_each_prefix_reaches_its_command);
    test_run("matching_is_by_prefix", test_matching_is_by_prefix);
    test_run("unknown_lines_are_none", test_unknown_lines_are_none);
    test_run("no_prefix_shadows_another", test_no_prefix_shadows_another);
    test_run("every_command_has_a_prefix", test_every_command_has_a_prefix);
    printf("\n=== %d tests passed ===\n", test_count);
    return 0;
}
