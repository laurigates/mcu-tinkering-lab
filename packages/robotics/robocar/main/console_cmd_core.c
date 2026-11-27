/**
 * @file console_cmd_core.c
 * @brief The console command table (console_cmd_core.h).
 */

#include "console_cmd_core.h"

#include <string.h>

typedef struct {
    const char *prefix;
    console_cmd_t cmd;
} console_cmd_entry_t;

/* The order of main.c's former strncmp chain; no prefix starts another, so it
 * does not decide anything (test_no_prefix_shadows_another). */
static const console_cmd_entry_t k_table[] = {
    {"gpio", CONSOLE_CMD_GPIO},     {"voice", CONSOLE_CMD_VOICE},   {"snap", CONSOLE_CMD_SNAP},
    {"listen", CONSOLE_CMD_LISTEN}, {"trace", CONSOLE_CMD_TRACE},   {"mic", CONSOLE_CMD_MIC},
    {"cam", CONSOLE_CMD_CAM},       {"plan", CONSOLE_CMD_PLAN},     {"idle", CONSOLE_CMD_IDLE},
    {"cliff", CONSOLE_CMD_CLIFF},   {"motors", CONSOLE_CMD_MOTORS}, {"ota", CONSOLE_CMD_OTA},
    {"report", CONSOLE_CMD_REPORT}, {"quest", CONSOLE_CMD_QUEST},   {"sound", CONSOLE_CMD_PERIPH},
    {"servo", CONSOLE_CMD_PERIPH},  {"led", CONSOLE_CMD_PERIPH},
};

#define TABLE_LEN (sizeof(k_table) / sizeof(k_table[0]))

console_cmd_t console_cmd_lookup(const char *line)
{
    if (!line) {
        return CONSOLE_CMD_NONE;
    }
    for (size_t i = 0; i < TABLE_LEN; i++) {
        if (strncmp(line, k_table[i].prefix, strlen(k_table[i].prefix)) == 0) {
            return k_table[i].cmd;
        }
    }
    return CONSOLE_CMD_NONE;
}

size_t console_cmd_prefix_count(void)
{
    return TABLE_LEN;
}

const char *console_cmd_prefix_at(size_t i)
{
    return i < TABLE_LEN ? k_table[i].prefix : NULL;
}
