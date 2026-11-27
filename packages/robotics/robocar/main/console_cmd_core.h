/**
 * @file console_cmd_core.h
 * @brief The console command table: which handler a console line reaches.
 *
 * One table, read by the serial console (execute_console_line() in main.c)
 * and by the MQTT command topic (mqtt_command.c), so the two cannot disagree
 * about which lines are commands. Before it, each kept its own prefix list,
 * synced by hand.
 *
 * A line reaches a command when it starts with that command's prefix, as the
 * strncmp chain it replaces did. No prefix starts another
 * (test_console_cmd_core.c), so table order does not matter. `mqtt` and
 * `gemini` are serial-only and stay out of the table: command_task() handles
 * them before it, so no MQTT message can reach them (#626).
 *
 * Pure C, no ESP-IDF: compiled unmodified by the host tests.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONSOLE_CMD_NONE = 0, /**< Not a table command. */
    CONSOLE_CMD_GPIO,
    CONSOLE_CMD_VOICE,
    CONSOLE_CMD_SNAP,
    CONSOLE_CMD_LISTEN,
    CONSOLE_CMD_TRACE,
    CONSOLE_CMD_MIC,
    CONSOLE_CMD_CAM,
    CONSOLE_CMD_PLAN,
    CONSOLE_CMD_IDLE,
    CONSOLE_CMD_CLIFF,
    CONSOLE_CMD_MOTORS,
    CONSOLE_CMD_OTA,
    CONSOLE_CMD_REPORT,
    CONSOLE_CMD_QUEST,
    CONSOLE_CMD_PERIPH, /**< `sound`, `servo` and `led`. */
    CONSOLE_CMD_COUNT,
} console_cmd_t;

/** The command @p line reaches, or CONSOLE_CMD_NONE (also for NULL). */
console_cmd_t console_cmd_lookup(const char *line);

/** Number of prefixes in the table. */
size_t console_cmd_prefix_count(void);

/** The @p i-th prefix, or NULL past the end. */
const char *console_cmd_prefix_at(size_t i);

#ifdef __cplusplus
}
#endif
