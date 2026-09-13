/**
 * @file mqtt_command.c
 * @brief Validate and route commands received on the MQTT command topic.
 *
 * See mqtt_command.h for the design rationale (the ESP-IDF-free split from
 * mqtt_logger.c) and the fragmentation hazard mqtt_command_extract_line()
 * guards against.
 */

#include "mqtt_command.h"

#include <string.h>

static bool is_printable_ascii(char c)
{
    return c >= 0x20 && c < 0x7F;
}

bool mqtt_command_extract_line(const char *data, int data_len, int total_data_len,
                               int current_data_offset, char *out_line, size_t out_line_size)
{
    if (!data || !out_line || out_line_size <= MQTT_COMMAND_MAX_LEN) {
        return false;
    }
    if (data_len <= 0 || (size_t)data_len > MQTT_COMMAND_MAX_LEN) {
        return false;
    }
    /* A fragmented message: the topic ("target") was already resolved on the
     * first MQTT_EVENT_DATA callback, but more of this payload is still
     * arriving on later ones. Reject rather than act on a partial command —
     * every command line this firmware recognises is far under the size at
     * which esp-mqtt would ever fragment it, so a legitimate message always
     * has current_data_offset == 0 and total_data_len == data_len. */
    if (current_data_offset != 0 || total_data_len != data_len) {
        return false;
    }

    /* Trim trailing whitespace some MQTT publishers append (a shell heredoc's
     * trailing newline, mosquitto_pub's -m argument, ...) BEFORE the
     * printable-ASCII check below — CR/LF/tab are control bytes, and doing
     * this after the check would reject an otherwise-valid line for the
     * whitespace it is about to lose anyway. Leading whitespace is left
     * alone: none of the recognised commands start with it, so a line that
     * does simply fails to match anything downstream and is rejected as
     * unrecognised — the correct outcome for that malformed input. */
    int len = data_len;
    while (len > 0 && (data[len - 1] == ' ' || data[len - 1] == '\t' || data[len - 1] == '\r' ||
                       data[len - 1] == '\n')) {
        len--;
    }
    if (len == 0) {
        return false;  // all-whitespace payload
    }

    for (int i = 0; i < len; i++) {
        if (!is_printable_ascii(data[i])) {
            return false;
        }
    }

    memcpy(out_line, data, (size_t)len);
    out_line[len] = '\0';
    return true;
}

/*
 * Console command-line prefixes forwarded verbatim to ops->console_line.
 * Kept as an explicit allowlist — rather than "anything that isn't
 * movement" — so a typo or an as-yet-unsupported string is rejected and
 * logged instead of silently reaching a handler chain via a coincidental
 * prefix match. Mirrors the strncmp chain in main.c's command_task() /
 * execute_console_line(); keep the two lists in sync.
 */
static const char *const k_console_prefixes[] = {
    "gpio", "voice", "snap", "listen", "trace", "mic", "cam", "plan", "sound", "servo", "led",
};

static bool matches_console_prefix(const char *line)
{
    for (size_t i = 0; i < sizeof(k_console_prefixes) / sizeof(k_console_prefixes[0]); i++) {
        size_t plen = strlen(k_console_prefixes[i]);
        if (strncmp(line, k_console_prefixes[i], plen) == 0) {
            return true;
        }
    }
    return false;
}

/* Single-letter forms mirror the console's command_task() switch exactly
 * (buf_pos == 1). Word forms mirror dispatch_movement()'s vocabulary, so an
 * MQTT client can send either without this module knowing about queues or
 * reactive_manual_cmd_t. */
static const char *movement_word_for(const char *line)
{
    if (line[1] == '\0') {
        switch (line[0]) {
            case 'F':
            case 'f':
                return "forward";
            case 'B':
            case 'b':
                return "backward";
            case 'L':
            case 'l':
                return "left";
            case 'R':
            case 'r':
                return "right";
            case 'C':
            case 'c':
                return "rotate_cw";
            case 'W':
            case 'w':
                return "rotate_ccw";
            case 'S':
            case 's':
                return "stop";
            default:
                return NULL;
        }
    }

    static const char *const words[] = {
        "forward", "backward", "left", "right", "rotate_cw", "rotate_ccw", "stop",
    };
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
        if (strcmp(line, words[i]) == 0) {
            return words[i];
        }
    }
    return NULL;
}

bool mqtt_command_dispatch(const char *line, const mqtt_command_ops_t *ops)
{
    if (!line || !ops || line[0] == '\0') {
        return false;
    }

    const char *word = movement_word_for(line);
    if (word) {
        if (!ops->movement) {
            return false;
        }
        ops->movement(word, ops->ctx);
        return true;
    }

    if (matches_console_prefix(line)) {
        if (!ops->console_line) {
            return false;
        }
        ops->console_line(line, ops->ctx);
        return true;
    }

    return false;
}
