/**
 * @file console_line.c
 * @brief Serial console line assembly, with Improv packets kept out of it.
 */

#include "console_line.h"

void console_line_init(console_line_t *line)
{
    line->len = 0;
    line->buf[0] = '\0';
}

const char *console_line_feed(console_line_t *line, char ch, bool improv_owned)
{
    if (improv_owned) {
        /* Part of an Improv packet: whatever is buffered is its header (and
         * anything typed before it on the same line), so drop it all. */
        line->len = 0;
        return NULL;
    }
    if (ch == '\n' || ch == '\r') {
        if (line->len == 0) {
            return NULL;
        }
        line->buf[line->len] = '\0';
        line->len = 0;
        return line->buf;
    }
    if (line->len < sizeof(line->buf) - 1) {
        line->buf[line->len++] = ch;
    }
    return NULL;
}
