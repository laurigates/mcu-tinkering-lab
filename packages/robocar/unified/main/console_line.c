/**
 * @file console_line.c
 * @brief Serial console line assembly, with Improv packets kept out of it.
 */

#include "console_line.h"

#include "improv_wifi.h"

void console_line_init(console_line_t *line)
{
    line->len = 0;
    line->buf[0] = '\0';
    line->improv_session = 0;
    line->in_packet = false;
    line->draining = false;
    line->skip_to_eol = false;
}

bool console_line_improv_armed(uint32_t improv_session)
{
    return (improv_session & 1u) != 0;
}

/* Improv was armed or disarmed since the last byte (issue #1087). An arm
 * resets the parser at once: whatever it held is from before the gap. A
 * disarm does too, unless a packet is in flight; then the parser finishes
 * that packet and is reset at its end, because its remaining bytes, a 0x0A
 * length byte among them, cannot be told from text without it. */
static void on_session_change(console_line_t *line, bool armed)
{
    if (armed || !line->in_packet) {
        improv_wifi_reset_parser();
        line->draining = false;
        if (line->in_packet) {
            /* Re-armed with a packet cut off: its tail is no command. */
            line->in_packet = false;
            line->skip_to_eol = true;
        }
    } else {
        line->draining = true;
    }
}

const char *console_line_feed(console_line_t *line, char ch, uint32_t improv_session)
{
    if (improv_session != line->improv_session) {
        line->improv_session = improv_session;
        on_session_change(line, console_line_improv_armed(improv_session));
    }

    const bool parse = console_line_improv_armed(improv_session) || line->draining;
    if (parse && improv_wifi_process_byte((uint8_t)ch)) {
        /* Part of an Improv packet: whatever is buffered is its header (and
         * anything typed before it on the same line), so drop it all. */
        line->len = 0;
        line->in_packet = true;
        line->skip_to_eol = false;
        return NULL;
    }
    if (line->draining) {
        /* The packet in flight at the disarm has ended. */
        line->draining = false;
        improv_wifi_reset_parser();
    }
    if (line->in_packet) {
        /* The first byte after a packet. A well-formed packet ends with a
         * newline here; anything else up to the newline is the rest of a
         * packet the parser rejected, such as one whose length byte was too
         * small, and holds the password just as well. */
        line->in_packet = false;
        line->skip_to_eol = true;
    }

    if (ch == '\n' || ch == '\r') {
        if (line->skip_to_eol) {
            line->skip_to_eol = false;
            line->len = 0;
            return NULL;
        }
        if (line->len == 0) {
            return NULL;
        }
        line->buf[line->len] = '\0';
        line->len = 0;
        return line->buf;
    }
    if (line->skip_to_eol) {
        return NULL;
    }
    if (line->len < sizeof(line->buf) - 1) {
        line->buf[line->len++] = ch;
    }
    return NULL;
}
