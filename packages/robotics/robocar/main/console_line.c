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
    line->last_ms = 0;
    line->in_packet = false;
    line->draining = false;
    line->dropping = false;
}

bool console_line_improv_armed(uint32_t improv_session)
{
    return (improv_session & 1u) != 0;
}

/* The first byte after a quiet gap (issue #1087). A client writes a packet in
 * one go, so a packet the parser was inside when the gap began is either cut
 * off for good (a new packet follows) or was stalled across it (its tail
 * follows). The parser is reset for the first case, and the burst is dropped
 * for the second, since the tail holds the password and its 0x0A length byte
 * would split it into a line. A fresh header in the burst still parses. A
 * partly matched header is kept: the parser drops it by itself on the first
 * byte that does not continue it, and if it was a packet's start, the rest of
 * the header completes it. */
static void on_quiet_gap(console_line_t *line)
{
    const bool in_frame = line->in_packet && !improv_wifi_parser_idle();
    if (in_frame) {
        improv_wifi_reset_parser();
        line->draining = false;
    }
    line->in_packet = false;
    line->dropping = in_frame;
}

const char *console_line_feed(console_line_t *line, char ch, uint32_t improv_session,
                              uint32_t now_ms)
{
    if ((uint32_t)(now_ms - line->last_ms) > CONSOLE_LINE_BURST_GAP_MS) {
        on_quiet_gap(line);
    }
    line->last_ms = now_ms;

    if (improv_session != line->improv_session) {
        /* Armed or disarmed since the last byte. A packet the parser has
         * started on, header bytes included, keeps being fed until it ends,
         * whatever the new state: its remaining bytes cannot be told from text
         * without the parser. */
        line->improv_session = improv_session;
        line->draining = !improv_wifi_parser_idle();
    }

    const bool parse = console_line_improv_armed(improv_session) || line->draining;
    const bool claimed = parse && improv_wifi_process_byte((uint8_t)ch);
    if (line->draining && improv_wifi_parser_idle()) {
        /* The packet in flight at the edge has ended (or the header bytes were
         * text after all), so the parser is not fed again while disarmed. */
        line->draining = false;
    }
    if (claimed) {
        /* Part of an Improv packet: whatever is buffered is its header (and
         * anything typed before it on the same line), so drop it all. */
        line->len = 0;
        line->in_packet = true;
        return NULL;
    }
    if (line->in_packet) {
        /* The first byte after a packet. A well-formed one ends with a newline
         * here; anything else is the rest of a packet the parser rejected, such
         * as one whose length byte was too small, and holds the password just
         * as well, 0x0A and 0x0D bytes included. Drop the rest of the burst. */
        line->in_packet = false;
        line->dropping = true;
    }
    if (line->dropping) {
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
