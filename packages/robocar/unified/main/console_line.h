/**
 * @file console_line.h
 * @brief Serial console line assembly, with Improv packets kept out of it.
 *
 * command_task() reads the USB-Serial-JTAG console one byte at a time, and
 * ESP Web Tools speaks Improv Serial over the same port. While Improv is
 * armed, each byte goes to the Improv parser first; this module then builds
 * console lines from the bytes the parser did not claim. A byte it claimed
 * (improv_wifi_process_byte() returned true) drops the partial line and is
 * never stored, so:
 *
 *  - a Send Wi-Fi settings packet, which carries the password in plaintext,
 *    never becomes a line, and so is never logged as `Serial cmd:` (whence the
 *    UART0 mirror and the GlitchTip breadcrumb ring) or dispatched;
 *  - a 0x0A or 0x0D inside a packet (a length byte, a checksum) does not end a
 *    line, so no fragment of the packet is dispatched either, such as a
 *    one-byte checksum of 'F' read as "drive forward".
 *
 * Issue #1087 closed the paths around a packet's edges:
 *
 *  - the parser is reset whenever Improv is armed, and when it is disarmed
 *    (at once, or, with a packet in flight, when that packet ends), so a
 *    half-read packet from before a link edge cannot make the parser mis-frame
 *    the next one and hand its password back as text;
 *  - the bytes after a packet up to the next CR or LF are dropped: after a
 *    well-formed packet that is only its newline, after one the parser
 *    rejected (a length byte too small) it is the rest of that packet.
 *
 * The armed state arrives as a session count rather than a flag: the network
 * supervisor bumps it on every arm and every disarm, so it is odd while armed,
 * and a disarm followed by a re-arm between two console bytes is still seen.
 *
 * No ESP-IDF dependency, so test_console_line.c runs it on the host together
 * with the real Improv parser.
 */

#ifndef CONSOLE_LINE_H
#define CONSOLE_LINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Longest serial console line, NUL included. 32 until issue #626: `mqtt auth
 * <user> <pass>` needs room for a 32-character username and a 64-character
 * password. Every handler parses with width-bounded sscanf formats, so the
 * longer line is safe for all of them. */
#define CONSOLE_LINE_MAX 128

typedef struct {
    char buf[CONSOLE_LINE_MAX];
    size_t len;
    uint32_t improv_session;  ///< The session the last byte was read in
    bool in_packet;           ///< The parser claimed the last byte
    bool draining;            ///< Disarmed, finishing the packet in flight
    bool skip_to_eol;         ///< Dropping the bytes after a packet
} console_line_t;

/** Start with an empty line, Improv disarmed (session 0). */
void console_line_init(console_line_t *line);

/** Whether Improv is armed in @p improv_session: odd sessions are. */
bool console_line_improv_armed(uint32_t improv_session);

/**
 * @brief Feed one console byte.
 *
 * Runs the Improv parser on it while Improv is armed, and resets the parser
 * when the session has changed since the last byte.
 *
 * @param line            The line being assembled.
 * @param ch              The byte read.
 * @param improv_session  Arm and disarm edges so far (odd while armed), read
 *                        once per byte.
 * @return The completed line, NUL-terminated, when @p ch is a CR or LF ending
 *         a non-empty line; NULL otherwise. The string is valid until the next
 *         call. Bytes past CONSOLE_LINE_MAX - 1 are dropped.
 */
const char *console_line_feed(console_line_t *line, char ch, uint32_t improv_session);

#endif  // CONSOLE_LINE_H
