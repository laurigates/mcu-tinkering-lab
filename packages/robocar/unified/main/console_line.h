/**
 * @file console_line.h
 * @brief Serial console line assembly, with Improv packets kept out of it.
 *
 * command_task() reads the USB-Serial-JTAG console one byte at a time, and
 * ESP Web Tools speaks Improv Serial over the same port. Each byte goes to the
 * Improv parser first; this module then builds console lines from the bytes
 * the parser did not claim. A byte it claimed (improv_wifi_process_byte()
 * returned true) drops the partial line and is never stored, so:
 *
 *  - a Send Wi-Fi settings packet, which carries the password in plaintext,
 *    never becomes a line, and so is never logged as `Serial cmd:` (whence the
 *    UART0 mirror and the GlitchTip breadcrumb ring) or dispatched;
 *  - a 0x0A or 0x0D inside a packet (a length byte, a checksum) does not end a
 *    line, so no fragment of the packet is dispatched either, such as a
 *    one-byte checksum of 'F' read as "drive forward".
 *
 * No ESP-IDF dependency, so test_console_line.c runs it on the host together
 * with the real Improv parser.
 */

#ifndef CONSOLE_LINE_H
#define CONSOLE_LINE_H

#include <stdbool.h>
#include <stddef.h>

/* Longest serial console line, NUL included. 32 until issue #626: `mqtt auth
 * <user> <pass>` needs room for a 32-character username and a 64-character
 * password. Every handler parses with width-bounded sscanf formats, so the
 * longer line is safe for all of them. */
#define CONSOLE_LINE_MAX 128

typedef struct {
    char buf[CONSOLE_LINE_MAX];
    size_t len;
} console_line_t;

/** Start with an empty line. */
void console_line_init(console_line_t *line);

/**
 * @brief Feed one console byte.
 *
 * @param line          The line being assembled.
 * @param ch            The byte read.
 * @param improv_owned  What improv_wifi_process_byte() returned for it, or
 *                      false when Improv is not armed.
 * @return The completed line, NUL-terminated, when @p ch is a CR or LF ending
 *         a non-empty line; NULL otherwise. The string is valid until the next
 *         call. Bytes past CONSOLE_LINE_MAX - 1 are dropped.
 */
const char *console_line_feed(console_line_t *line, char ch, bool improv_owned);

#endif  // CONSOLE_LINE_H
