/**
 * @file improv_console.h
 * @brief Improv Serial over the USB-Serial-JTAG console (issue #644).
 *
 * This board's console is the USB-Serial-JTAG on the USB-C connector, and
 * that is the port ESP Web Tools opens to provision WiFi. Improv packets are
 * binary, so both directions have to bypass the console's line-ending
 * translation:
 *
 *   - RX: the default CR->LF mapping turns any 0x0D in a request (a 13-byte
 *     SSID or password, or a checksum that happens to be 0x0D) into 0x0A.
 *   - TX: the default LF->CRLF mapping inserts a 0x0D before every 0x0A in a
 *     reply.
 *
 * The shared improv-wifi component defaults to writing UART0, which on the
 * XIAO is D6/D7 — not the console, and not initialised. improv_console_write()
 * is the writer this project hands it instead.
 */

#ifndef IMPROV_CONSOLE_H
#define IMPROV_CONSOLE_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Stop translating CR to LF on console input.
 *
 * Call once, before the console reader starts. command_task already treats
 * both '\r' and '\n' as a line terminator, so typed commands are unaffected.
 */
void improv_console_raw_input(void);

/**
 * @brief Write one Improv packet to the console, byte for byte.
 *
 * Matches improv_wifi_write_fn_t. Holds the stdout and stderr stream locks
 * for the whole write, so a log line from another task cannot be interleaved
 * into the packet, and switches TX line-ending translation off only for the
 * duration of the write.
 */
void improv_console_write(const uint8_t *data, size_t len);

#endif /* IMPROV_CONSOLE_H */
