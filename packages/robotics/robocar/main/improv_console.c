/**
 * @file improv_console.c
 * @brief Improv Serial over the USB-Serial-JTAG console (issue #644).
 *
 * See improv_console.h for why both directions bypass line-ending translation.
 *
 * Why not install the usb_serial_jtag driver and call
 * usb_serial_jtag_write_bytes(): switching the console VFS to driver mode makes
 * stdin reads block (unless O_NONBLOCK is set), which would stop command_task's 1 Hz Improv
 * announcement (it runs between getchar() calls that currently return EOF), and it changes how
 * every log line on the board is transmitted. This file leaves the console in its default
 * non-driver mode and only borrows it for one write at a time.
 */

#include "improv_console.h"

#include <stdio.h>
#include <unistd.h>

#include "driver/usb_serial_jtag_vfs.h"
#include "sdkconfig.h"

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#error "improv_console.c assumes the primary console is USB-Serial-JTAG; with a UART console, \
use the improv-wifi component's UART0 default writer and install the UART driver instead"
#endif

/* The TX translation the console runs with, restored after each raw write.
 * The VFS has no getter, so this mirrors its own DEFAULT_TX_MODE derivation
 * (esp_driver_usb_serial_jtag/src/usb_serial_jtag_vfs.c, ESP-IDF v5.4). */
#if CONFIG_NEWLIB_STDOUT_LINE_ENDING_CRLF
#define CONSOLE_TX_LINE_ENDINGS ESP_LINE_ENDINGS_CRLF
#elif CONFIG_NEWLIB_STDOUT_LINE_ENDING_CR
#define CONSOLE_TX_LINE_ENDINGS ESP_LINE_ENDINGS_CR
#elif CONFIG_NEWLIB_STDOUT_LINE_ENDING_LF
#define CONSOLE_TX_LINE_ENDINGS ESP_LINE_ENDINGS_LF
#else
#error "no CONFIG_NEWLIB_STDOUT_LINE_ENDING_* set; check the libc line-ending Kconfig names"
#endif

void improv_console_raw_input(void)
{
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
}

void improv_console_write(const uint8_t *data, size_t len)
{
    /* Every printf and ESP_LOG line goes through the stdout stream and takes
     * its lock, so holding it (and stderr's) means no other writer is inside
     * the VFS while the TX mode is switched. Flush first so text already
     * buffered goes out before the packet, translated as it was written. */
    flockfile(stdout);
    flockfile(stderr);
    fflush(stdout);
    fflush(stderr);

    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    /* write(), not fwrite(): straight to the VFS in one call, no stdio buffer
     * to split the packet across a line-buffer flush. With no host reading,
     * the VFS drops the bytes after 50 ms, which is the right outcome. */
    (void)write(fileno(stdout), data, len);
    /* The non-driver VFS only flushes the 64-byte TX FIFO when it writes a
     * '\n' (usb_serial_jtag_tx_char_no_driver), and a packet need not contain
     * or end with one. Without this the tail of a reply waits in the FIFO for
     * the next log line. fsync() reaches usb_serial_jtag_wait_tx_done_no_driver
     * through the console VFS, flushes, and gives up after the same 50 ms. */
    (void)fsync(fileno(stdout));
    usb_serial_jtag_vfs_set_tx_line_endings(CONSOLE_TX_LINE_ENDINGS);

    funlockfile(stderr);
    funlockfile(stdout);
}
