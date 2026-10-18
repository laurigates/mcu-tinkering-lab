/**
 * @file log_mirror_core.h
 * @brief Render one ESP_LOG line into a bounded buffer for the UART0 mirror
 *        (issue #798).
 *
 * Pure C, no ESP-IDF dependency — see .claude/rules/testing.md's shared-core
 * pattern. log_mirror.c is the ESP-IDF half: it installs the esp_log vprintf
 * hook and hands this module's output to the UART0 driver.
 *
 * The hook runs on whatever task logged, so the buffer lives on that task's
 * stack and has to stay small. A line longer than the buffer is cut, not
 * dropped, and the cut is marked: it ends in "...\r\n", so a reader on J15 can
 * tell a truncated line from a short one and the next line still starts on a
 * line of its own.
 *
 * Line endings: the USB-Serial-JTAG console turns "\n" into "\r\n" in its VFS
 * (CONFIG_NEWLIB_STDOUT_LINE_ENDING_CRLF). The UART0 mirror writes straight to
 * the driver with no VFS in between, so this module does the same expansion
 * itself — otherwise a plain terminal on a USB-serial adapter shows a
 * staircase. A "\r\n" already in the text is left alone, not doubled.
 */

#ifndef LOG_MIRROR_CORE_H
#define LOG_MIRROR_CORE_H

#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Smallest buffer log_mirror_format() will write into: the "...\r\n"
 *  truncation marker, its NUL, and room for at least a couple of characters. */
#define LOG_MIRROR_MIN_CAP 8

/** The suffix a truncated line ends with. */
#define LOG_MIRROR_TRUNC_MARK "...\r\n"

/**
 * Format @p fmt / @p ap into @p buf and expand every bare "\n" to "\r\n".
 *
 * The result always fits in @p cap bytes including a terminating NUL, and the
 * return value is its length (NUL not counted) — the number of bytes to send.
 * When the expanded text would not fit, as much of it as fits is kept and the
 * line ends in LOG_MIRROR_TRUNC_MARK instead.
 *
 * Returns 0, sending nothing, when @p buf or @p fmt is NULL, when @p cap is
 * below LOG_MIRROR_MIN_CAP, when the formatted text is empty, or when the
 * formatter reports an error. Consumes @p ap: a caller that needs the
 * arguments again must pass a va_copy().
 */
size_t log_mirror_format(char *buf, size_t cap, const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif  // LOG_MIRROR_CORE_H
