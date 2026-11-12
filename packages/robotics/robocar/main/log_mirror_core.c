/**
 * @file log_mirror_core.c
 * @brief Bounded rendering of one log line for the UART0 mirror (issue #798).
 *
 * See log_mirror_core.h. Pure C, compiled unmodified into the firmware and
 * into test/test_log_mirror_core.c.
 */

#include "log_mirror_core.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* True when buf[i] is a "\n" that the wire needs a "\r" in front of. */
static bool is_bare_lf(const char *buf, size_t i)
{
    return buf[i] == '\n' && (i == 0 || buf[i - 1] != '\r');
}

/* Expand every bare LF in buf[0..len) to CRLF, in place, from the end.
 * @p out_len is the expanded length the caller has already measured and
 * checked against the buffer. */
static void expand_in_place(char *buf, size_t len, size_t out_len)
{
    size_t j = out_len;
    for (size_t i = len; i > 0; i--) {
        const size_t k = i - 1;
        buf[--j] = buf[k];
        if (is_bare_lf(buf, k)) {
            buf[--j] = '\r';
        }
    }
}

size_t log_mirror_format(char *buf, size_t cap, const char *fmt, va_list ap)
{
    if (buf == NULL || fmt == NULL || cap < LOG_MIRROR_MIN_CAP) {
        return 0;
    }

    const int n = vsnprintf(buf, cap, fmt, ap);
    if (n <= 0) {
        return 0;
    }

    const size_t limit = cap - 1; /* bytes available before the NUL */
    size_t len = (size_t)n < limit ? (size_t)n : limit;

    if ((size_t)n <= limit) {
        size_t out_len = len;
        for (size_t i = 0; i < len; i++) {
            out_len += is_bare_lf(buf, i) ? 1 : 0;
        }
        if (out_len <= limit) {
            expand_in_place(buf, len, out_len);
            buf[out_len] = '\0';
            return out_len;
        }
        /* Fits as formatted, but not once its LFs are expanded: cut it. */
    }

    /* Keep the longest prefix whose expansion leaves room for the marker. */
    const size_t mark_len = sizeof(LOG_MIRROR_TRUNC_MARK) - 1;
    const size_t budget = limit - mark_len;
    size_t keep = 0;
    size_t keep_out = 0;
    for (size_t i = 0; i < len; i++) {
        const size_t cost = is_bare_lf(buf, i) ? 2 : 1;
        if (keep_out + cost > budget) {
            break;
        }
        keep_out += cost;
        keep = i + 1;
    }
    /* Never leave a lone CR in front of the marker: its LF was cut, and the
     * marker supplies the line ending. */
    if (keep > 0 && buf[keep - 1] == '\r') {
        keep--;
        keep_out--;
    }

    expand_in_place(buf, keep, keep_out);
    memcpy(buf + keep_out, LOG_MIRROR_TRUNC_MARK, mark_len);
    buf[keep_out + mark_len] = '\0';
    return keep_out + mark_len;
}
