/**
 * @file test_log_mirror_core.c
 * @brief Host tests for the UART0 log mirror's line rendering (issue #798).
 *
 * The properties under test: a rendered line never exceeds its buffer, a line
 * that had to be cut says so and still ends a line, and every "\n" reaches the
 * wire as "\r\n" exactly once. These are what a bench cannot stage on demand —
 * a log line long enough to overflow the hook's stack buffer is rare, and a
 * missing NUL or an off-by-one there writes past the end of a task's stack.
 */

#include "log_mirror_core.h"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* The hook receives a va_list; this is the shape of that call. */
static size_t render(char *buf, size_t cap, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    size_t n = log_mirror_format(buf, cap, fmt, ap);
    va_end(ap);
    return n;
}

#define GUARD 0x5A

/* A buffer with guard bytes past cap, so an overrun shows up as a changed
 * guard rather than as silent corruption. */
typedef struct {
    char buf[300];
} padded_t;

static void pad_init(padded_t *p)
{
    memset(p->buf, GUARD, sizeof(p->buf));
}

static int guard_intact(const padded_t *p, size_t cap)
{
    for (size_t i = cap; i < sizeof(p->buf); i++) {
        if ((unsigned char)p->buf[i] != GUARD) {
            return 0;
        }
    }
    return 1;
}

/* An ordinary ESP_LOG line: formatted, LF expanded to CRLF. */
static void test_plain_line_gets_crlf(void)
{
    char buf[64];
    size_t n = render(buf, sizeof(buf), "I (%u) %s: %s\n", 1234u, "main", "ready");
    ASSERT(n == strlen("I (1234) main: ready\r\n"));
    ASSERT(strcmp(buf, "I (1234) main: ready\r\n") == 0);
}

/* Text without a newline (a partial vprintf) passes through unchanged. */
static void test_no_newline_passes_through(void)
{
    char buf[32];
    size_t n = render(buf, sizeof(buf), "abc%d", 7);
    ASSERT(n == 4);
    ASSERT(strcmp(buf, "abc7") == 0);
}

/* An existing CRLF is not doubled; every bare LF is expanded. */
static void test_existing_crlf_not_doubled(void)
{
    char buf[32];
    size_t n = render(buf, sizeof(buf), "a\r\nb\nc\n");
    ASSERT(strcmp(buf, "a\r\nb\r\nc\r\n") == 0);
    ASSERT(n == strlen("a\r\nb\r\nc\r\n"));
}

/* A leading LF (i == 0, no predecessor) is expanded too. */
static void test_leading_newline(void)
{
    char buf[16];
    size_t n = render(buf, sizeof(buf), "\nx");
    ASSERT(n == 3);
    ASSERT(strcmp(buf, "\r\nx") == 0);
}

/* A line that exactly fills the buffer after expansion is not truncated. */
static void test_exact_fit_not_truncated(void)
{
    /* "abcdef\n" -> "abcdef\r\n" is 8 bytes + NUL = 9. */
    char buf[9];
    size_t n = render(buf, sizeof(buf), "abcdef\n");
    ASSERT(n == 8);
    ASSERT(strcmp(buf, "abcdef\r\n") == 0);
}

/* One byte too long after expansion: cut and marked, within the buffer. */
static void test_expansion_overflow_truncates(void)
{
    /* "abcdefg\n" -> 9 bytes + NUL = 10 > 9. */
    padded_t p;
    pad_init(&p);
    size_t n = render(p.buf, 9, "abcdefg\n");
    ASSERT(n == 8);
    ASSERT(strcmp(p.buf, "abc" LOG_MIRROR_TRUNC_MARK) == 0);
    ASSERT(guard_intact(&p, 9));
}

/* A long line: the result fills the buffer, ends with the marker, and never
 * writes past cap. */
static void test_long_line_truncated_and_bounded(void)
{
    char longmsg[600];
    memset(longmsg, 'x', sizeof(longmsg) - 1);
    longmsg[sizeof(longmsg) - 1] = '\0';

    padded_t p;
    pad_init(&p);
    const size_t cap = 160;
    size_t n = render(p.buf, cap, "I (1) tag: %s\n", longmsg);
    ASSERT(n == cap - 1);
    ASSERT(p.buf[n] == '\0');
    ASSERT(strlen(p.buf) == n);
    ASSERT(strcmp(p.buf + n - strlen(LOG_MIRROR_TRUNC_MARK), LOG_MIRROR_TRUNC_MARK) == 0);
    ASSERT(strncmp(p.buf, "I (1) tag: xxx", 14) == 0);
    ASSERT(guard_intact(&p, cap));
}

/* Truncation counts the expansion of the LFs it keeps: a body of newlines
 * cut mid-way still fits, and every kept LF is a CRLF. */
static void test_truncation_counts_kept_newlines(void)
{
    padded_t p;
    pad_init(&p);
    const size_t cap = 16;
    size_t n = render(p.buf, cap, "%s", "\n\n\n\n\n\n\n\n\n\n\n\n");
    ASSERT(n <= cap - 1);
    ASSERT(p.buf[n] == '\0');
    ASSERT(guard_intact(&p, cap));
    /* (cap - 1) - 5 marker bytes = 10 bytes of body, i.e. five "\r\n". */
    ASSERT(strcmp(p.buf, "\r\n\r\n\r\n\r\n\r\n" LOG_MIRROR_TRUNC_MARK) == 0);
}

/* A cut landing inside an existing CRLF drops the whole pair rather than
 * keeping a lone CR in front of the marker or overrunning the buffer. */
static void test_truncation_through_existing_crlf(void)
{
    padded_t p;
    pad_init(&p);
    const size_t cap = 8;
    size_t n = render(p.buf, cap, "ab\r\ncdefgh");
    ASSERT(n <= cap - 1);
    ASSERT(guard_intact(&p, cap));
    ASSERT(strcmp(p.buf, "ab" LOG_MIRROR_TRUNC_MARK) == 0);
}

/* The smallest accepted buffer still yields a bounded, marked line. */
static void test_min_cap(void)
{
    padded_t p;
    pad_init(&p);
    size_t n = render(p.buf, LOG_MIRROR_MIN_CAP, "%s", "0123456789");
    ASSERT(n == LOG_MIRROR_MIN_CAP - 1);
    ASSERT(strcmp(p.buf, "01" LOG_MIRROR_TRUNC_MARK) == 0);
    ASSERT(guard_intact(&p, LOG_MIRROR_MIN_CAP));
}

/* Inputs the hook must survive without sending anything. */
static void test_rejects_bad_inputs(void)
{
    padded_t p;
    pad_init(&p);
    ASSERT(render(NULL, 64, "x\n") == 0);
    ASSERT(render(p.buf, LOG_MIRROR_MIN_CAP - 1, "x\n") == 0);
    ASSERT(guard_intact(&p, 0));
    ASSERT(render(p.buf, 0, "x\n") == 0);
    ASSERT(guard_intact(&p, 0));
    ASSERT(render(p.buf, 64, "%s", "") == 0);
}

/* A NULL format is rejected before the formatter sees it. */
static size_t render_null_fmt(char *buf, size_t cap, ...)
{
    va_list ap;
    va_start(ap, cap);
    size_t n = log_mirror_format(buf, cap, NULL, ap);
    va_end(ap);
    return n;
}

static void test_null_fmt(void)
{
    char buf[32];
    ASSERT(render_null_fmt(buf, sizeof(buf), 0) == 0);
}

int main(void)
{
    printf("=== UART0 log mirror line-rendering host tests ===\n\n");

    test_run("plain_line_gets_crlf", test_plain_line_gets_crlf);
    test_run("no_newline_passes_through", test_no_newline_passes_through);
    test_run("existing_crlf_not_doubled", test_existing_crlf_not_doubled);
    test_run("leading_newline", test_leading_newline);
    test_run("exact_fit_not_truncated", test_exact_fit_not_truncated);
    test_run("expansion_overflow_truncates", test_expansion_overflow_truncates);
    test_run("long_line_truncated_and_bounded", test_long_line_truncated_and_bounded);
    test_run("truncation_counts_kept_newlines", test_truncation_counts_kept_newlines);
    test_run("truncation_through_existing_crlf", test_truncation_through_existing_crlf);
    test_run("min_cap", test_min_cap);
    test_run("rejects_bad_inputs", test_rejects_bad_inputs);
    test_run("null_fmt", test_null_fmt);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
