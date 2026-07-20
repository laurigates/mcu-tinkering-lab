/**
 * @file test_base64_stream.c
 * @brief Host tests for the streaming base64 decoder used by the TTS path.
 *
 * The decoder consumes an HTTP body in whatever chunks the TCP stack hands
 * over, so the property that actually matters is *chunk-size independence*:
 * feeding the same body one byte at a time, in awkward 3-byte slices, or in
 * one shot must all produce byte-identical PCM. Several tests therefore sweep
 * every chunk size rather than picking one — a quartet or the `"data"` key
 * straddling a boundary is exactly the bug that would ship as intermittent
 * audio corruption.
 */

#include "base64.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * Test harness
 * ========================================================================= */

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

/* =========================================================================
 * Sink
 * ========================================================================= */

#define SINK_CAP 8192

typedef struct {
    uint8_t buf[SINK_CAP];
    size_t len;
    size_t abort_after; /* 0 = never abort */
    int call_count;
} sink_t;

static bool collect(const uint8_t *data, size_t len, void *ctx)
{
    sink_t *s = (sink_t *)ctx;
    s->call_count++;

    if (s->abort_after && s->len >= s->abort_after) {
        return false;
    }
    ASSERT(s->len + len <= SINK_CAP);
    memcpy(s->buf + s->len, data, len);
    s->len += len;
    return true;
}

/** Feed @p body through the decoder in fixed-size slices. */
static void feed_chunked(const char *body, size_t chunk, sink_t *sink)
{
    base64_stream_t st;
    base64_stream_init(&st);

    const size_t total = strlen(body);
    for (size_t off = 0; off < total; off += chunk) {
        const size_t n = (off + chunk > total) ? (total - off) : chunk;
        base64_stream_feed(&st, body + off, n, collect, sink);
    }
}

/* =========================================================================
 * Tests
 * ========================================================================= */

/* "SGVsbG8=" decodes to "Hello" — padded, non-multiple-of-3 length. */
static void test_decodes_simple_payload(void)
{
    const char *body = "{\"inlineData\":{\"mimeType\":\"audio/L16\",\"data\":\"SGVsbG8=\"}}";
    sink_t sink = {0};
    feed_chunked(body, strlen(body), &sink);

    ASSERT(sink.len == 5);
    ASSERT(memcmp(sink.buf, "Hello", 5) == 0);
}

/* The decoder must ignore everything before the payload and stop at the
 * closing quote — trailing JSON must not leak into the audio. */
static void test_stops_at_closing_quote(void)
{
    const char *body = "{\"data\":\"SGVsbG8=\",\"other\":\"QUJD\"}";
    sink_t sink = {0};
    feed_chunked(body, strlen(body), &sink);

    ASSERT(sink.len == 5);
    ASSERT(memcmp(sink.buf, "Hello", 5) == 0);
}

/* The core property: identical output at every chunk size, including 1 (which
 * splits every quartet) and sizes that split the `"data"` key itself. */
static void test_chunk_size_independence(void)
{
    const char *body =
        "{\"candidates\":[{\"content\":{\"parts\":[{\"inlineData\":{\"mimeType\":\"audio/"
        "L16;rate=24000\",\"data\":\"VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVyIHRoZSBsYXp5IGRvZw==\""
        "}}]}}]}";
    const char *expect = "The quick brown fox jumps over the lazy dog";
    const size_t expect_len = strlen(expect);

    for (size_t chunk = 1; chunk <= strlen(body); chunk++) {
        sink_t sink = {0};
        feed_chunked(body, chunk, &sink);

        if (sink.len != expect_len || memcmp(sink.buf, expect, expect_len) != 0) {
            printf("     chunk=%zu produced %zu bytes: %.*s\n", chunk, sink.len, (int)sink.len,
                   sink.buf);
        }
        ASSERT(sink.len == expect_len);
        ASSERT(memcmp(sink.buf, expect, expect_len) == 0);
    }
}

/* Binary payload with embedded NULs and high bytes — PCM is not text, and a
 * decoder that treated it as a C string would truncate at the first zero. */
static void test_binary_payload_with_nuls(void)
{
    /* base64 of the 6 bytes {0x00, 0xFF, 0x00, 0x80, 0x7F, 0x00} */
    const char *body = "{\"data\":\"AP8AgH8A\"}";
    const uint8_t expect[6] = {0x00, 0xFF, 0x00, 0x80, 0x7F, 0x00};

    for (size_t chunk = 1; chunk <= strlen(body); chunk++) {
        sink_t sink = {0};
        feed_chunked(body, chunk, &sink);
        ASSERT(sink.len == 6);
        ASSERT(memcmp(sink.buf, expect, 6) == 0);
    }
}

/* Whitespace and newlines inside the string are skipped, not decoded. */
static void test_skips_embedded_whitespace(void)
{
    const char *body = "{\"data\":\"SGVs\n  bG8=\"}";
    sink_t sink = {0};
    feed_chunked(body, strlen(body), &sink);

    ASSERT(sink.len == 5);
    ASSERT(memcmp(sink.buf, "Hello", 5) == 0);
}

/* `"inlineData"` contains the substring `Data` but must not match the
 * quote-delimited `"data"` key — otherwise the decoder would start consuming
 * the mimeType value as audio. */
static void test_does_not_match_inlinedata_key(void)
{
    const char *body = "{\"inlineData\":{\"mimeType\":\"audio/L16\",\"data\":\"QUJD\"}}";
    sink_t sink = {0};
    feed_chunked(body, strlen(body), &sink);

    ASSERT(sink.len == 3);
    ASSERT(memcmp(sink.buf, "ABC", 3) == 0);
}

/* An error response carries no "data" key — decode nothing rather than
 * emitting garbage into the speaker. */
static void test_error_body_yields_nothing(void)
{
    const char *body = "{\"error\":{\"code\":429,\"message\":\"Resource exhausted\"}}";
    sink_t sink = {0};
    feed_chunked(body, strlen(body), &sink);

    ASSERT(sink.len == 0);
    ASSERT(sink.call_count == 0);
}

/* A sink returning false must halt decoding — this is the backpressure abort
 * path used when playback stalls. */
static void test_sink_abort_halts_decode(void)
{
    const char *body = "{\"data\":\"VGhlIHF1aWNrIGJyb3duIGZveCBqdW1wcyBvdmVy\"}";
    sink_t sink = {0};
    sink.abort_after = 6;

    base64_stream_t st;
    base64_stream_init(&st);
    const int ret = base64_stream_feed(&st, body, strlen(body), collect, &sink);

    ASSERT(ret == -1);
    ASSERT(sink.len < 30); /* stopped well before the full payload */
}

/* done() reports true only once the payload's closing quote is consumed. */
static void test_done_flag(void)
{
    const char *body = "{\"data\":\"QUJD\"}";
    base64_stream_t st;
    base64_stream_init(&st);
    sink_t sink = {0};

    ASSERT(!base64_stream_done(&st));
    base64_stream_feed(&st, body, 10, collect, &sink); /* mid-payload */
    ASSERT(!base64_stream_done(&st));
    base64_stream_feed(&st, body + 10, strlen(body) - 10, collect, &sink);
    ASSERT(base64_stream_done(&st));
}

/* NULL guards. */
static void test_null_args(void)
{
    base64_stream_t st;
    base64_stream_init(&st);
    sink_t sink = {0};

    ASSERT(base64_stream_feed(NULL, "x", 1, collect, &sink) == -1);
    ASSERT(base64_stream_feed(&st, NULL, 1, collect, &sink) == -1);
    ASSERT(base64_stream_feed(&st, "x", 1, NULL, &sink) == -1);
    base64_stream_init(NULL); /* must not crash */
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== base64 streaming decoder host tests ===\n\n");

    test_run("decodes_simple_payload", test_decodes_simple_payload);
    test_run("stops_at_closing_quote", test_stops_at_closing_quote);
    test_run("chunk_size_independence", test_chunk_size_independence);
    test_run("binary_payload_with_nuls", test_binary_payload_with_nuls);
    test_run("skips_embedded_whitespace", test_skips_embedded_whitespace);
    test_run("does_not_match_inlinedata_key", test_does_not_match_inlinedata_key);
    test_run("error_body_yields_nothing", test_error_body_yields_nothing);
    test_run("sink_abort_halts_decode", test_sink_abort_halts_decode);
    test_run("done_flag", test_done_flag);
    test_run("null_args", test_null_args);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
