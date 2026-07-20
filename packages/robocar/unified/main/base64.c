/**
 * @file base64.c
 * @brief Base64 encoding implementation
 */

#include "base64.h"
#include <stdlib.h>
#include <string.h>

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode_length(size_t input_length)
{
    return ((input_length + 2) / 3) * 4 + 1;  // +1 for null terminator
}

int base64_encode(const uint8_t *input, size_t input_length, char *output, size_t *output_length)
{
    if (!input || !output || !output_length) {
        return -1;
    }

    size_t required_length = base64_encode_length(input_length);
    if (*output_length < required_length) {
        return -1;
    }

    size_t i = 0;
    size_t j = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];

    while (i < input_length) {
        char_array_3[0] = input[i++];
        char_array_3[1] = (i < input_length) ? input[i++] : 0;
        char_array_3[2] = (i < input_length) ? input[i++] : 0;

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (int k = 0; k < 4; k++) {
            output[j++] = base64_chars[char_array_4[k]];
        }
    }

    // Handle padding
    if (input_length % 3 == 1) {
        output[j - 2] = '=';
        output[j - 1] = '=';
    } else if (input_length % 3 == 2) {
        output[j - 1] = '=';
    }

    output[j] = '\0';
    *output_length = j;
    return 0;
}

char *base64_encode_alloc(const uint8_t *input, size_t input_length)
{
    if (!input) {
        return NULL;
    }

    size_t output_length = base64_encode_length(input_length);
    char *output = malloc(output_length);
    if (!output) {
        return NULL;
    }

    if (base64_encode(input, input_length, output, &output_length) != 0) {
        free(output);
        return NULL;
    }

    return output;
}

/* =========================================================================
 * Streaming decoder
 * ========================================================================= */

/** The JSON key whose string value carries the PCM payload.
 *
 *  Matched case-sensitively and *with* its surrounding quotes, which is what
 *  keeps it from also matching the enclosing `"inlineData"` object (capital
 *  D, and no quote immediately before `data`).
 */
static const char BASE64_DATA_KEY[] = "\"data\"";
#define BASE64_DATA_KEY_LEN 6

/** Reverse base64 alphabet. Returns 0-63, or -1 for any non-alphabet char. */
static int b64_val(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

/**
 * Decode whatever is held in the carry and hand it to the sink.
 *
 * A full quartet yields 3 bytes; a trailing 3 yields 2; a trailing 2 yields 1.
 * A trailing 1 is malformed base64 and yields nothing (silently dropped —
 * losing one sample at the tail of an utterance is not worth failing over).
 */
static bool flush_quartet(base64_stream_t *st, base64_sink_fn sink, void *ctx)
{
    uint8_t out[3];
    size_t n = 0;

    if (st->quartet_len >= 2) {
        out[0] = (uint8_t)((st->quartet[0] << 2) | (st->quartet[1] >> 4));
        n = 1;
    }
    if (st->quartet_len >= 3) {
        out[1] = (uint8_t)((st->quartet[1] << 4) | (st->quartet[2] >> 2));
        n = 2;
    }
    if (st->quartet_len == 4) {
        out[2] = (uint8_t)((st->quartet[2] << 6) | st->quartet[3]);
        n = 3;
    }

    st->quartet_len = 0;
    return (n == 0) ? true : sink(out, n, ctx);
}

void base64_stream_init(base64_stream_t *st)
{
    if (st) {
        memset(st, 0, sizeof(*st));
    }
}

bool base64_stream_done(const base64_stream_t *st)
{
    return st && st->phase == 2;
}

int base64_stream_feed(base64_stream_t *st, const char *chunk, size_t len, base64_sink_fn sink,
                       void *ctx)
{
    if (!st || !chunk || !sink) {
        return -1;
    }

    for (size_t i = 0; i < len; i++) {
        const char c = chunk[i];

        if (st->phase == 2) {
            return 0;  // payload already complete; ignore the JSON tail
        }

        if (st->phase == 0) {
            if (st->key_match < BASE64_DATA_KEY_LEN) {
                if (c == BASE64_DATA_KEY[st->key_match]) {
                    st->key_match++;
                } else {
                    // A mismatch may still be the start of the next candidate.
                    st->key_match = (c == '"') ? 1 : 0;
                }
                continue;
            }

            // Key matched: consume `:` then the opening quote of the value.
            if (!st->saw_colon) {
                if (c == ':') {
                    st->saw_colon = true;
                } else if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
                    st->key_match = 0;  // "data" appeared somewhere else
                }
                continue;
            }
            if (c == '"') {
                st->phase = 1;
            }
            continue;
        }

        // phase 1 — inside the payload string.
        if (c == '"') {
            if (!flush_quartet(st, sink, ctx)) {
                return -1;
            }
            st->phase = 2;
            return 0;
        }
        if (c == '=') {
            continue;  // padding; the closing quote triggers the flush
        }

        const int v = b64_val(c);
        if (v < 0) {
            continue;  // whitespace or line breaks inside the string
        }

        st->quartet[st->quartet_len++] = (uint8_t)v;
        if (st->quartet_len == 4 && !flush_quartet(st, sink, ctx)) {
            return -1;
        }
    }

    return 0;
}
