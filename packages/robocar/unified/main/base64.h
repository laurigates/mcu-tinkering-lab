/**
 * @file base64.h
 * @brief Base64 encoding for image data
 */

#ifndef BASE64_H
#define BASE64_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Calculate the size needed for base64 encoded data
 * @param input_length Length of input data
 * @return Required buffer size for base64 output
 */
size_t base64_encode_length(size_t input_length);

/**
 * @brief Encode data to base64
 *
 * @param input          Input data buffer
 * @param input_length   Length of input data
 * @param output         Output buffer
 * @param[in,out] inout_length
 *        In:  capacity of @p output in bytes. Must be at least
 *             base64_encode_length(input_length), or the call fails with -1
 *             rather than overrunning the buffer.
 *        Out: encoded length, excluding the terminating NUL.
 *
 * The dual role is documented explicitly because reading this parameter as
 * output-only — as the previous comment implied — leads straight to passing an
 * uninitialised size_t and getting a spurious failure or a truncated encode.
 *
 * @return 0 on success, -1 on error (NULL argument or insufficient capacity)
 */
int base64_encode(const uint8_t *input, size_t input_length, char *output, size_t *inout_length);

/**
 * @brief Encode data to base64 with memory allocation
 * @param input Input data buffer
 * @param input_length Length of input data
 * @return Allocated base64 string (caller must free), NULL on error
 */
char *base64_encode_alloc(const uint8_t *input, size_t input_length);

/* =========================================================================
 * Streaming decoder
 *
 * Gemini TTS returns ~48 kB/s of 24 kHz PCM as a single base64 string inside
 * `candidates[0].content.parts[N].inlineData.data`.  A 4 s utterance is
 * ~256 kB of base64 — 16x the 16 kB HTTP response buffer in gemini_backend.c
 * — so it can never be buffered whole.  This decoder consumes arbitrary
 * chunk boundaries (a base64 quartet may straddle two TCP reads) and emits
 * decoded bytes via a sink callback as they become available.
 *
 * It also locates the payload itself: the decoder scans for the `"data"` key
 * and decodes only the string that follows, ignoring the surrounding JSON.
 * This avoids pulling a streaming JSON parser in for a response whose shape
 * is fixed and known.
 *
 * **Multiple payloads per body.**  Against `:streamGenerateContent?alt=sse`
 * the response is not one JSON object but a sequence of SSE events, each a
 * complete object carrying its own `"data"` payload (233 of them for a ~15 s
 * utterance).  The decoder therefore resumes seeking after each closing quote
 * and concatenates every payload it finds, which is exactly the PCM stream in
 * order.  The SSE framing keyword is a bare `data:` with no quotes, so it
 * cannot satisfy the quote-delimited key match and is skipped like any other
 * non-payload byte.
 * ========================================================================= */

/**
 * @brief Sink for decoded bytes. Return false to abort the decode.
 *
 * Called from the HTTP event context — must not block for long.
 */
typedef bool (*base64_sink_fn)(const uint8_t *data, size_t len, void *ctx);

/** Internal decoder state. Zero-initialise, then feed chunks. */
typedef struct {
    uint8_t phase;      /**< 0=seeking "data" key, 1=in payload, 2=between  */
    uint8_t key_match;  /**< progress through the literal `"data"`       */
    uint8_t quartet[4]; /**< carry for a partial 4-char group            */
    uint8_t quartet_len;
    bool saw_colon; /**< consumed the `:` after the key              */
} base64_stream_t;

/** Reset a decoder to its initial (seeking) state. */
void base64_stream_init(base64_stream_t *st);

/**
 * @brief Feed one chunk of the HTTP body through the decoder.
 *
 * Safe to call with partial quartets and with the `"data"` key split across
 * chunk boundaries. Decoded bytes are handed to @p sink as they complete.
 *
 * @return 0 on success, -1 if the sink aborted or the input was malformed.
 */
int base64_stream_feed(base64_stream_t *st, const char *chunk, size_t len, base64_sink_fn sink,
                       void *ctx);

/**
 * @brief True when the decoder is *between* payloads — at least one payload's
 *        closing quote consumed, and not currently inside another.
 *
 * Not an end-of-stream signal: on a multi-payload SSE body this flips back to
 * false as each subsequent payload opens. The transport, not the decoder,
 * decides when the response is finished.
 */
bool base64_stream_done(const base64_stream_t *st);

#endif  // BASE64_H
