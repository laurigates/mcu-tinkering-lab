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
 * @param input Input data buffer
 * @param input_length Length of input data
 * @param output Output buffer (must be large enough)
 * @param output_length Pointer to store actual output length
 * @return 0 on success, -1 on error
 */
int base64_encode(const uint8_t *input, size_t input_length, char *output, size_t *output_length);

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
 * ========================================================================= */

/**
 * @brief Sink for decoded bytes. Return false to abort the decode.
 *
 * Called from the HTTP event context — must not block for long.
 */
typedef bool (*base64_sink_fn)(const uint8_t *data, size_t len, void *ctx);

/** Internal decoder state. Zero-initialise, then feed chunks. */
typedef struct {
    uint8_t phase;      /**< 0=seeking "data" key, 1=in payload, 2=done  */
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

/** True once the payload's closing quote has been consumed. */
bool base64_stream_done(const base64_stream_t *st);

#endif  // BASE64_H
