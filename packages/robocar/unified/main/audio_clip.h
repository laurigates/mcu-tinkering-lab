/**
 * @file audio_clip.h
 * @brief Sizing, WAV framing and normalisation for a recorded microphone clip.
 *
 * ## Why this is a separate, pure module
 *
 * Two jobs that a bench will never exercise the failing case of, and that are
 * both miserable to debug where they actually break:
 *
 * 1. **The size arithmetic.** `seconds * rate * 2 * 4 / 3` is the classic
 *    silent-overflow shape: a 32-bit wrap yields a *small* allocation, the
 *    encoder then writes past it, and the crash lands somewhere else entirely.
 *    No plausible console input triggers it, so it cannot be found by using the
 *    robot — only by calling the function with the input directly.
 *
 * 2. **The WAV header.** Gemini rejects raw PCM (see below), so every clip must
 *    carry a canonical 44-byte header. A field-width or byte-order slip there
 *    does not fail locally: it produces an HTTP 400 from a remote server whose
 *    message names no field. Pinning the 44 bytes in a host test turns a
 *    remote, opaque failure into a local, obvious one.
 *
 * ## The API rejects audio/pcm — this is measured, not assumed
 *
 * Probed live against `models/gemini-flash-latest:generateContent` (2026-07)
 * with a 0.5 s 16 kHz mono tone as an `inlineData` part:
 *
 *   - `mimeType: "audio/wav"`  -> 200, replied, usage reported 13 AUDIO tokens.
 *   - `mimeType: "audio/pcm"`  -> **HTTP 400 "Request contains an invalid
 *     argument."**, naming no field.
 *
 * So the firmware wraps. The header is fixed-size and trivially built for a
 * known rate/width/channel count, which is why doing it on-device costs
 * essentially nothing — 44 bytes prepended to a buffer that is already hundreds
 * of kB.
 *
 * Audio is cheap in tokens (~26 tokens/second measured), so clip length is
 * bounded by MEMORY, not by API cost. See AUDIO_CLIP_MAX_BYTES.
 */

#ifndef AUDIO_CLIP_H
#define AUDIO_CLIP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Canonical PCM WAV header length. Not configurable — it is the format. */
#define AUDIO_CLIP_WAV_HEADER_BYTES 44

/** Default record window. A knob (`listen <secs>`) because "is four seconds
 *  long enough to finish a sentence" is a judgement only someone talking to the
 *  robot can make. */
#define AUDIO_CLIP_WINDOW_MS_DEFAULT 4000

/** Console-accepted window bounds, in seconds. */
#define AUDIO_CLIP_WINDOW_S_MIN 1
#define AUDIO_CLIP_WINDOW_S_MAX 8

/**
 * @brief Hard ceiling on raw PCM bytes for one clip.
 *
 * 8 s at 16 kHz mono 16-bit = 256 000 bytes. The base64 of that plus its header
 * is ~341 kB, and cJSON duplicates the string once, so the transient peak is
 * roughly 430 kB of PSRAM if — and only if — the caller frees in the order
 * voice_turn.c documents. That fits alongside the camera framebuffers and the
 * 512 kB TTS ring; much beyond it does not.
 */
#define AUDIO_CLIP_MAX_BYTES 256000u

/** Per-clip measurements, all computed in one pass. */
typedef struct {
    int32_t dc;       /**< Mean sample value before removal. Large = a DC-biased mic. */
    int16_t peak;     /**< Largest absolute sample after DC removal. */
    uint32_t clipped; /**< Samples at or beyond full scale. */
    uint32_t samples; /**< Samples examined. */
} audio_clip_stats_t;

/**
 * @brief Bytes of raw PCM for a window, or 0 if the request is out of bounds.
 *
 * Returns 0 — never a truncated or wrapped value — for a window that would
 * exceed AUDIO_CLIP_MAX_BYTES or overflow the arithmetic. A caller that treats
 * 0 as "refuse" cannot be handed a short buffer to overrun.
 */
size_t audio_clip_pcm_bytes(uint32_t window_ms, uint32_t sample_rate_hz);

/**
 * @brief Total base64 length for a WAV-wrapped clip of @p pcm_bytes, or 0.
 *
 * Accounts for the 44-byte header, which is the whole reason this is a function
 * and not an expression at the call site: forgetting the header under-allocates
 * by exactly 60 base64 characters, which corrupts only the tail of the audio and
 * therefore looks like a microphone problem rather than an arithmetic one.
 * Returns 0 on overflow.
 */
size_t audio_clip_b64_length(size_t pcm_bytes);

/**
 * @brief Write a canonical 44-byte PCM WAV header into @p out.
 *
 * @param out         Destination, at least AUDIO_CLIP_WAV_HEADER_BYTES.
 * @param pcm_bytes   Payload size that will follow the header.
 * @param sample_rate_hz  Sample rate.
 * @param channels    Channel count (1 here).
 * @param bits        Bits per sample (16 here).
 * @return true on success, false on a NULL pointer or a degenerate parameter.
 */
bool audio_clip_wav_header(uint8_t *out, size_t pcm_bytes, uint32_t sample_rate_hz,
                           uint16_t channels, uint16_t bits);

/**
 * @brief Remove DC and measure the clip, in place.
 *
 * The stats are the point, not the DC removal: `peak` and `clipped` are the
 * audio analogue of what `cam` prints for gain and exposure, and they are the
 * only way to tell a mis-set microphone gain from a bad model reply. A clip that
 * comes back with peak ~200 was never going to be transcribed no matter what the
 * model said.
 *
 * A large `dc` is itself diagnostic: PDM microphones carry a substantial DC
 * term, and leaving it in wastes headroom and biases every downstream measure.
 */
void audio_clip_normalise(int16_t *pcm, size_t samples, audio_clip_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_CLIP_H
