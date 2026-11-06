/**
 * @file tts_request.h
 * @brief Build the Gemini TTS request body: pure C, host-tested.
 *
 * Model: `gemini-3.8-flash-tts` (issue #844). `gemini-3.1-flash-tts-preview`
 * is a legacy preview, with a listed shutdown of 2026-11-17. Flash-Lite TTS is
 * not used because it has no Finnish.
 *
 * Three things changed from the preview, per the 3.8 docs (read 2026-11-04,
 * not yet probed against the live API):
 *
 * - **The text is read as a verbatim transcript.** "Directions written into
 *   the text might be spoken aloud", so the old `"<style>: <text>"` prefix
 *   would be. The persona's style goes in the part's `speechMetadata.style`
 *   instead, and the text carries only what is to be said.
 * - **Inline tags use angle brackets.** The preview performed `[sighs]`; 3.8
 *   lists `<sighs>`, `<laugh>`, `<whispers>`, `<gasp>` and asks for English
 *   tags whatever the language. The planner and the voice turn still write the
 *   square-bracket tags of speech_tags.h, and they are translated here, at the
 *   one place that knows which model it is talking to. A square tag with no
 *   3.8 equivalent is dropped rather than risk it being read out.
 * - **The streamed audio is unchanged**: headerless 24 kHz s16le mono PCM
 *   (`audio/l16`) by default, so the base64 stream decoder and the player are
 *   untouched. Only a unary request returns WAV.
 *
 * Building the body from the persona strings with cJSON, instead of
 * snprintf into a fixed buffer, also removes the 576-byte `styled[]` buffer
 * that kept about 190 bytes of spoken text under the fi-1950 style (#847).
 */

#ifndef TTS_REQUEST_H
#define TTS_REQUEST_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Speech-generation model id. */
#define TTS_MODEL "gemini-3.8-flash-tts"

/**
 * @brief Copy @p in to @p out, rewriting each allowed square-bracket tag
 *        (speech_tags.h) to its 3.8 angle-bracket form.
 *
 * A bracketed run with no mapping is removed, along with one adjacent space,
 * so no stage direction reaches the transcript in a form the model may speak.
 *
 * @return Length written, excluding the NUL. @p out is always NUL-terminated
 *         when @p out_len > 0, and the copy stops at a whole UTF-8 sequence.
 */
size_t tts_request_translate_tags(const char *in, char *out, size_t out_len);

/**
 * @brief Build the `:streamGenerateContent` request body.
 *
 * @param style         Delivery style, sent as `speechMetadata.style`; NULL or
 *                      empty omits the field.
 * @param language_code BCP-47, e.g. "fi-FI".
 * @param voice         Prebuilt voice name, e.g. "Schedar".
 * @param text          Line to speak, with speech_tags.h square tags.
 * @return Heap-allocated JSON (free with free()), or NULL on allocation
 *         failure or a NULL argument other than @p style.
 */
char *tts_request_build(const char *style, const char *language_code, const char *voice,
                        const char *text);

#ifdef __cplusplus
}
#endif

#endif /* TTS_REQUEST_H */
