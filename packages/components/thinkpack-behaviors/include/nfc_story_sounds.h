/**
 * @file nfc_story_sounds.h
 * @brief Parser + dispatcher for LLM-generated NFC "story sound" sequences.
 *
 * Brainbox returns a short JSON document describing a sequence of audio
 * actions for a tagged story fragment; Finderbox parses it offline and
 * walks the sequence, invoking an application-supplied dispatch callback
 * for each step.  The parser is intentionally tiny and hand-rolled — no
 * cJSON dependency, safe on the host for unit tests.
 *
 * Supported JSON shape:
 * @code
 *   {"sequence":[
 *     {"kind":"tone","param":440},
 *     {"kind":"wait","param":120},
 *     {"kind":"clip","param":3}
 *   ]}
 * @endcode
 *
 * Grammar (whitespace between tokens is permitted):
 *   root       = '{' '"sequence"' ':' '[' step (',' step)* ']' '}'
 *   step       = '{' '"kind"' ':' '"' KIND '"' ',' '"param"' ':' INT '}'
 *   KIND       = "tone" | "clip" | "wait"
 *   INT        = signed decimal integer, fits in int32_t
 *
 * Any deviation (bad keys, unexpected token, overflow, too many steps)
 * fails the parse cleanly and leaves @p out zero-initialised.
 */

#ifndef NFC_STORY_SOUNDS_H
#define NFC_STORY_SOUNDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum steps in a parsed story sequence. */
#define NFC_STORY_SOUNDS_MAX_STEPS 16

/** Step kinds understood by the dispatcher. */
typedef enum {
    NFC_STORY_KIND_TONE = 0, /**< param = frequency in Hz (e.g. 440) */
    NFC_STORY_KIND_CLIP = 1, /**< param = chatterbox clip_id to broadcast */
    NFC_STORY_KIND_WAIT = 2, /**< param = delay in milliseconds */
} nfc_story_kind_t;

/** One parsed step. */
typedef struct {
    uint8_t kind;  /**< @ref nfc_story_kind_t */
    int32_t param; /**< kind-specific integer parameter */
} story_step_t;

/** Parsed sequence. */
typedef struct {
    story_step_t steps[NFC_STORY_SOUNDS_MAX_STEPS];
    uint8_t count; /**< Valid entries in steps[] */
} story_sequence_t;

/**
 * @brief Dispatch callback invoked once per step during execute.
 *
 * The callback is called synchronously from @ref nfc_story_sounds_execute
 * and is free to block (e.g. for WAIT steps).  Return a non-zero value to
 * abort the remaining sequence.
 */
typedef int (*audio_dispatch_fn)(uint8_t kind, int32_t param, void *user_ctx);

/**
 * @brief Parse @p json into @p out.
 *
 * @param json  JSON text (not required to be NUL-terminated).
 * @param len   Number of bytes in @p json.
 * @param out   Destination.  Zeroed on entry and left zeroed on failure.
 * @return true if the sequence was fully parsed, false on any error.
 */
bool nfc_story_sounds_parse(const char *json, size_t len, story_sequence_t *out);

/**
 * @brief Invoke @p dispatch once for each step in @p seq.
 *
 * Stops early and returns the dispatcher's non-zero return value if any
 * step rejects the dispatch; otherwise returns 0 once all steps have fired.
 */
int nfc_story_sounds_execute(const story_sequence_t *seq, audio_dispatch_fn dispatch,
                             void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* NFC_STORY_SOUNDS_H */
