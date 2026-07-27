/**
 * @file speech_tags.h
 * @brief Inline delivery tags in spoken text — the allowed set, and the filter.
 *
 * Gemini TTS performs bracketed tags inside the transcript — `[whispers]`,
 * `[sighs]`, `[laughs]` — rather than reading them aloud. That is a second
 * expressive axis alongside the persona's `tts_style` directive, and unlike the
 * directive it can differ *within* an utterance.
 *
 * Where tags come from
 * --------------------
 * The planner emits them, not this module. A tag has to fit what is being said
 * — `[laughs]` belongs on a wry observation and nowhere near a fault report —
 * and only the model generating the sentence knows that. So the persona carries
 * a `tts_tag_brief` naming the allowed tags, the generation prompt passes it on,
 * and the model places them. Contrast the openers/shapes pools in
 * dialogue_style.h, which are drawn at random *because* they are interchangeable
 * by construction; tags are not.
 *
 * Why a filter is still needed
 * ----------------------------
 * A model told it may write `[sighs]` will eventually write `[robottiäänellä]`
 * or `[pauses dramatically]`. An unrecognised tag is not silently ignored by the
 * TTS model — it is liable to be read out as words, so the robot announces its
 * own stage directions. Everything reaching the speaker is therefore filtered
 * against the allow-list below, at the single choke point in
 * speech_queue_post().
 *
 * Deliberately free of ESP-IDF so it can be unit-tested on the host.
 */

#ifndef SPEECH_TAGS_H
#define SPEECH_TAGS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tags the TTS model is known to perform, exactly as they must be
 *        written. Verified against the live API (2026-07).
 *
 * Kept short on purpose. Every entry is one the model may reach for, so a long
 * list is a louder invitation to decorate; these are the ones that suit a robot
 * remarking on its surroundings.
 */
extern const char *const SPEECH_TAG_ALLOWED[];
extern const size_t SPEECH_TAG_ALLOWED_COUNT;

/**
 * @brief Most tags kept in one utterance; the rest are stripped.
 *
 * A model that discovers the mechanism tends to over-use it, and a one-sentence
 * line with four stage directions in it sounds like a parody rather than a
 * character. Two is enough to colour a sentence.
 */
#define SPEECH_TAG_MAX_PER_LINE 2

/**
 * @brief Filter bracketed runs in @p text in place, keeping only allowed tags.
 *
 * Removes any `[...]` whose contents are not in SPEECH_TAG_ALLOWED, and any
 * allowed tag beyond SPEECH_TAG_MAX_PER_LINE. Whitespace left behind by a
 * removal is collapsed so the text does not acquire double spaces. An unclosed
 * `[` is treated as a removal to the end of the string — a truncated tag would
 * otherwise be spoken.
 *
 * @return Number of bracketed runs removed (0 means the text was already clean).
 */
size_t speech_tags_sanitize(char *text);

/**
 * @brief Copy @p in to @p out with *all* bracketed runs removed.
 *
 * For consumers that want the words alone: the recent-openings avoid-list (an
 * opening of "[sighs] Jaahas" would otherwise teach the model to avoid the tag
 * rather than the word) and any human-facing log line.
 *
 * @param out     Destination; always NUL-terminated when out_len > 0.
 * @return Length written, excluding the NUL.
 */
size_t speech_tags_strip(const char *in, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif /* SPEECH_TAGS_H */
