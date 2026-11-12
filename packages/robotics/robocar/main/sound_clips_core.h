/**
 * @file sound_clips_core.h
 * @brief Prebuilt sound clips: their names, the `[tag]` that plays one inside a
 *        spoken line, and where in the line it plays.
 *
 * The robot carries a few short clips in flash (sounds/manifest.toml, built by
 * tools/build-sounds.py into main/sounds/<tag>.pcm). Three things can play one, and
 * all three go through speech_queue so a clip never fights a sentence for the
 * speaker:
 *
 *   - the model writes `[horn]` inside a `speak` line, beside `[sighs]`;
 *   - the planner calls its `sound` tool, which gemini_parse turns into a line
 *     holding just the tag;
 *   - the obstacle reflex posts `[horn]` when it stops the robot mid-drive
 *     (sound_clips_reflex_due() below).
 *
 * The TTS task then splits the line (sound_clips_split()): the tag comes out,
 * the words go to Gemini TTS, and the clip is written into the playback ring
 * before or after them. A tag never reaches the TTS request, where it would be
 * read out as a word. A line that is only a tag makes no request at all, so a
 * honk costs nothing and works without WiFi.
 *
 * Deliberately free of ESP-IDF so it can be unit-tested on the host; the PCM
 * itself lives in sound_clips.c.
 */

#ifndef SOUND_CLIPS_CORE_H
#define SOUND_CLIPS_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The clips in flash. The order matches SOUND_CLIP_NAMES. */
typedef enum {
    SOUND_CLIP_NONE = -1,
    SOUND_CLIP_HORN = 0, /**< polite two-toot car horn */
    SOUND_CLIP_AOOGAH,   /**< vintage klaxon */
    SOUND_CLIP_BELL,     /**< bicycle bell, ring-ring */
    SOUND_CLIP_BOING,    /**< cartoon spring boing */
    SOUND_CLIP_COUNT,
} sound_clip_t;

/** Bare names ("horn"), as the planner's `sound` tool takes them. */
extern const char *const SOUND_CLIP_NAMES[SOUND_CLIP_COUNT];

/** Most clips kept in one spoken line; speech_tags_sanitize() drops the rest. */
#define SOUND_CLIP_MAX_PER_LINE 1

/** Shortest gap between two reflex honks. A person standing in front of the
 *  robot can trip and release the reflex every few hundred milliseconds; one
 *  honk per encounter is polite, a honk per trip is not. */
#define SOUND_REFLEX_COOLDOWN_MS 20000u

/** Name of @p clip ("horn"), or NULL for SOUND_CLIP_NONE or out of range. */
const char *sound_clip_name(sound_clip_t clip);

/** Clip whose bare name is @p name, or SOUND_CLIP_NONE. Case-sensitive. */
sound_clip_t sound_clip_from_name(const char *name);

/** Clip whose tag is exactly the @p len bytes at @p p ("[horn]"), or
 *  SOUND_CLIP_NONE. */
sound_clip_t sound_clip_from_tag(const char *p, size_t len);

/** Where a line's clip plays relative to its words. */
typedef struct {
    sound_clip_t lead;  /**< plays before the words, or SOUND_CLIP_NONE */
    sound_clip_t trail; /**< plays after the words, or SOUND_CLIP_NONE */
} sound_split_t;

/**
 * @brief Take every sound tag out of @p text in place and say where the clip
 *        plays.
 *
 * Only the first sound tag plays; any others are removed with it. It leads when
 * nothing but whitespace and delivery tags precede it (`[horn] Pardon me`,
 * `[sighs] [horn] ...`), and trails otherwise: a sentence is fetched and played
 * as one stream, so a clip cannot be spliced into its middle, and the end is
 * the nearest place to where the model put it. Whitespace left by a removal is
 * collapsed and the result trimmed, so a line that was only a tag comes back as
 * "".
 */
sound_split_t sound_clips_split(char *text);

/**
 * @brief Should the obstacle reflex honk now?
 *
 * @param now_ms    Monotonic milliseconds.
 * @param last_ms   When the reflex last honked.
 * @param honked    False until the first honk; then last_ms is meaningful.
 * @param blocking  The reflex is holding back motion that was asked for: a
 *                  manual lease, or a fresh goal that moves. A reflex that trips
 *                  because someone walked up to a robot standing still has
 *                  nothing to ask them to move for.
 *
 * Wrap-safe across the uint32 millisecond rollover at day 49.
 */
bool sound_clips_reflex_due(uint32_t now_ms, uint32_t last_ms, bool honked, bool blocking);

#ifdef __cplusplus
}
#endif

#endif /* SOUND_CLIPS_CORE_H */
