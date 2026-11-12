/**
 * @file voice_endpoint.h
 * @brief When has the person finished speaking? End-of-speech detection for a
 *        hands-free voice turn.
 *
 * ## Why a fixed window was wrong both ways (issue #616)
 *
 * VAD turns used to record a fixed 3.5 s. A short question then waited out the
 * whole window before anything was sent, and a long one was cut off mid-word.
 * This module ends the recording once the speaker has gone quiet instead.
 *
 * ## The rule
 *
 * Each recorded frame is classified as speech when its level is at least
 * @c margin_db above the ambient noise floor. The floor is ambient_audio's, in
 * the same units as ambient_fingerprint_t::level_db, and it is frozen for the
 * turn because the listener does not run while the turn holds the microphone —
 * so speech is measured against the room as it was before anyone spoke, never
 * against a floor that has crept up to meet the voice.
 *
 * The recording ends:
 *
 *   - with END_SILENCE once @c silence_ms has passed since the last speech frame
 *     AND at least @c min_ms has been recorded, or
 *   - with END_MAX at @c max_ms, which the caller sets from the clip buffer it
 *     actually has (the memory bound, not a judgement about speech).
 *
 * begin() counts as speech: a turn is only started because speech was just
 * detected, and the pre-roll in front of the recording holds it. Without that, a
 * speaker who pauses for breath straight after the cue would be cut off at
 * @c min_ms.
 *
 * Every comparison is an unsigned difference on the caller's uint32 millisecond
 * clock, so it holds across the wrap at day 49 — pinned by a host test, because
 * the wrong version either ends every turn at once or never ends one on silence.
 *
 * Pure C with the clock and levels injected; test/test_voice_endpoint.c builds it
 * on the host.
 */

#ifndef VOICE_ENDPOINT_H
#define VOICE_ENDPOINT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Quiet time after the last speech frame that ends the turn. Inside the
 *  600-800 ms band issue #616 proposed: long enough to survive the pause between
 *  "Teuvo," and the question, short enough that the reply does not feel late.
 *  A starting point to tune with `voice endpoint`, not a measured constant. */
#define VOICE_ENDPOINT_SILENCE_MS_DEFAULT 700u

/** Least recording after the cue, so a turn whose trigger was the whole of a
 *  short word still captures whatever follows the beep. */
#define VOICE_ENDPOINT_MIN_MS_DEFAULT 1000u

/** dB above the noise floor that counts as speech. Half the VAD trigger's 12 dB
 *  loudness threshold: the trigger has to reject the room, the endpointer only
 *  has to notice that someone is still talking, and unstressed syllables sit
 *  well below a stressed onset. 0 counts every frame as speech, so the turn
 *  always runs to @c max_ms: a fixed window at the ceiling, which is longer
 *  than the 3.5 s window this module replaced. */
#define VOICE_ENDPOINT_MARGIN_DB_DEFAULT 6u

typedef struct {
    uint32_t min_ms;     /**< Least recording before silence may end it. */
    uint32_t max_ms;     /**< Hard ceiling; the memory bound. */
    uint32_t silence_ms; /**< Quiet after the last speech frame that ends it. */
    uint8_t margin_db;   /**< dB above the floor that counts as speech. */
} voice_endpoint_cfg_t;

typedef enum {
    VOICE_ENDPOINT_CONTINUE = 0,
    VOICE_ENDPOINT_END_SILENCE,
    VOICE_ENDPOINT_END_MAX,
} voice_endpoint_verdict_t;

typedef struct {
    voice_endpoint_cfg_t cfg;
    uint32_t start_ms;       /**< Clock at begin(). */
    uint32_t last_speech_ms; /**< Clock at the last speech frame (begin() counts). */
    uint32_t speech_frames;  /**< Frames classified as speech, for the log line. */
} voice_endpoint_t;

/** @brief The defaults above, with @p max_ms as the ceiling. */
voice_endpoint_cfg_t voice_endpoint_default_cfg(uint32_t max_ms);

/** @brief Whether a frame at @p level_db is speech against @p floor_db. */
bool voice_endpoint_is_speech(int16_t level_db, int16_t floor_db, uint8_t margin_db);

/** @brief Start a recording at @p now_ms. */
void voice_endpoint_begin(voice_endpoint_t *ep, const voice_endpoint_cfg_t *cfg, uint32_t now_ms);

/**
 * @brief Fold in one recorded frame and say whether to stop.
 *
 * @param level_db  This frame's level (ambient_fingerprint_t::level_db).
 * @param floor_db  The noise floor (ambient_audio_floor_db()).
 * @param now_ms    Clock at the END of the frame.
 */
voice_endpoint_verdict_t voice_endpoint_update(voice_endpoint_t *ep, int16_t level_db,
                                               int16_t floor_db, uint32_t now_ms);

/** @brief Short name for a verdict, for the log line. */
const char *voice_endpoint_verdict_name(voice_endpoint_verdict_t v);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_ENDPOINT_H */
