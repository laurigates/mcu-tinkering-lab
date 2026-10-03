/**
 * @file speech_trigger.h
 * @brief Is someone talking? The hands-free (VAD) trigger, shaped like speech
 *        rather than like loudness.
 *
 * ## The problem this closes (issue #617)
 *
 * The VAD trigger used to fire on ambient_audio's broadband loudness excursion,
 * so a door slam, a dropped object or the robot's own motors started a voice
 * turn as readily as a voice did. A false turn is not free: it beeps, uploads a
 * clip and spends a request on a model that answers `__IGNORE__`.
 *
 * ## The rule
 *
 * A frame is speech-shaped when BOTH hold:
 *
 *   - **Voice-band share.** At least @c share_pct percent of the frame's energy
 *     (after DC removal) lies in roughly 300-3400 Hz, measured by a 2nd-order
 *     Butterworth high-pass at 300 Hz cascaded with a 2nd-order low-pass at
 *     3400 Hz. Voiced speech puts most of its energy in the formant region;
 *     broadband noise and a click (flat spectrum) leave over half of theirs
 *     outside it — white noise measures about 40% — and motor rumble sits
 *     below it.
 *   - **Above the floor.** The frame's level is at least @c margin_db above
 *     ambient_audio's adaptive noise floor, in the same units
 *     (ambient_fingerprint_t::level_db), so a quiet room's own murmur does not
 *     qualify.
 *
 * The trigger fires once consecutive speech-shaped frames have covered
 * @c sustain_ms (150 ms: three 64 ms frames). That duration is what rejects a
 * transient: a slam or a click lasts one frame. It does not reject a steady
 * in-band tone (a whistle, a beep) that outlasts it — a host test pins that
 * deliberately, so nobody mistakes the duration for a tonality check.
 *
 * Frame duration is derived from the sample count, and a gap of more than
 * SPEECH_TRIGGER_MAX_GAP_MS between notes breaks the run (the listener is locked
 * out while a voice turn records, and speech before that gap is not speech now).
 * The gap test is an unsigned difference, so it holds across the uint32
 * millisecond wrap.
 *
 * ## Thresholds
 *
 * All three are console knobs (`voice trigger`) and none persists, for the same
 * reason as every other voice threshold here: what counts as speech in this room,
 * at this distance, with this mic, is a judgement made standing in front of the
 * robot. A share or margin of 0 drops that check.
 *
 * ## Cost
 *
 * Two biquads over a 1024-sample frame, ~15 times a second: about 30 k
 * multiply-accumulate steps per second on a core with an FPU. No FFT.
 *
 * Pure C with module state, like ambient_audio.c; test/test_speech_trigger.c
 * builds it on the host. Single writer (the listener task); the console only
 * reads the scores.
 */

#ifndef SPEECH_TRIGGER_H
#define SPEECH_TRIGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Sample rate the filter coefficients are computed for. Must match
 *  MIC_SAMPLE_RATE_HZ; restated, as in ambient_audio.h, so this module needs no
 *  firmware header. */
#define SPEECH_TRIGGER_SAMPLE_RATE_HZ 16000

/** Voice-band edges, Hz. */
#define SPEECH_TRIGGER_BAND_LO_HZ 300.0f
#define SPEECH_TRIGGER_BAND_HI_HZ 3400.0f

/** Least voice-band share of frame energy, percent. Between white noise (~40%)
 *  and synthetic voiced speech (~90% in the host test); a starting point to
 *  tune with `voice trigger`, not a measured constant. */
#define SPEECH_TRIGGER_SHARE_PCT_DEFAULT 65u

/** Least level above the noise floor, whole dB. Below the old 12 dB loudness
 *  trigger, because the band-share test now does the rejecting the loudness
 *  threshold used to attempt alone, and speech at a metre is not loud. */
#define SPEECH_TRIGGER_MARGIN_DB_DEFAULT 9u

/** Speech-shaped audio needed before the trigger fires, ms. */
#define SPEECH_TRIGGER_SUSTAIN_MS_DEFAULT 150u

/** A pause between notes longer than this breaks the run, ms. A little under
 *  four 64 ms frames: one late frame must not reset it, a locked-out listener
 *  must. */
#define SPEECH_TRIGGER_MAX_GAP_MS 250u

/** @brief Reset thresholds and state to the defaults. */
void speech_trigger_init(void);

/**
 * @brief Percentage (0-100) of a frame's energy inside the voice band.
 *
 * Pure. DC is removed first; the filters start from rest for every frame, so
 * the result depends on this frame alone. 0 for silence, NULL, or n == 0.
 */
uint8_t speech_trigger_band_share_pct(const int16_t *pcm, size_t n);

/**
 * @brief Fold in one accepted microphone frame.
 *
 * @param pcm       The frame (mono, SPEECH_TRIGGER_SAMPLE_RATE_HZ).
 * @param n         Samples in it.
 * @param level_db  Its level (ambient_fingerprint_t::level_db).
 * @param floor_db  ambient_audio_floor_db().
 * @param now_ms    Caller's monotonic clock.
 * @return true while the current run of speech-shaped frames has lasted at
 *         least the sustain time.
 */
bool speech_trigger_note(const int16_t *pcm, size_t n, int16_t level_db, int16_t floor_db,
                         uint32_t now_ms);

/** @brief Break the current run — call for any frame not passed to note()
 *         (quarantined, failed, under the start cue) and after a trigger fires,
 *         so the next turn needs fresh speech. */
void speech_trigger_reset_run(void);

/** @brief Length of the current speech-shaped run, ms. */
uint32_t speech_trigger_run_ms(void);

/** @brief Voice-band share of the last noted frame, percent. */
uint8_t speech_trigger_last_share_pct(void);

/**
 * @brief For tuning in the room — speak, then read `mic`: the longest run since
 *        the last call with @p clear, and the highest voice-band share of any
 *        frame that cleared the floor margin in that time.
 *
 * The share is taken over frames that failed the share test too, so a threshold
 * set too high shows up as a peak just below it rather than as nothing at all.
 * Either pointer may be NULL.
 */
void speech_trigger_best(uint32_t *run_ms, uint8_t *peak_share_pct, bool clear);

void speech_trigger_set_share_pct(uint8_t pct);
void speech_trigger_set_margin_db(uint8_t db);
void speech_trigger_set_sustain_ms(uint32_t ms);
uint8_t speech_trigger_share_pct(void);
uint8_t speech_trigger_margin_db(void);
uint32_t speech_trigger_sustain_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEECH_TRIGGER_H */
