/**
 * @file ambient_audio.h
 * @brief Has the room SOUNDED different since the robot last remarked on it?
 *
 * The problem this closes
 * ----------------------
 * scene_change.h answers "has the view changed"; nothing answers "has anything
 * happened that the camera cannot see". A door slams behind the robot, someone
 * walks in and starts talking, the fan the robot has been staring past finally
 * switches off — none of it moves a single pixel, so a scene-gated robot stays
 * mute through all of it.
 *
 * The planner cannot answer this either. Every request to the model is
 * stateless and carries one image: the model has never heard the room, has no
 * record of what it sounded like a minute ago, and cannot be instructed into
 * knowing. Asking it to "remark when something happens" names a fact it has no
 * channel to (see .claude/rules/stateless-model-gating.md §1). So the question
 * is answered where the information is — on-device, from the microphone — and
 * the answer gates the `speak` tool the same way the budget and the scene do.
 *
 * How it works
 * ------------
 * Two deliberately ORTHOGONAL sub-measures, each of which must be immune to a
 * uniform level change, because the microphone's own gain is not a fact about
 * the room:
 *
 *   (1) LOUDNESS EXCURSION above a slowly-adapting floor. The floor is a
 *       minimum-statistics tracker: it FALLS fast toward the current level
 *       (one-pole, AMBIENT_FLOOR_FALL_TAU_MS) and RISES only at
 *       AMBIENT_FLOOR_RISE_DB_PER_S. A fan that switches on therefore raises
 *       the level, the floor creeps up to meet it over a handful of seconds,
 *       and the excursion decays to zero — the fan STOPS BEING NEWS by
 *       construction, with no separate "is this steady?" test to tune. A 64 ms
 *       door slam is a spike the rise rate cannot follow. HVAC, mic AGC
 *       pumping and the robot's own motors raising the room floor are all
 *       absorbed by the same mechanism. Absolute RMS is never thresholded.
 *
 *   (2) SPECTRAL-SHAPE DISTANCE. Eight band levels in dB, each stored as its
 *       DEVIATION FROM THE MEAN OF THE EIGHT. In the log domain a uniform gain
 *       adds the same constant to every band, and that constant cancels
 *       against the mean — so the shape fingerprint is exactly gain-invariant.
 *       This is the precise analogue of scene_change's per-block deviation
 *       from the frame mean, and it is what still catches "empty room ->
 *       someone talking" long after the loudness excursion has decayed.
 *
 * The bands come from a crude parallel one-pole low-pass bank (cutoffs 125,
 * 250, 500, 1k, 2k, 3k, 5k Hz; each band is the difference of two successive
 * low-pass outputs). We measure CHANGE IN SHAPE, not spectrum, so sloppy
 * overlapping bands are fine and an FFT would buy nothing for the cost.
 *
 * DC is removed before any energy is computed. A PDM microphone carries a
 * large subsonic term; left in, it swamps the lowest band and buries the other
 * seven, and the shape measure degenerates to a constant.
 *
 * The scores are LATCHED PEAKS, not instantaneous readings
 * -------------------------------------------------------
 * This is the one structural difference from scene_change, and it is forced by
 * the sample rates. A camera frame and a planner request are the SAME event, so
 * scene_change can compare whatever frame is current. The microphone produces
 * ~15 frames per second against a 15 s planner period — 200x faster — so an
 * instantaneous reading would let almost every transient in the room fall
 * between two planner cycles unseen. The bang has to still be there when the
 * planner finally asks.
 *
 * So both sub-scores hold their peak since the last mark_spoken(), and expire
 * AMBIENT_LATCH_TTL_MS after the peak was set — otherwise a bang the budget
 * suppressed ten minutes ago would still license a remark now. The TTL is why
 * the clock is injected, and it is what makes the uint32 millisecond wrap at
 * day 49 reachable in a host test instead of only in the field.
 *
 * The loud latch holds a LEVEL, not an excursion, and the score is that level
 * measured against the floor as it stands now. That is what lets the latch and
 * the adaptive floor agree rather than fight: a transient's score survives the
 * floor falling back to the quiet room (it is bounded below by the floor at the
 * moment of the peak), while a steady sound's score is eaten by the floor
 * rising through it. Latching the excursion instead would freeze the fan's
 * first second for the whole TTL and undo the floor entirely.
 *
 * Threshold 0 means NEVER NOVEL here — the polarity is inverted from
 * scene_change
 * -------------------------------------------------------------------------
 * scene_change is AND-ed into may_speak, so its disabled state must be `true`
 * ("this term is not participating"). These two sub-gates are OR-ed in, so
 * their disabled state must be `false` — the same intent, the opposite
 * constant, because the neutral element follows the operator. Getting it
 * backwards makes the robot speak on every cycle the budget permits, which
 * looks like a tuning problem and is not one. Setting BOTH thresholds to 0
 * therefore restores the pre-microphone behaviour exactly: may_speak collapses
 * back to `budget && scene`. That reduction is pinned by a host test.
 *
 * Concurrency
 * -----------
 * Free of FreeRTOS and ESP-IDF so all of the above is unit-testable on the
 * host. The listener task is the only writer (note(), ~15 Hz); the planner task
 * calls novel()/mark_spoken() once per period; the console reads the scores and
 * thresholds to print them. note() only ever RAISES a latch, and novel() reads
 * the timestamp before the value, so the worst a torn read can do is expire a
 * latch one cycle early — it fails toward silence, never toward speech. The
 * console reads may misreport one status line and nothing else.
 */

#ifndef AMBIENT_AUDIO_H
#define AMBIENT_AUDIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of spectral bands in a fingerprint. Eight is enough to tell "hum"
 *  from "speech" from "clatter"; more would only sharpen a measure whose whole
 *  point is to be coarse. */
#define AMBIENT_BANDS 8

/** Samples per analysis frame. 1024 at 16 kHz is 64 ms — long enough for a
 *  meaningful low-band estimate, short enough that a door slam still dominates
 *  the frame it lands in. */
#define AMBIENT_FRAME_SAMPLES 1024

/** Frames shorter than this are refused. Below ~16 ms the 125 Hz band has not
 *  seen a full cycle and its level is an artefact of the filter's start-up. */
#define AMBIENT_MIN_SAMPLES 256

/** Sample rate the band filter coefficients are computed for.
 *
 *  MUST match MIC_SAMPLE_RATE_HZ in pin_config.h. It is restated rather than
 *  included because this module is deliberately free of every firmware header
 *  so it can be built on the host; a mismatch shifts every band edge and makes
 *  the shape measure quietly less discriminating (never wrong, just blunt). */
#define AMBIENT_SAMPLE_RATE_HZ 16000

/** Loudness excursion above the floor, in whole dB, at or above which the room
 *  counts as having done something.
 *
 *  A starting point, NOT a measured constant — it depends on the room, the mic
 *  and how much of the robot's own noise reaches it. Tune from the `loud=`
 *  field in the per-cycle log via `voice loud <db>`. */
#define AMBIENT_LOUD_THRESHOLD_DB_DEFAULT 12

/** Mean per-band shape distance, in whole dB, at or above which the room counts
 *  as sounding different. Same caveat as above; `voice shape <db>`. */
#define AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT 6

/** How long a latched peak stays evidence, in ms.
 *
 *  Chosen to match the speech budget's default minimum gap: one door slam
 *  should license at most one remark, and a gap-length TTL is what stops it
 *  licensing four across four planner cycles. If the gap is changed at runtime
 *  (`voice quiet`), set this alongside it — the derivation is not automatic. */
#define AMBIENT_LATCH_TTL_MS_DEFAULT 60000u

/** How long after the amplifier stops the microphone is still considered
 *  contaminated, in ms. Covers room reverb and the amp's own tail; the robot
 *  hearing its own last word and remarking on it is the failure this prevents. */
#define AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT 500u

/** Rate at which the noise floor may RISE, in dB per second.
 *
 *  The single constant that decides what "steady" means: a sound must outrun
 *  1 dB/s to stay newsworthy. A slam cannot be followed at all; a fan is
 *  absorbed in about (excursion / this) seconds. */
#define AMBIENT_FLOOR_RISE_DB_PER_S 1.0f

/** Time constant for the floor FALLING toward a quieter room, in ms. Fast, so
 *  that the moment a sustained noise stops the floor is back where it belongs
 *  and the next real transient is measured against the quiet room. */
#define AMBIENT_FLOOR_FALL_TAU_MS 200.0f

/** One frame's coarse sound: per-band level deviation from the mean of the
 *  bands, plus the frame's absolute level.
 *
 *  A value type on purpose — the listener builds one per frame on its stack and
 *  hands it over, so there is never a question of which frame a stored pointer
 *  refers to. */
typedef struct {
    int8_t band[AMBIENT_BANDS]; /**< Whole-dB deviation from the mean of the 8. */
    int16_t level_db;           /**< Whole-dB frame level, 0 = one LSB RMS. */
    bool valid;                 /**< false ONLY for degenerate input, never for silence. */
} ambient_fingerprint_t;

/**
 * @brief Reduce a mono 16-bit PCM frame to a fingerprint.
 *
 * @param pcm  Mono samples at AMBIENT_SAMPLE_RATE_HZ.
 * @param n    Sample count. Must be >= AMBIENT_MIN_SAMPLES.
 * @param out  Written unconditionally (zeroed first, `valid` set last) so a
 *             caller cannot mistake an all-zero fingerprint for a failure.
 *
 * Digital silence is a LEGITIMATE reading: it yields valid == true with
 * level_db == 0 (the level is floored at one LSB so log10f never sees zero).
 * `valid` is false only for NULL pcm or a frame that is too short — the audio
 * counterpart of scene_change's undecodable frame.
 */
void ambient_fingerprint_from_pcm(const int16_t *pcm, size_t n, ambient_fingerprint_t *out);

/**
 * @brief Mean absolute per-band difference between two fingerprints, in whole
 *        dB (0 = same shape).
 *
 * Exactly 0 for the same soundscape at a different gain — that invariance is
 * the whole reason the bands are stored as deviations. Returns 0 if either
 * operand is invalid: "cannot tell" reads as "no change", so a run of failed
 * reads cannot trigger speech about nothing.
 */
unsigned ambient_fingerprint_distance(const ambient_fingerprint_t *a,
                                      const ambient_fingerprint_t *b);

/** @brief Reset thresholds, floor and both latches. */
void ambient_audio_init(void);

/**
 * @brief Fold one frame into the floor tracker and the latches.
 *
 * Call for every microphone frame that ambient_capture_allowed() accepted.
 * @p now_ms is the caller's monotonic millisecond clock; it is injected rather
 * than read so the latch TTL — including its behaviour across the uint32 wrap —
 * is testable. An invalid fingerprint is DROPPED: it moves neither the floor
 * nor a latch, because a frame nobody could measure is not evidence either way.
 */
void ambient_audio_note(const ambient_fingerprint_t *fp, uint32_t now_ms);

/**
 * @brief Whether the room has done something worth remarking on since the robot
 *        last spoke.
 *
 * True before anything has been spoken (the first observation is by definition
 * new), false when BOTH thresholds are 0 (see the header note on polarity), and
 * otherwise the OR of the two sub-gates, each requiring its latch to be both
 * over threshold and younger than the TTL.
 */
bool ambient_audio_novel(uint32_t now_ms);

/** @brief Latched loudness excursion above the floor, in whole dB, for logging.
 *         Decays as the floor rises through a steady sound. */
unsigned ambient_audio_loud_score(void);

/** @brief Latched spectral-shape distance from the last spoken-about room, in
 *         whole dB, for logging. */
unsigned ambient_audio_shape_score(void);

/** @brief Current adapted noise floor in whole dB, for logging. A floor that
 *         never moves is the tell that the listener has stopped feeding. */
int16_t ambient_audio_floor_db(void);

/**
 * @brief Adopt the current room as the reference and clear both latches.
 *
 * Call when an utterance actually reaches the speech queue. The reference
 * adopted is the LAST NOTED FINGERPRINT, so a caller that has been holding the
 * microphone (a voice turn) must note a fresh post-turn frame BEFORE calling
 * this — otherwise the reference is the room as it was before the conversation
 * started, and the robot spontaneously remarks that something changed one cycle
 * after answering you.
 */
void ambient_audio_mark_spoken(void);

/** @brief Set the loudness threshold in whole dB; 0 disables that sub-gate
 *         (never contributes true). */
void ambient_audio_set_loud_threshold(uint8_t db);

/** @brief Set the shape threshold in whole dB; 0 disables that sub-gate. */
void ambient_audio_set_shape_threshold(uint8_t db);

/** @brief Current loudness threshold, whole dB. */
uint8_t ambient_audio_loud_threshold(void);

/** @brief Current shape threshold, whole dB. */
uint8_t ambient_audio_shape_threshold(void);

/** @brief Set how long a latched peak stays evidence, in ms. 0 expires every
 *         latch immediately. */
void ambient_audio_set_latch_ttl_ms(uint32_t ms);

/** @brief Current latch TTL, ms. */
uint32_t ambient_audio_latch_ttl_ms(void);

/**
 * @brief Whether a microphone frame arriving now may be measured at all.
 *
 * Pure, and here rather than in the listener task precisely so it can be
 * tested. The microphone hears the MAX98357A perfectly well, so frames captured
 * while the amplifier is active are worthless — and so are the ones just after
 * it stops, because room reverb and the amp's tail outlive @p playback_active.
 *
 * @param playback_active       The amplifier is fetching or playing right now.
 * @param now_ms                Caller's monotonic clock.
 * @param last_playback_end_ms  When playback last went inactive.
 * @param hangover_ms           Quarantine after that instant; 0 disables it.
 *
 * The elapsed comparison is an unsigned difference, so it holds across the
 * uint32 wrap. Getting that wrong deafens the robot for 49 days.
 */
bool ambient_capture_allowed(bool playback_active, uint32_t now_ms, uint32_t last_playback_end_ms,
                             uint32_t hangover_ms);

#ifdef __cplusplus
}
#endif

#endif /* AMBIENT_AUDIO_H */
