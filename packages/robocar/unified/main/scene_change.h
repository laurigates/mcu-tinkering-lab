/**
 * @file scene_change.h
 * @brief Has the view changed since the robot last remarked on it?
 *
 * The problem this closes
 * ----------------------
 * speech_budget.h rations *how often* the robot may speak; it cannot tell
 * whether there is anything to speak about.  On a bench with no motors the view
 * never changes, so a rationed robot still narrates the same unchanging desk —
 * just less often.  The planner prompt asks the model to "remark only if this
 * frame shows something worth remarking on", but every request is stateless: the
 * model has never seen the previous frame and cannot answer that.
 *
 * Nothing in the firmware could answer it either, until this.  So the question
 * gets answered where the information actually is — on-device, by comparing
 * consecutive frames — and the answer gates the `speak` tool the same way the
 * budget does.
 *
 * How it works
 * ------------
 * frame_stats_log() already decodes every planner frame to a 1/8-scale
 * thumbnail to log luma statistics.  That same decode produces the fingerprint
 * here, so the detector costs a few hundred integer operations and no extra
 * JPEG work.  The thumbnail is reduced to SCENE_BLOCKS block means, and each
 * block is stored as its deviation from the *frame* mean.
 *
 * Removing the frame mean is the load-bearing part.  The sensor's AGC/AEC loops
 * rewrite gain and exposure every frame (see camera.c), so a completely static
 * scene drifts in absolute brightness continuously — an absolute comparison
 * would report "the scene changed!" every time a cloud passed the window, which
 * is precisely the false positive that would reintroduce the chattering this
 * exists to stop.  Deviations from the frame mean are invariant to that uniform
 * shift and still register anything that moves within the frame.
 *
 * The reference is the frame as it was WHEN THE ROBOT LAST SPOKE
 * -------------------------------------------------------------
 * Not the previous frame.  The question worth asking is "is there anything new
 * since I last said something", and comparing consecutive frames answers a
 * different one: a scene that drifts slowly would never trip a per-frame
 * threshold, and a scene that changed once would read as unchanged on the very
 * next frame — before the robot had said a word about it.
 *
 * A consequence worth stating plainly: pointed at a genuinely static scene, this
 * robot goes quiet and stays quiet.  That is the intended behaviour, not a
 * fault — `voice` prints the live score so a silent robot can be told apart from
 * a broken one.
 *
 * Concurrency
 * -----------
 * Free of FreeRTOS and ESP-IDF so the block maths is unit-testable on the host.
 * The planner task is the only writer; the console reads the score and threshold
 * to print them.  A torn read misreports one status line and nothing else.
 */

#ifndef SCENE_CHANGE_H
#define SCENE_CHANGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Fingerprint grid. 8x6 keeps the 4:3 aspect of the frame and divides the
 *  40x30 thumbnail exactly (5x5 pixels per block) — coarse enough that sensor
 *  noise averages out, fine enough that an object entering one corner moves
 *  several blocks. */
#define SCENE_BLOCKS_X 8
#define SCENE_BLOCKS_Y 6
#define SCENE_BLOCKS (SCENE_BLOCKS_X * SCENE_BLOCKS_Y)

/** Mean absolute block difference (luma units) at or above which the scene
 *  counts as changed.
 *
 *  A starting point, NOT a measured constant — the right value depends on the
 *  room, the lens and how much the sensor's own noise survives block averaging,
 *  none of which can be known from here. Tune it by watching the `scene=` field
 *  in the per-frame log and setting `voice scene <n>` from the console. */
#define SCENE_CHANGE_THRESHOLD_DEFAULT 8

/** One frame's coarse appearance: per-block luma deviation from the frame mean.
 *
 *  Deliberately a value type — the planner builds one on its stack per frame and
 *  hands it over, so there is no question of which frame a stored pointer refers
 *  to. 48 bytes. */
typedef struct {
    int8_t block[SCENE_BLOCKS];
    bool valid; /**< false when the frame could not be decoded. */
} scene_fingerprint_t;

/**
 * @brief Reduce an 8-bit luma plane to a fingerprint.
 *
 * @param luma  Row-major luma samples, @p w * @p h bytes.
 * @param w,h   Plane dimensions. Need not divide evenly by the block grid;
 *              block bounds are computed proportionally.
 * @param out   Written unconditionally — `valid` is false for a degenerate
 *              input (NULL plane, zero dimension) so a caller cannot mistake an
 *              all-zero fingerprint for a uniformly grey frame.
 */
void scene_fingerprint_from_luma(const uint8_t *luma, int w, int h, scene_fingerprint_t *out);

/**
 * @brief Mean absolute per-block difference between two fingerprints, in luma
 *        units (0 = identical).
 *
 * Returns 0 if either is invalid — "cannot tell" reads as "no change", which
 * keeps an undecodable frame from triggering speech.
 */
unsigned scene_fingerprint_distance(const scene_fingerprint_t *a, const scene_fingerprint_t *b);

/** @brief Reset to the default threshold and forget both frames. */
void scene_change_init(void);

/**
 * @brief Record the current frame's fingerprint.
 *
 * Call once per planner frame, before the request that may carry a `speak`
 * call. An invalid fingerprint is ignored: a frame nobody could decode is not
 * evidence that the scene did or did not change.
 */
void scene_change_note(const scene_fingerprint_t *fp);

/**
 * @brief Whether the current frame differs enough from the last spoken-about
 *        one to be worth remarking on.
 *
 * True before anything has been spoken (nothing to compare against yet, and the
 * robot's first observation is by definition new) and whenever the threshold is
 * 0, which disables the gate entirely.
 */
bool scene_change_novel(void);

/** @brief Distance between the current frame and the reference, for logging. */
unsigned scene_change_score(void);

/**
 * @brief Adopt the current frame as the reference.
 *
 * Call when an utterance actually reaches the speech queue — that is the moment
 * the robot has "remarked on" this view.
 */
void scene_change_mark_spoken(void);

/** @brief Set the change threshold; 0 disables the gate (always novel). */
void scene_change_set_threshold(uint8_t threshold);

/** @brief Current threshold. */
uint8_t scene_change_threshold(void);

#ifdef __cplusplus
}
#endif

#endif /* SCENE_CHANGE_H */
