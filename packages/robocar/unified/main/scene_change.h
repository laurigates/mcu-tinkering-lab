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
 * One reference per dwell pose (ADR-029)
 * -------------------------------------
 * The head moves, so "the frame the robot last spoke about" is kept per dwell
 * pose (scene_pose_t): a frame at pan -40 is compared with the last frame
 * spoken about at pan -40. A frame taken between poses cannot be measured and
 * is neither compared nor adopted. A head that stays centred uses only the
 * centre reference, which is the single reference this gate had before.
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

/** The head's dwell poses (ADR-029 §1–2). A camera on a moving head sees a
 *  different view at every pose, so one reference would read every pose change
 *  as "the scene changed". This module and plan_activity.c therefore keep one
 *  reference per dwell pose, and compare a frame only with the last one taken
 *  at the same pose.
 *
 *  CENTRE is 0 on purpose: a zeroed fingerprint, a robot with no servos and a
 *  head that stays forward all land on the centre reference, which is exactly
 *  the single reference these gates had before the poses existed. */
typedef enum {
    SCENE_POSE_CENTRE = 0, /**< pan 0, level */
    SCENE_POSE_LEFT,       /**< pan -SCENE_POSE_SIDE_PAN_DEG, level */
    SCENE_POSE_RIGHT,      /**< pan +SCENE_POSE_SIDE_PAN_DEG, level */
    SCENE_POSE_FLOOR,      /**< pan 0, tilt SCENE_POSE_FLOOR_TILT_DEG */
    SCENE_POSES,
    /** Between dwell poses (mid-slew, tracking, a lease). The frame is
     *  fingerprinted but is "cannot measure": never compared, never adopted. */
    SCENE_POSE_NONE = SCENE_POSES,
} scene_pose_t;

/** Pan of the two side poses, degrees (+ = right). ADR-029's starting value. */
#define SCENE_POSE_SIDE_PAN_DEG 40
/** Tilt of the floor pose, degrees (+ = up). ADR-029's starting value; the
 *  bench sets the real one. */
#define SCENE_POSE_FLOOR_TILT_DEG (-20)
/** How far, in degrees on each axis, the head may sit from a dwell pose and
 *  still count as at it. The servo position is the commanded angle in whole
 *  degrees, and a clamp to the live limits or the slew's last step can leave it
 *  a degree or two short. Wider would let a slewing head's frames through. */
#define SCENE_POSE_TOLERANCE_DEG 3

/**
 * @brief Name the dwell pose a head position is at, or SCENE_POSE_NONE.
 *
 * @param pan_deg   Head pan at capture, degrees, + = right.
 * @param tilt_deg  Head tilt at capture, degrees, + = up.
 */
scene_pose_t scene_pose_from_head(int pan_deg, int tilt_deg);

/** @brief One-letter name for logs: C, L, R, F, or `-` for no dwell pose. */
char scene_pose_letter(uint8_t pose);

/** One frame's coarse appearance: per-block luma deviation from the frame mean,
 *  and the dwell pose it was taken at.
 *
 *  Deliberately a value type — the planner builds one on its stack per frame and
 *  hands it over, so there is no question of which frame a stored pointer refers
 *  to. 50 bytes. */
typedef struct {
    int8_t block[SCENE_BLOCKS];
    bool valid;   /**< false when the frame could not be decoded. */
    uint8_t pose; /**< scene_pose_t; scene_fingerprint_from_luma() sets CENTRE,
                       planner_task sets the pose at capture. */
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

/** @brief Reset to the default threshold and forget the current frame and
 *         every pose's reference. */
void scene_change_init(void);

/**
 * @brief Record the current frame's fingerprint.
 *
 * Call once per planner frame, before the request that may carry a `speak`
 * call. An invalid fingerprint is ignored: a frame nobody could decode is not
 * evidence that the scene did or did not change. A valid one taken off a dwell
 * pose (`pose == SCENE_POSE_NONE`) is recorded, and makes the gate unable to
 * measure until the next on-pose frame.
 */
void scene_change_note(const scene_fingerprint_t *fp);

/**
 * @brief Whether the current frame differs enough from the last frame spoken
 *        about AT THE SAME POSE to be worth remarking on.
 *
 * True whenever the threshold is 0, which disables the gate entirely; that check
 * comes first, so it holds even before any frame has decoded. Otherwise false
 * while no decodable frame has ever been seen: there is no view to have a first
 * impression of, so the gate fails closed (issue #690). False, for the same
 * reason, while the current frame is off a dwell pose. Once an on-pose frame
 * has been seen, true while nothing has been spoken about at that pose
 * (nothing to compare against yet, and a first view of it is by definition new).
 */
bool scene_change_novel(void);

/**
 * @brief Whether scene_change_novel() is, right now, the result of an actual
 *        comparison against a spoken-about frame at the current pose.
 *
 * False while the current pose has no reference (nothing spoken about there
 * yet, or the robot spoke before any decodable frame), while no decodable frame
 * has been seen, while the current frame is off a dwell pose, and while the
 * threshold is 0. In each of those states novel() answers without comparing, so
 * the planner prompt must not describe its answer as "the view has (not)
 * changed since you last spoke" (speech_evidence.h, issue #631).
 */
bool scene_change_compared(void);

/** @brief Distance between the current frame and its pose's reference, for
 *         logging. 0 when the two cannot be compared. */
unsigned scene_change_score(void);

/** @brief The current frame's pose (scene_pose_t), for logging. */
uint8_t scene_change_pose(void);

/**
 * @brief Adopt the current frame as its pose's reference.
 *
 * Call when an utterance actually reaches the speech queue — that is the moment
 * the robot has "remarked on" this view. Only that pose's reference moves; an
 * off-pose or undecoded current frame moves none.
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
