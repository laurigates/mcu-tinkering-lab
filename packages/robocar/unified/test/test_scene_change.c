/**
 * @file test_scene_change.c
 * @brief Host tests for the scene-change gate.
 *
 * The behaviour under test is a claim about the world — "the robot goes quiet
 * when nothing is happening, and speaks up when something does" — and the two
 * ways it can fail are opposites:
 *
 *   - too sensitive, and the robot chatters at sensor noise and at the AGC
 *     breathing on a motionless scene (the exact failure this module exists to
 *     prevent, so the brightness-shift test below is the load-bearing one);
 *   - too blind, and it never speaks again after its first sentence.
 *
 * Both are cheap to pin down with synthetic luma planes and impossible to
 * reproduce reliably on a bench, where "nothing changed" is never quite true.
 */

#include "scene_change.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Test harness
 * ========================================================================= */

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* =========================================================================
 * Synthetic frames — the real thumbnail geometry (320x240 at 1/8 scale)
 * ========================================================================= */

#define W 40
#define H 30

static uint8_t g_frame[W * H];

/** Flat grey frame at @p level. */
static void fill(uint8_t level)
{
    memset(g_frame, level, sizeof(g_frame));
}

/** Paint an axis-aligned rectangle (in thumbnail pixels) at @p level. */
static void rect(int x0, int y0, int x1, int y1, uint8_t level)
{
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            g_frame[(y * W) + x] = level;
        }
    }
}

static void fingerprint(scene_fingerprint_t *out)
{
    scene_fingerprint_from_luma(g_frame, W, H, out);
}

/* =========================================================================
 * Fingerprint construction
 * ========================================================================= */

static void test_flat_frame_has_no_deviation(void)
{
    scene_fingerprint_t fp;
    fill(128);
    fingerprint(&fp);
    ASSERT(fp.valid);
    for (int i = 0; i < SCENE_BLOCKS; ++i) {
        ASSERT(fp.block[i] == 0);
    }
}

static void test_degenerate_input_is_marked_invalid(void)
{
    /* An all-zero fingerprint is a legitimate reading (a flat frame), so a
     * failure must be distinguishable from it — otherwise an undecodable frame
     * would masquerade as a uniformly grey one and read as "no change". */
    scene_fingerprint_t fp;
    scene_fingerprint_from_luma(NULL, W, H, &fp);
    ASSERT(!fp.valid);
    scene_fingerprint_from_luma(g_frame, 0, H, &fp);
    ASSERT(!fp.valid);
    scene_fingerprint_from_luma(g_frame, W, -1, &fp);
    ASSERT(!fp.valid);
    scene_fingerprint_from_luma(g_frame, W, H, NULL); /* must not crash */
}

static void test_bright_corner_lands_in_the_right_block(void)
{
    /* Top-left block covers x 0..4, y 0..4 at this geometry. Getting the block
     * indexing transposed would still "work" — every test that only measures
     * distance would pass — while comparing different regions of the two frames. */
    scene_fingerprint_t fp;
    fill(100);
    rect(0, 0, 5, 5, 200);
    fingerprint(&fp);

    ASSERT(fp.block[0] > 50);                 /* top-left, well above the mean */
    ASSERT(fp.block[SCENE_BLOCKS_X - 1] < 5); /* top-right, near the mean */
    ASSERT(fp.block[SCENE_BLOCKS - 1] < 5);   /* bottom-right */
}

static void test_odd_geometry_covers_every_pixel(void)
{
    /* The grid need not divide the plane evenly. A block-size-based split would
     * drop the remainder columns silently; proportional bounds must not. */
    static uint8_t odd[37 * 23];
    memset(odd, 60, sizeof(odd));
    odd[(22 * 37) + 36] = 255; /* the very last pixel, in the remainder region */

    scene_fingerprint_t fp;
    scene_fingerprint_from_luma(odd, 37, 23, &fp);
    ASSERT(fp.valid);
    ASSERT(fp.block[SCENE_BLOCKS - 1] > 0); /* it reached the bottom-right block */
}

/* =========================================================================
 * Distance
 * ========================================================================= */

static void test_identical_frames_are_distance_zero(void)
{
    scene_fingerprint_t a;
    scene_fingerprint_t b;
    fill(90);
    rect(10, 10, 20, 20, 180);
    fingerprint(&a);
    fingerprint(&b);
    ASSERT(scene_fingerprint_distance(&a, &b) == 0);
}

static void test_uniform_brightness_shift_is_not_a_change(void)
{
    /* THE test. The sensor's AGC/AEC rewrites gain and exposure every frame, so
     * a motionless scene drifts in absolute brightness continuously. An absolute
     * comparison would call that a scene change and the robot would chatter at
     * a passing cloud — reintroducing exactly what this gate exists to stop. */
    scene_fingerprint_t dim;
    scene_fingerprint_t bright;

    fill(60);
    rect(12, 8, 28, 22, 110);
    fingerprint(&dim);

    fill(140); /* same scene, every pixel +80 */
    rect(12, 8, 28, 22, 190);
    fingerprint(&bright);

    ASSERT(scene_fingerprint_distance(&dim, &bright) == 0);
}

static void test_an_object_moving_registers(void)
{
    scene_fingerprint_t before;
    scene_fingerprint_t after;

    fill(80);
    rect(0, 0, 15, 15, 220); /* object top-left */
    fingerprint(&before);

    fill(80);
    rect(25, 15, 40, 30, 220); /* same object, opposite corner */
    fingerprint(&after);

    ASSERT(scene_fingerprint_distance(&before, &after) >= SCENE_CHANGE_THRESHOLD_DEFAULT);
}

static void test_invalid_operands_read_as_no_change(void)
{
    scene_fingerprint_t good;
    scene_fingerprint_t bad;
    fill(90);
    rect(5, 5, 20, 20, 200);
    fingerprint(&good);
    memset(&bad, 0, sizeof(bad));

    ASSERT(scene_fingerprint_distance(&good, &bad) == 0);
    ASSERT(scene_fingerprint_distance(&bad, &good) == 0);
    ASSERT(scene_fingerprint_distance(NULL, &good) == 0);
    ASSERT(scene_fingerprint_distance(&good, NULL) == 0);
}

/* =========================================================================
 * The gate
 * ========================================================================= */

/** Observe the frame currently painted in g_frame. */
static void observe(void)
{
    scene_fingerprint_t fp;
    fingerprint(&fp);
    scene_change_note(&fp);
}

static void test_first_view_is_novel(void)
{
    /* No decodable frame yet: there is no view to have a first impression of,
     * so the gate fails CLOSED (issue #690), like ambient_audio_novel() with no
     * microphone. Answering true here would open `speak` on the strength of a
     * camera that has never produced a picture. */
    scene_change_init();
    ASSERT(!scene_change_novel());

    /* Once a frame decodes, nothing has been spoken about yet, so there is no
     * reference — and the robot's very first observation genuinely is new.
     * Getting this wrong would mute the boot self-introduction. */
    fill(100);
    observe();
    ASSERT(scene_change_novel());
}

static void test_static_scene_goes_quiet_after_speaking(void)
{
    scene_change_init();
    fill(100);
    rect(10, 10, 25, 25, 180);
    observe();
    scene_change_mark_spoken();

    /* The bench case: nothing moves, so nothing more is worth saying — for as
     * long as that stays true. */
    for (int i = 0; i < 20; ++i) {
        observe();
        ASSERT(!scene_change_novel());
        ASSERT(scene_change_score() == 0);
    }
}

static void test_change_after_speaking_reopens_the_gate(void)
{
    scene_change_init();
    fill(100);
    rect(10, 10, 25, 25, 180);
    observe();
    scene_change_mark_spoken();
    ASSERT(!scene_change_novel());

    fill(100);
    rect(0, 0, 12, 12, 20); /* something new in the corner */
    observe();
    ASSERT(scene_change_novel());

    /* And once remarked on, the new view becomes the thing being compared
     * against — the robot must not keep re-announcing the same arrival. */
    scene_change_mark_spoken();
    ASSERT(!scene_change_novel());
}

static void test_reference_is_the_spoken_frame_not_the_previous_one(void)
{
    /* A scene that drifts a little each frame must eventually trip the gate.
     * Comparing consecutive frames would never accumulate: each step is below
     * threshold, so the robot would fall permanently silent as the room changed
     * completely around it. */
    scene_change_init();
    fill(100);
    observe();
    scene_change_mark_spoken();

    int fired = 0;
    for (int step = 1; step <= 12; ++step) {
        fill(100);
        rect(0, 0, step * 3, 30, 200); /* a shadow creeping across the frame */
        observe();
        if (scene_change_novel()) {
            fired = 1;
            break;
        }
    }
    ASSERT(fired);
}

static void test_undecodable_frames_do_not_move_the_gate(void)
{
    scene_change_init();
    fill(100);
    rect(10, 10, 25, 25, 180);
    observe();
    scene_change_mark_spoken();

    scene_fingerprint_t bad;
    memset(&bad, 0, sizeof(bad)); /* valid == false */
    for (int i = 0; i < 5; ++i) {
        scene_change_note(&bad);
    }
    scene_change_note(NULL);

    /* A run of corrupt captures says nothing about whether the scene moved. If
     * they overwrote the current frame, its all-zero blocks would read as a
     * completely different view and trigger speech about nothing. */
    ASSERT(!scene_change_novel());
    ASSERT(scene_change_score() == 0);
}

static void test_threshold_zero_disables_the_gate(void)
{
    scene_change_init();
    fill(100);
    observe();
    scene_change_mark_spoken();
    ASSERT(!scene_change_novel());

    scene_change_set_threshold(0);
    ASSERT(scene_change_threshold() == 0);
    ASSERT(scene_change_novel());

    /* Disabled outranks the no-frame fail-closed check (issue #690): with the
     * gate off it is not participating, frame or no frame. */
    scene_change_init();
    scene_change_set_threshold(0);
    ASSERT(scene_change_novel());
}

static void test_compared_only_against_a_spoken_about_frame(void)
{
    /* novel() answers true in two states where it compared nothing: before any
     * frame has been spoken about, and with the gate switched off. The planner
     * prompt must not phrase either as "the view has changed since you last
     * spoke" (issue #631), so the gate says whether it actually compared. */
    scene_change_init();
    ASSERT(!scene_change_novel());    /* no frame: fails closed (issue #690) */
    ASSERT(!scene_change_compared()); /* no frame, no reference */

    fill(100);
    observe();
    ASSERT(scene_change_novel());
    ASSERT(!scene_change_compared()); /* a frame, but nothing spoken about yet */

    scene_change_mark_spoken();
    ASSERT(!scene_change_novel());
    ASSERT(scene_change_compared()); /* "same" is now a measured answer */

    rect(0, 0, 12, 12, 20);
    observe();
    ASSERT(scene_change_novel());
    ASSERT(scene_change_compared()); /* and so is "changed" */

    scene_change_set_threshold(0);
    ASSERT(scene_change_novel());
    ASSERT(!scene_change_compared()); /* gate off: true without a comparison */
    scene_change_set_threshold(SCENE_CHANGE_THRESHOLD_DEFAULT);
}

static void test_speaking_before_any_decodable_frame_is_not_a_reference(void)
{
    /* The robot can speak (the sound gate opened) before the camera has produced
     * a frame anyone could decode. mark_spoken() then copies an invalid frame,
     * and a later "same"/"changed" would be measured against nothing. */
    scene_change_init();
    scene_change_mark_spoken();
    ASSERT(!scene_change_compared());
    ASSERT(!scene_change_novel()); /* still no frame to remark on */

    fill(100);
    observe();
    ASSERT(!scene_change_compared());
    ASSERT(scene_change_novel()); /* the first decodable view is new */
}

static void test_threshold_decides_the_verdict(void)
{
    scene_change_init();
    fill(100);
    rect(10, 10, 25, 25, 180);
    observe();
    scene_change_mark_spoken();

    fill(100);
    rect(10, 10, 25, 25, 180);
    rect(0, 0, 8, 8, 150); /* a small change */
    observe();

    const unsigned score = scene_change_score();
    ASSERT(score > 0);

    scene_change_set_threshold((uint8_t)score);
    ASSERT(scene_change_novel()); /* at the threshold counts as changed */
    scene_change_set_threshold((uint8_t)(score + 1));
    ASSERT(!scene_change_novel());

    scene_change_set_threshold(SCENE_CHANGE_THRESHOLD_DEFAULT);
}

/* =========================================================================
 * Pose-keyed references (ADR-029 §2)
 *
 * A head that looks around shows the camera a different view at every pose.
 * Against one reference every pose change reads as "the scene changed", and
 * `speak` would open on every cycle. Each dwell pose therefore has its own
 * reference, and a frame taken off a dwell pose cannot be measured.
 * ========================================================================= */

/** Paint the static view seen at @p pose: a bright band in a different place
 *  per pose, so any cross-pose comparison scores far over the threshold. */
static void paint_pose_view(scene_pose_t pose)
{
    fill(90);
    const int x0 = (int)pose * 10;
    rect(x0, 0, x0 + 10, H, 200);
}

static void view_at_pose(scene_pose_t pose, scene_fingerprint_t *out)
{
    paint_pose_view(pose);
    fingerprint(out);
    out->pose = (uint8_t)pose;
}

static void test_pose_classifier_names_each_dwell_pose(void)
{
    ASSERT(scene_pose_from_head(0, 0) == SCENE_POSE_CENTRE);
    ASSERT(scene_pose_from_head(-SCENE_POSE_SIDE_PAN_DEG, 0) == SCENE_POSE_LEFT);
    ASSERT(scene_pose_from_head(SCENE_POSE_SIDE_PAN_DEG, 0) == SCENE_POSE_RIGHT);
    ASSERT(scene_pose_from_head(0, SCENE_POSE_FLOOR_TILT_DEG) == SCENE_POSE_FLOOR);

    /* Within the tolerance still counts: a clamp or the slew's last step can
     * stop a degree or two short. */
    ASSERT(scene_pose_from_head(SCENE_POSE_TOLERANCE_DEG, -SCENE_POSE_TOLERANCE_DEG) ==
           SCENE_POSE_CENTRE);
    ASSERT(scene_pose_from_head(SCENE_POSE_SIDE_PAN_DEG + SCENE_POSE_TOLERANCE_DEG, 0) ==
           SCENE_POSE_RIGHT);
}

static void test_pose_classifier_rejects_everything_between_poses(void)
{
    ASSERT(scene_pose_from_head(SCENE_POSE_TOLERANCE_DEG + 1, 0) == SCENE_POSE_NONE);
    ASSERT(scene_pose_from_head(20, 0) == SCENE_POSE_NONE); /* mid-slew */
    ASSERT(scene_pose_from_head(0, SCENE_POSE_FLOOR_TILT_DEG / 2) == SCENE_POSE_NONE);
    ASSERT(scene_pose_from_head(-SCENE_POSE_SIDE_PAN_DEG, SCENE_POSE_FLOOR_TILT_DEG) ==
           SCENE_POSE_NONE);                                /* no floor pose to the side */
    ASSERT(scene_pose_from_head(0, 15) == SCENE_POSE_NONE); /* looking up */
}

/** The head-forward default: a robot with no servos, or one whose head stays
 *  centred, keeps exactly the single-reference behaviour it had before. */
static void test_a_fresh_fingerprint_is_centred(void)
{
    fill(100);
    scene_fingerprint_t fp;
    fingerprint(&fp);
    ASSERT(fp.pose == SCENE_POSE_CENTRE);
}

/** ADR-029 step 1, first acceptance test: a head moving over a static scene
 *  scores 0 at every pose once each pose has been spoken about. */
static void test_a_moving_head_over_a_static_scene_scores_zero(void)
{
    scene_change_init();

    scene_fingerprint_t fp;
    for (int p = 0; p < SCENE_POSES; ++p) {
        view_at_pose((scene_pose_t)p, &fp);
        scene_change_note(&fp);
        scene_change_mark_spoken();
    }

    /* Three full scans, in an order unlike the one the references were taken
     * in, with a mid-slew frame after every pose. */
    static const scene_pose_t k_order[] = {SCENE_POSE_CENTRE, SCENE_POSE_RIGHT,  SCENE_POSE_CENTRE,
                                           SCENE_POSE_FLOOR,  SCENE_POSE_CENTRE, SCENE_POSE_LEFT};
    for (int scan = 0; scan < 3; ++scan) {
        for (size_t i = 0; i < sizeof(k_order) / sizeof(k_order[0]); ++i) {
            view_at_pose(k_order[i], &fp);
            scene_change_note(&fp);
            ASSERT(scene_change_score() == 0u);
            ASSERT(scene_change_compared());
            ASSERT(!scene_change_novel());

            scene_fingerprint_t slew;
            view_at_pose(SCENE_POSE_RIGHT, &slew); /* any image; the pose is what counts */
            slew.pose = SCENE_POSE_NONE;
            scene_change_note(&slew);
            ASSERT(scene_change_score() == 0u);
            ASSERT(!scene_change_novel());
        }
    }
}

/** ADR-029 step 1, second acceptance test: a change at one pose scores at that
 *  pose only. */
static void test_a_change_at_one_pose_scores_at_that_pose_only(void)
{
    scene_change_init();

    scene_fingerprint_t fp;
    for (int p = 0; p < SCENE_POSES; ++p) {
        view_at_pose((scene_pose_t)p, &fp);
        scene_change_note(&fp);
        scene_change_mark_spoken();
    }

    /* Something appears in the corner of the left view. */
    scene_fingerprint_t changed_left;
    paint_pose_view(SCENE_POSE_LEFT);
    rect(30, 15, 40, 30, 230);
    fingerprint(&changed_left);
    changed_left.pose = SCENE_POSE_LEFT;
    scene_change_note(&changed_left);
    const unsigned changed = scene_change_score();
    ASSERT(changed >= SCENE_CHANGE_THRESHOLD_DEFAULT);
    ASSERT(scene_change_novel());

    for (int p = 0; p < SCENE_POSES; ++p) {
        if (p == SCENE_POSE_LEFT) {
            continue;
        }
        scene_fingerprint_t other;
        view_at_pose((scene_pose_t)p, &other);
        scene_change_note(&other);
        ASSERT(scene_change_score() == 0u);
        ASSERT(!scene_change_novel());
    }

    /* Back at the left pose, the change is still there: looking away and back
     * does not launder it, because the reference only moves on speaking. */
    scene_change_note(&changed_left);
    ASSERT(scene_change_score() == changed);
}

/** A frame between poses is "cannot measure": not compared, fails closed, and
 *  speaking over it adopts nothing. */
static void test_an_off_pose_frame_is_neither_compared_nor_adopted(void)
{
    scene_change_init();

    scene_fingerprint_t centre;
    view_at_pose(SCENE_POSE_CENTRE, &centre);
    scene_change_note(&centre);
    scene_change_mark_spoken();

    scene_fingerprint_t slew;
    view_at_pose(SCENE_POSE_FLOOR, &slew);
    slew.pose = SCENE_POSE_NONE;
    scene_change_note(&slew);
    ASSERT(!scene_change_compared());
    ASSERT(!scene_change_novel());
    ASSERT(scene_change_score() == 0u);

    /* Speaking while the frame is off-pose must not move any reference. */
    scene_change_mark_spoken();
    scene_change_note(&centre);
    ASSERT(scene_change_score() == 0u);
    ASSERT(!scene_change_novel());

    /* And the floor pose still has no reference, so it is a first impression. */
    scene_fingerprint_t floor_view;
    view_at_pose(SCENE_POSE_FLOOR, &floor_view);
    scene_change_note(&floor_view);
    ASSERT(scene_change_novel());
    ASSERT(!scene_change_compared());

    /* Threshold 0 disables the gate whatever the pose, as before. */
    scene_change_note(&slew);
    scene_change_set_threshold(0);
    ASSERT(scene_change_novel());
    scene_change_set_threshold(SCENE_CHANGE_THRESHOLD_DEFAULT);
}

/** Speaking at one pose re-references that pose alone. */
static void test_speaking_re_references_only_the_current_pose(void)
{
    scene_change_init();

    scene_fingerprint_t left;
    view_at_pose(SCENE_POSE_LEFT, &left);
    scene_change_note(&left);
    scene_change_mark_spoken();

    scene_fingerprint_t right;
    view_at_pose(SCENE_POSE_RIGHT, &right);
    scene_change_note(&right);
    ASSERT(scene_change_novel()); /* never spoken about at this pose */
    ASSERT(!scene_change_compared());

    scene_change_note(&left);
    ASSERT(scene_change_compared());
    ASSERT(scene_change_score() == 0u);
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== scene_change host tests ===\n\n");

    test_run("a flat frame has no deviation", test_flat_frame_has_no_deviation);
    test_run("degenerate input is marked invalid", test_degenerate_input_is_marked_invalid);
    test_run("a bright corner lands in the right block",
             test_bright_corner_lands_in_the_right_block);
    test_run("odd geometry covers every pixel", test_odd_geometry_covers_every_pixel);

    test_run("identical frames are distance zero", test_identical_frames_are_distance_zero);
    test_run("a uniform brightness shift is not a change",
             test_uniform_brightness_shift_is_not_a_change);
    test_run("an object moving registers", test_an_object_moving_registers);
    test_run("invalid operands read as no change", test_invalid_operands_read_as_no_change);

    test_run("the first view is novel", test_first_view_is_novel);
    test_run("a static scene goes quiet after speaking",
             test_static_scene_goes_quiet_after_speaking);
    test_run("a change after speaking reopens the gate",
             test_change_after_speaking_reopens_the_gate);
    test_run("the reference is the spoken frame, not the previous one",
             test_reference_is_the_spoken_frame_not_the_previous_one);
    test_run("undecodable frames do not move the gate",
             test_undecodable_frames_do_not_move_the_gate);
    test_run("threshold zero disables the gate", test_threshold_zero_disables_the_gate);
    test_run("compared only against a spoken-about frame",
             test_compared_only_against_a_spoken_about_frame);
    test_run("speaking before any decodable frame is not a reference",
             test_speaking_before_any_decodable_frame_is_not_a_reference);
    test_run("the threshold decides the verdict", test_threshold_decides_the_verdict);

    test_run("the pose classifier names each dwell pose",
             test_pose_classifier_names_each_dwell_pose);
    test_run("the pose classifier rejects everything between poses",
             test_pose_classifier_rejects_everything_between_poses);
    test_run("a fresh fingerprint is centred", test_a_fresh_fingerprint_is_centred);
    test_run("a moving head over a static scene scores zero",
             test_a_moving_head_over_a_static_scene_scores_zero);
    test_run("a change at one pose scores at that pose only",
             test_a_change_at_one_pose_scores_at_that_pose_only);
    test_run("an off-pose frame is neither compared nor adopted",
             test_an_off_pose_frame_is_neither_compared_nor_adopted);
    test_run("speaking re-references only the current pose",
             test_speaking_re_references_only_the_current_pose);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
