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
    /* Nothing has been spoken about yet, so there is no reference — and the
     * robot's very first observation genuinely is new. Getting this wrong would
     * mute the boot self-introduction. */
    scene_change_init();
    ASSERT(scene_change_novel());

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
    test_run("the threshold decides the verdict", test_threshold_decides_the_verdict);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
