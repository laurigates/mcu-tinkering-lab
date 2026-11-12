/**
 * @file test_voice_preroll.c
 * @brief Host tests for the voice-turn pre-roll ring (issue #616).
 *
 * The ring exists so a hands-free turn keeps the words that triggered it. The
 * failures worth pinning are the ones a bench shows only as "the robot ignored
 * me": samples out of order, the oldest kept instead of the newest, the robot's
 * own voice leaking in from the playback quarantine, or audio from before an
 * earlier turn spliced onto the next one because taking did not empty the ring.
 * Every frame below carries distinct sample values so a wrong copy is visible as
 * a wrong number, not merely a wrong count.
 */

#include "voice_preroll.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

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

/** Fill @p f with first, first+1, ... so every sample is identifiable. */
static void ramp(int16_t *f, size_t n, int first)
{
    for (size_t i = 0; i < n; ++i) {
        f[i] = (int16_t)(first + (int)i);
    }
}

static void test_frames_before_the_trigger_come_out_in_order(void)
{
    int16_t store[64];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 64);

    int16_t f[8];
    ramp(f, 8, 100);
    voice_preroll_offer(&rb, f, 8, true, false);
    ramp(f, 8, 200);
    voice_preroll_offer(&rb, f, 8, true, false);
    ramp(f, 8, 300);
    voice_preroll_offer(&rb, f, 8, true, false);
    ASSERT(voice_preroll_count(&rb) == 24);

    int16_t out[64];
    memset(out, 0, sizeof(out));
    ASSERT(voice_preroll_take(&rb, out, 64) == 24);
    for (int i = 0; i < 8; ++i) {
        ASSERT(out[i] == 100 + i);
        ASSERT(out[8 + i] == 200 + i);
        ASSERT(out[16 + i] == 300 + i);
    }
}

static void test_a_full_ring_keeps_the_newest(void)
{
    /* The newest samples are the ones next to the trigger and contiguous with the
     * recording that follows — keeping the oldest would put a hole right where
     * the question starts. */
    int16_t store[10];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 10);

    int16_t f[5];
    for (int k = 0; k < 5; ++k) {
        ramp(f, 5, k * 5);
        voice_preroll_offer(&rb, f, 5, true, false);
    }
    ASSERT(voice_preroll_count(&rb) == 10);

    int16_t out[10];
    ASSERT(voice_preroll_take(&rb, out, 10) == 10);
    for (int i = 0; i < 10; ++i) {
        ASSERT(out[i] == 15 + i); /* 15..24, not 0..9 */
    }
}

static void test_a_frame_longer_than_the_ring_keeps_its_tail(void)
{
    int16_t store[4];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 4);

    int16_t f[11];
    ramp(f, 11, 50);
    voice_preroll_offer(&rb, f, 11, true, false);

    int16_t out[4];
    ASSERT(voice_preroll_take(&rb, out, 4) == 4);
    ASSERT(out[0] == 57 && out[1] == 58 && out[2] == 59 && out[3] == 60);
}

static void test_take_into_a_short_buffer_keeps_the_newest(void)
{
    int16_t store[32];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 32);

    int16_t f[20];
    ramp(f, 20, 0);
    voice_preroll_offer(&rb, f, 20, true, false);

    int16_t out[6];
    ASSERT(voice_preroll_take(&rb, out, 6) == 6);
    for (int i = 0; i < 6; ++i) {
        ASSERT(out[i] == 14 + i);
    }
}

static void test_take_empties_the_ring(void)
{
    /* Once a turn holds the microphone the listener stops reading, so anything
     * left behind would be from before this turn and would open the next one. */
    int16_t store[16];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 16);

    int16_t f[8];
    ramp(f, 8, 1);
    voice_preroll_offer(&rb, f, 8, true, false);

    int16_t out[16];
    ASSERT(voice_preroll_take(&rb, out, 16) == 8);
    ASSERT(voice_preroll_count(&rb) == 0);
    ASSERT(voice_preroll_take(&rb, out, 16) == 0);

    /* Discarding with a NULL destination empties it too — the manual `listen`
     * path does this so its own long hold cannot leave stale audio behind. */
    voice_preroll_offer(&rb, f, 8, true, false);
    ASSERT(voice_preroll_take(&rb, NULL, 0) == 0);
    ASSERT(voice_preroll_count(&rb) == 0);
}

static void test_a_quarantined_frame_never_enters_and_empties_the_ring(void)
{
    int16_t store[64];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 64);

    int16_t f[8];
    ramp(f, 8, 100); /* speech before the robot talked */
    voice_preroll_offer(&rb, f, 8, true, false);

    int16_t robot[8];
    for (int i = 0; i < 8; ++i) {
        robot[i] = 9999; /* the robot's own voice */
    }
    voice_preroll_offer(&rb, robot, 8, false, false);
    ASSERT(voice_preroll_count(&rb) == 0);

    ramp(f, 8, 300); /* the person speaking after the hangover */
    voice_preroll_offer(&rb, f, 8, true, false);

    int16_t out[64];
    const size_t got = voice_preroll_take(&rb, out, 64);
    ASSERT(got == 8);
    for (size_t i = 0; i < got; ++i) {
        ASSERT(out[i] != 9999);
        ASSERT(out[i] == (int16_t)(300 + (int)i)); /* nothing from before either */
    }
}

static void test_a_cue_frame_is_kept_as_silence(void)
{
    /* The beep would set the clip's peak and pin the normaliser's gain near 1x;
     * silence of the same length keeps the timing without the tone. */
    int16_t store[64];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 64);

    int16_t f[8];
    ramp(f, 8, 100);
    voice_preroll_offer(&rb, f, 8, true, false);

    int16_t beep[8];
    for (int i = 0; i < 8; ++i) {
        beep[i] = 30000;
    }
    voice_preroll_offer(&rb, beep, 8, true, true);
    ramp(f, 8, 300);
    voice_preroll_offer(&rb, f, 8, true, false);

    int16_t out[64];
    ASSERT(voice_preroll_take(&rb, out, 64) == 24);
    for (int i = 0; i < 8; ++i) {
        ASSERT(out[i] == 100 + i);
        ASSERT(out[8 + i] == 0);
        ASSERT(out[16 + i] == 300 + i);
    }

    /* A NULL frame is fine while the cue is active: nothing is copied. */
    voice_preroll_offer(&rb, NULL, 4, true, true);
    ASSERT(voice_preroll_count(&rb) == 4);
}

static void test_quarantine_outranks_the_cue(void)
{
    int16_t store[16];
    voice_preroll_t rb;
    voice_preroll_init(&rb, store, 16);

    int16_t f[8];
    ramp(f, 8, 1);
    voice_preroll_offer(&rb, f, 8, true, false);
    voice_preroll_offer(&rb, f, 8, false, true);
    ASSERT(voice_preroll_count(&rb) == 0);
}

static void test_a_disabled_ring_holds_nothing(void)
{
    /* The device's state when the PSRAM allocation failed: degrade to "no
     * pre-roll", never crash. */
    voice_preroll_t rb;
    voice_preroll_init(&rb, NULL, 100);

    int16_t f[8];
    ramp(f, 8, 1);
    voice_preroll_offer(&rb, f, 8, true, false);
    voice_preroll_offer(&rb, NULL, 8, true, true);
    ASSERT(voice_preroll_count(&rb) == 0);

    int16_t out[8];
    ASSERT(voice_preroll_take(&rb, out, 8) == 0);
}

int main(void)
{
    printf("=== voice_preroll host tests ===\n\n");

    test_run("frames before the trigger come out in order",
             test_frames_before_the_trigger_come_out_in_order);
    test_run("a full ring keeps the newest", test_a_full_ring_keeps_the_newest);
    test_run("a frame longer than the ring keeps its tail",
             test_a_frame_longer_than_the_ring_keeps_its_tail);
    test_run("take into a short buffer keeps the newest",
             test_take_into_a_short_buffer_keeps_the_newest);
    test_run("take empties the ring", test_take_empties_the_ring);
    test_run("a quarantined frame never enters and empties the ring",
             test_a_quarantined_frame_never_enters_and_empties_the_ring);
    test_run("a cue frame is kept as silence", test_a_cue_frame_is_kept_as_silence);
    test_run("quarantine outranks the cue", test_quarantine_outranks_the_cue);
    test_run("a disabled ring holds nothing", test_a_disabled_ring_holds_nothing);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
