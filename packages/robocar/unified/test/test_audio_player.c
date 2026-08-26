/**
 * @file test_audio_player.c
 * @brief Host tests for the producer-side byte accounting in audio_player.c.
 *
 * One invariant, and every symptom below came from breaking it:
 *
 *     s_written_total == the number of bytes the ring actually accepted.
 *
 * The two quantities are not the same thing, because audio_player_write() holds
 * an odd trailing byte back in s_carry so no 16-bit sample straddles a ring read
 * boundary. Counting the caller's `bytes` instead of what was sent charged the
 * ring for that carried byte, and audio_player_end_utterance() then discarded
 * it — so ring_pending() gained +1 for every utterance with an odd-length tail
 * and never returned to zero. audio_player_is_active() ORs in
 * `ring_pending() > 0`, so it latched true for the rest of the boot, which
 * permanently muted the PDM microphone (ambient_listener.c) and made `listen`
 * refuse to start (voice_turn.c). Nothing about playback sounded wrong, which is
 * why it survived: the robot simply went deaf, one coin-flip utterance in.
 *
 * A bench cannot stage this. The trigger is `total_pcm % TTS_PCM_BATCH_BYTES`
 * landing odd — invisible, roughly half the time, and cumulative — and the
 * consequence shows up minutes later in a different subsystem.
 *
 * SCOPE: these tests drive the producer only. The player task is not started
 * (see test/include/freertos/task.h), so s_played_total does not advance and
 * audio_player_is_active() is not asserted directly. The chain from the
 * invariant above to that predicate is the two-line subtraction in
 * ring_pending(); what is worth pinning, and what actually broke, is the tally.
 */

#include "audio_player.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "freertos/ringbuf.h"
#include "pin_config.h"

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
 * Fixture
 * ========================================================================= */

/** Mirrors TTS_PCM_BATCH_BYTES in gemini_tts.c — the size the decoder's
 *  accumulator flushes at. Every write is this long except the tail. */
#define BATCH_BYTES 2048

static uint8_t s_pcm[BATCH_BYTES];

/** Bytes the ring has accepted, straight from the fake. */
static size_t ring_received(void)
{
    return ringbuf_fake_bytes_received(ringbuf_fake_last_created());
}

/** Drain everything the module has queued, so a multi-utterance run is not
 *  bounded by ring capacity. Deliberately does NOT touch the module's own
 *  played counter — the invariant under test is write-side only. */
static void drain(void)
{
    for (;;) {
        size_t got = 0;
        void *item = xRingbufferReceiveUpTo(ringbuf_fake_last_created(), &got, 0, BATCH_BYTES);
        if (!item) {
            return;
        }
        vRingbufferReturnItem(ringbuf_fake_last_created(), item);
    }
}

/**
 * Play one utterance of @p total_pcm bytes through the real producer path,
 * batched exactly the way gemini_tts.c's pcm_sink()/batch_flush() batches it.
 *
 * Returns the number of bytes the module counted for this utterance.
 */
static size_t utterance(size_t total_pcm)
{
    const size_t before = audio_player_written_total_for_test();

    audio_player_begin_utterance();
    size_t left = total_pcm;
    while (left > 0) {
        const size_t n = (left < BATCH_BYTES) ? left : BATCH_BYTES;
        ASSERT(audio_player_write(s_pcm, n, 0) == ESP_OK);
        left -= n;
    }
    audio_player_end_utterance();

    return audio_player_written_total_for_test() - before;
}

static void fixture_init(void)
{
    static bool done = false;
    if (!done) {
        memset(s_pcm, 0x5A, sizeof(s_pcm));
        ASSERT(audio_player_init() == ESP_OK);
        ASSERT(audio_player_is_ready());
        done = true;
    }
    drain();
}

/* =========================================================================
 * The invariant
 * ========================================================================= */

static void test_even_utterance_is_counted_exactly(void)
{
    fixture_init();
    const size_t before = ring_received();

    /* 48000 = AUDIO_PREROLL_BYTES, and 48000 % 2048 == 896: an even tail, so
     * no byte is ever carried. This is the case that always worked. */
    const size_t counted = utterance(48000);

    ASSERT(counted == 48000);
    ASSERT(ring_received() - before == counted);
}

static void test_odd_tail_does_not_overcount(void)
{
    fixture_init();
    const size_t before = ring_received();

    /* 47999 % 2048 == 895 — an odd tail. The last byte is held in s_carry and
     * then dropped by end_utterance(), so exactly 47998 bytes reach the ring.
     * The old code charged 47999, and that one byte never came back. */
    const size_t counted = utterance(47999);

    ASSERT(counted == 47998);
    ASSERT(ring_received() - before == counted);
}

static void test_odd_tails_do_not_accumulate_drift(void)
{
    fixture_init();

    /* The regression as it was actually reached: a run of utterances, roughly
     * half with an odd tail. The old code drifted +1 per odd tail with nothing
     * on the normal path to resync, so the error was monotonic across a boot.
     * Sizes are multiples of 3 (the base64 decoder emits 3-byte groups). */
    static const size_t sizes[] = {48000, 47999 + 1, 96003, 30001 + 2, 60000, 12345, 3, 2049};

    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        const size_t before = ring_received();
        const size_t counted = utterance(sizes[i]);
        ASSERT(ring_received() - before == counted);
        drain();
    }
}

static void test_carry_is_spliced_and_counted_once(void)
{
    fixture_init();
    const size_t before = ring_received();
    const size_t counted_before = audio_player_written_total_for_test();

    /* Exercise the splice branch, which the TTS path reaches only when two
     * chunk boundaries land odd in a row. Three 3-byte writes in ONE utterance:
     *
     *   w1: no carry  -> 2 bytes sent, 1 carried
     *   w2: carry     -> {carry, pcm[0]} spliced (2), then the remaining 2 sent
     *                    = 4 bytes this call, nothing left over
     *   w3: no carry  -> 2 bytes sent, 1 carried
     *   end_utterance -> the survivor is dropped
     *
     * 8 of the 9 bytes offered reach the ring. The carried byte must be counted
     * exactly once — on the splice that sends it, never on the write that held
     * it back. */
    audio_player_begin_utterance();
    ASSERT(audio_player_write(s_pcm, 3, 0) == ESP_OK);
    ASSERT(audio_player_write(s_pcm, 3, 0) == ESP_OK);
    ASSERT(audio_player_write(s_pcm, 3, 0) == ESP_OK);
    audio_player_end_utterance();

    const size_t counted = audio_player_written_total_for_test() - counted_before;
    ASSERT(counted == 8);
    ASSERT(ring_received() - before == counted);
}

static void test_zero_length_write_counts_nothing(void)
{
    fixture_init();
    const size_t before = ring_received();
    const size_t counted_before = audio_player_written_total_for_test();

    ASSERT(audio_player_write(s_pcm, 0, 0) == ESP_OK);
    ASSERT(audio_player_write(NULL, 16, 0) == ESP_OK);

    ASSERT(audio_player_written_total_for_test() == counted_before);
    ASSERT(ring_received() == before);
}

static void test_abort_resyncs_to_an_idle_state(void)
{
    fixture_init();

    /* abort() settles the played counter onto the written one, so whatever the
     * write side counted, the module must read as idle afterwards. This is the
     * one path that always resynced — it is asserted so a future change to the
     * accounting cannot quietly break it too. */
    audio_player_begin_utterance();
    ASSERT(audio_player_write(s_pcm, 2047, 0) == ESP_OK);
    audio_player_abort();

    ASSERT(!audio_player_is_active());
    drain();
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== audio_player host tests ===\n\n");

    test_run("an even utterance is counted exactly", test_even_utterance_is_counted_exactly);
    test_run("an odd tail does not overcount", test_odd_tail_does_not_overcount);
    test_run("odd tails do not accumulate drift", test_odd_tails_do_not_accumulate_drift);
    test_run("a carried byte is spliced and counted once", test_carry_is_spliced_and_counted_once);
    test_run("a zero-length write counts nothing", test_zero_length_write_counts_nothing);
    test_run("abort resyncs to an idle state", test_abort_resyncs_to_an_idle_state);

    printf("\n=== %d/%d passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
