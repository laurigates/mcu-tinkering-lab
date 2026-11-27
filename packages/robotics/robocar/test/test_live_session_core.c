/**
 * @file test_live_session_core.c
 * @brief Host tests for the Live session's decisions (ADR-032 phase 1).
 *
 * The half-duplex mute, the close rules, turn bookkeeping, frame reassembly and
 * the session gain, on an injected clock. The cases worth pinning are the ones
 * a bench cannot stage on demand: the reply's first audio arriving before the
 * amplifier starts, a lost turnComplete, the uint32 wrap at day 49.
 */

#include "live_session_core.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int test_count = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        fflush(stdout);
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
    printf("     PASS\n");
}

static const live_session_cfg_t k_cfg = {
    .idle_ms = 7000,
    .max_ms = 300000,
    .hangover_ms = 400,
};

static live_server_msg_t audio_msg(void)
{
    live_server_msg_t m = {0};
    m.audio_parts = 1;
    m.audio_bytes = 4800;
    return m;
}

static live_server_msg_t turn_complete_msg(const char *text)
{
    live_server_msg_t m = {0};
    m.turn_complete = true;
    if (text) {
        snprintf(m.output_text, sizeof(m.output_text), "%s", text);
    }
    return m;
}

/* ------------------------------------------------------------ half-duplex */

static void test_upstream_open_at_start(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 1000);
    ASSERT(live_session_upstream_allowed(&s, false, 1000));
    ASSERT(s.chunks_sent == 1 && s.chunks_muted == 0);
}

static void test_muted_from_first_reply_audio_before_the_amp_plays(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    const live_server_msg_t a = audio_msg();
    const unsigned ev = live_session_note_server(&s, &a, 100);
    ASSERT(ev & LIVE_EV_TURN_START);
    /* The player is still banking its preroll: the amp is silent, but the
     * reply has begun, so nothing goes up. */
    ASSERT(!live_session_upstream_allowed(&s, false, 150));
    ASSERT(s.chunks_muted == 1);
}

static void test_muted_through_playback_and_hangover(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t a = audio_msg();
    live_session_note_server(&s, &a, 100);
    live_server_msg_t done = turn_complete_msg("Hei.");
    const unsigned ev = live_session_note_server(&s, &done, 200);
    ASSERT(ev & LIVE_EV_TURN_END);
    ASSERT(!live_session_upstream_allowed(&s, true, 300));   /* playing */
    ASSERT(!live_session_upstream_allowed(&s, true, 3000));  /* still playing */
    ASSERT(!live_session_upstream_allowed(&s, false, 3100)); /* edge: hangover starts */
    ASSERT(!live_session_upstream_allowed(&s, false, 3499));
    ASSERT(live_session_upstream_allowed(&s, false, 3500)); /* hangover over */
}

static void test_turn_text_accumulates_fragments(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t a = audio_msg();
    snprintf(a.output_text, sizeof(a.output_text), "Minä olen");
    live_session_note_server(&s, &a, 10);
    live_server_msg_t b = audio_msg();
    snprintf(b.output_text, sizeof(b.output_text), " Teuvo.");
    ASSERT(!(live_session_note_server(&s, &b, 20) & LIVE_EV_TURN_START));
    live_server_msg_t done = turn_complete_msg(NULL);
    ASSERT(live_session_note_server(&s, &done, 30) & LIVE_EV_TURN_END);
    ASSERT(strcmp(s.turn_text, "Minä olen Teuvo.") == 0);
    ASSERT(s.turns == 1);
    ASSERT(s.audio_bytes == 9600);

    /* The next turn starts with an empty transcript. */
    live_server_msg_t c = audio_msg();
    ASSERT(live_session_note_server(&s, &c, 40) & LIVE_EV_TURN_START);
    ASSERT(s.turn_text[0] == '\0');
}

static void test_turn_text_is_bounded(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    for (int i = 0; i < 40; i++) {
        live_server_msg_t a = audio_msg();
        snprintf(a.output_text, sizeof(a.output_text), "kymmenen m"); /* 10 bytes */
        live_session_note_server(&s, &a, (uint32_t)i);
    }
    ASSERT(strlen(s.turn_text) == LIVE_SESSION_TURN_TEXT_MAX - 1);
}

static void test_turn_complete_without_audio_is_not_a_spoken_turn(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t done = turn_complete_msg(NULL);
    ASSERT(!(live_session_note_server(&s, &done, 10) & LIVE_EV_TURN_END));
    ASSERT(s.turns == 0);
    ASSERT(live_session_upstream_allowed(&s, false, 20));
}

static void test_wait_queues_a_silent_tool_response(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t m = {0};
    m.tool_call = true;
    snprintf(m.tool_id, sizeof(m.tool_id), "fc-1");
    snprintf(m.tool_name, sizeof(m.tool_name), "wait");
    const unsigned ev = live_session_note_server(&s, &m, 10);
    ASSERT(ev & LIVE_EV_TOOL);
    ASSERT(ev & LIVE_EV_WAIT);
    ASSERT(s.tool_pending && strcmp(s.tool_id, "fc-1") == 0);
    ASSERT(s.waits == 1);
    live_session_tool_sent(&s);
    ASSERT(!s.tool_pending);
}

static void test_unknown_tool_is_answered_but_not_a_wait(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t m = {0};
    m.tool_call = true;
    snprintf(m.tool_id, sizeof(m.tool_id), "fc-2");
    snprintf(m.tool_name, sizeof(m.tool_name), "drive");
    const unsigned ev = live_session_note_server(&s, &m, 10);
    ASSERT((ev & LIVE_EV_TOOL) && !(ev & LIVE_EV_WAIT));
    ASSERT(s.waits == 0);
}

static void test_interrupted_ends_the_turn(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t a = audio_msg();
    live_session_note_server(&s, &a, 10);
    live_server_msg_t i = {0};
    i.interrupted = true;
    const unsigned ev = live_session_note_server(&s, &i, 20);
    ASSERT(ev & LIVE_EV_INTERRUPTED);
    ASSERT(!(ev & LIVE_EV_TURN_END));
    ASSERT(s.interrupts == 1 && !s.model_speaking);
    /* Interrupted with nothing being spoken is not news. */
    ASSERT(!(live_session_note_server(&s, &i, 30) & LIVE_EV_INTERRUPTED));
}

/* ------------------------------------------------------------------ close */

static void test_idle_closes_after_silence(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 1000);
    ASSERT(live_session_should_end(&s, false, 7999) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, 8000) == LIVE_END_IDLE);
}

static void test_local_speech_keeps_it_open(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_session_note_speech(&s, 6000);
    ASSERT(live_session_should_end(&s, false, 12999) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, 13000) == LIVE_END_IDLE);
}

static void test_input_transcription_keeps_it_open(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t m = {0};
    snprintf(m.input_text, sizeof(m.input_text), "Hei");
    live_session_note_server(&s, &m, 5000);
    ASSERT(live_session_should_end(&s, false, 11999) == LIVE_END_NONE);
}

static void test_idle_counts_from_the_end_of_playback(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t a = audio_msg();
    live_session_note_server(&s, &a, 1000);
    live_server_msg_t done = turn_complete_msg(NULL);
    live_session_note_server(&s, &done, 1100);
    /* A 10 s reply: playing well past idle_ms after the last message. */
    ASSERT(live_session_should_end(&s, true, 9000) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, true, 11000) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, 11100) == LIVE_END_NONE); /* edge */
    ASSERT(live_session_should_end(&s, false, 18099) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, 18100) == LIVE_END_IDLE);
}

static void test_lost_turn_complete_still_closes_on_idle(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t a = audio_msg();
    live_session_note_server(&s, &a, 1000);
    /* turnComplete never arrives: the mute stays, but idle still ends it. */
    ASSERT(live_session_should_end(&s, false, 8000) == LIVE_END_IDLE);
}

static void test_cap_goaway_error_stop(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    for (uint32_t t = 0; t < 300000; t += 5000) {
        live_session_note_speech(&s, t);
    }
    ASSERT(live_session_should_end(&s, false, 299999) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, 300000) == LIVE_END_CAP);

    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t g = {0};
    g.go_away = true;
    g.go_away_ms = 10000;
    live_session_note_server(&s, &g, 10);
    ASSERT(live_session_should_end(&s, false, 20) == LIVE_END_GOAWAY);

    live_session_begin(&s, &k_cfg, 0);
    live_session_fail(&s);
    ASSERT(live_session_should_end(&s, false, 1) == LIVE_END_ERROR);

    live_session_begin(&s, &k_cfg, 0);
    live_session_stop(&s);
    ASSERT(live_session_should_end(&s, false, 1) == LIVE_END_STOPPED);
}

static void test_clock_wrap(void)
{
    live_session_t s;
    const uint32_t t0 = UINT32_MAX - 2000;
    live_session_begin(&s, &k_cfg, t0);
    ASSERT(live_session_should_end(&s, false, t0 + 6999) == LIVE_END_NONE);
    ASSERT(live_session_should_end(&s, false, t0 + 7000) == LIVE_END_IDLE);
    live_session_begin(&s, &k_cfg, t0);
    live_server_msg_t a = audio_msg();
    live_session_note_server(&s, &a, t0);
    live_server_msg_t done = turn_complete_msg(NULL);
    live_session_note_server(&s, &done, t0 + 1);
    live_session_upstream_allowed(&s, true, t0 + 100);
    ASSERT(!live_session_upstream_allowed(&s, false, t0 + 2100)); /* across the wrap */
    ASSERT(live_session_upstream_allowed(&s, false, t0 + 2500));
}

static void test_usage_is_kept(void)
{
    live_session_t s;
    live_session_begin(&s, &k_cfg, 0);
    live_server_msg_t m = {0};
    m.has_usage = true;
    m.total_tokens = 1123;
    live_session_note_server(&s, &m, 1);
    ASSERT(s.total_tokens == 1123);
}

static void test_end_names(void)
{
    ASSERT(strcmp(live_end_name(LIVE_END_IDLE), "idle") == 0);
    ASSERT(strcmp(live_end_name(LIVE_END_GOAWAY), "goaway") == 0);
    ASSERT(strcmp(live_end_name((live_end_t)99), "?") == 0);
}

/* ------------------------------------------------------------- reassembly */

static void test_rx_single_piece(void)
{
    live_rx_t rx;
    live_rx_init(&rx, 64);
    ASSERT(live_rx_feed(&rx, "{\"a\":1}", 7, 0, 7, true) == 1);
    ASSERT(rx.len == 7 && strcmp(rx.buf, "{\"a\":1}") == 0);
    /* The next message starts fresh. */
    ASSERT(live_rx_feed(&rx, "{}", 2, 0, 2, true) == 1);
    ASSERT(rx.len == 2 && strcmp(rx.buf, "{}") == 0);
    live_rx_free(&rx);
}

static void test_rx_pieces_and_frames(void)
{
    live_rx_t rx;
    live_rx_init(&rx, 64);
    /* Frame 1 of 2 (not fin), delivered in two pieces. */
    ASSERT(live_rx_feed(&rx, "{\"", 2, 0, 5, false) == 0);
    ASSERT(live_rx_feed(&rx, "a\":", 3, 2, 5, false) == 0);
    /* Frame 2, fin, in two pieces. */
    ASSERT(live_rx_feed(&rx, "1", 1, 0, 2, true) == 0);
    ASSERT(live_rx_feed(&rx, "}", 1, 1, 2, true) == 1);
    ASSERT(rx.len == 7 && strcmp(rx.buf, "{\"a\":1}") == 0);
    live_rx_free(&rx);
}

static void test_rx_drops_an_oversized_message_whole(void)
{
    live_rx_t rx;
    live_rx_init(&rx, 8);
    ASSERT(live_rx_feed(&rx, "0123456", 7, 0, 12, true) == 0);
    ASSERT(live_rx_feed(&rx, "789ab", 5, 7, 12, true) == -1);
    /* And recovers for the next one. */
    ASSERT(live_rx_feed(&rx, "{}", 2, 0, 2, true) == 1);
    ASSERT(strcmp(rx.buf, "{}") == 0);
    live_rx_free(&rx);
}

/* ------------------------------------------------------------------ gain */

static void test_gain_removes_dc_scales_and_saturates(void)
{
    int16_t pcm[4] = {110, 90, 1100, -32000};
    const uint32_t clipped = live_gain_apply(pcm, 4, 100, 512); /* 2.0x */
    ASSERT(pcm[0] == 20 && pcm[1] == -20);
    ASSERT(pcm[2] == 2000);
    ASSERT(pcm[3] == -32768);
    ASSERT(clipped == 1);
}

static void test_unity_gain_is_identity_without_dc(void)
{
    int16_t pcm[3] = {-5, 0, 32767};
    ASSERT(live_gain_apply(pcm, 3, 0, 256) == 0);
    ASSERT(pcm[0] == -5 && pcm[1] == 0 && pcm[2] == 32767);
}

int main(void)
{
    test_run("upstream open at start", test_upstream_open_at_start);
    test_run("muted from first reply audio",
             test_muted_from_first_reply_audio_before_the_amp_plays);
    test_run("muted through playback + hangover", test_muted_through_playback_and_hangover);
    test_run("turn text accumulates", test_turn_text_accumulates_fragments);
    test_run("turn text is bounded", test_turn_text_is_bounded);
    test_run("turnComplete without audio", test_turn_complete_without_audio_is_not_a_spoken_turn);
    test_run("wait queues a SILENT response", test_wait_queues_a_silent_tool_response);
    test_run("unknown tool answered, not a wait", test_unknown_tool_is_answered_but_not_a_wait);
    test_run("interrupted ends the turn", test_interrupted_ends_the_turn);
    test_run("idle closes", test_idle_closes_after_silence);
    test_run("local speech keeps it open", test_local_speech_keeps_it_open);
    test_run("input transcription keeps it open", test_input_transcription_keeps_it_open);
    test_run("idle counts from playback end", test_idle_counts_from_the_end_of_playback);
    test_run("lost turnComplete closes on idle", test_lost_turn_complete_still_closes_on_idle);
    test_run("cap, goAway, error, stop", test_cap_goaway_error_stop);
    test_run("clock wrap", test_clock_wrap);
    test_run("usage kept", test_usage_is_kept);
    test_run("end names", test_end_names);
    test_run("rx single piece", test_rx_single_piece);
    test_run("rx pieces and frames", test_rx_pieces_and_frames);
    test_run("rx drops oversized whole", test_rx_drops_an_oversized_message_whole);
    test_run("gain: dc, scale, saturate", test_gain_removes_dc_scales_and_saturates);
    test_run("unity gain identity", test_unity_gain_is_identity_without_dc);
    printf("\nAll %d live_session_core tests passed\n", test_count);
    return 0;
}
