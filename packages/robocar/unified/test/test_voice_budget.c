/**
 * @file test_voice_budget.c
 * @brief Host tests for the voice-turn ration and spend ceiling.
 *
 * The cases worth pinning are the ones a bench cannot stage: an hour of a
 * talkative room run in microseconds, the ceiling staying shut until somebody
 * resumes it, `listen` being exempt from the ration but not from the ceiling,
 * and the window surviving the uint32 millisecond wrap at day 49.
 */

#include <stdio.h>
#include <string.h>

#include "voice_budget.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                            \
        }                                                            \
    } while (0)

#define TEST(name)               \
    static void name(void);      \
    static void run_##name(void) \
    {                            \
        printf("- %s\n", #name); \
        voice_budget_init();     \
        name();                  \
    }                            \
    static void name(void)

#define SEC 1000u
#define MIN (60u * SEC)

/* -------------------------------------------------------------------------- */

TEST(test_a_fresh_budget_allows_both_kinds)
{
    ASSERT(voice_budget_check(0u, true) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_check(0u, false) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_requests() == 0u);
    ASSERT(voice_budget_vad_requests() == 0u);
    ASSERT(voice_budget_ignored_vad() == 0u);
    ASSERT(voice_budget_ignored_listen() == 0u);
    ASSERT(voice_budget_refused() == 0u);
    ASSERT(voice_budget_ration_used(0u) == 0u);
    ASSERT(voice_budget_ration_wait_ms(0u) == 0u);
}

TEST(test_defaults_are_the_documented_ones)
{
    uint8_t max_per = 0;
    uint32_t window_ms = 0;
    voice_budget_get_ration(&max_per, &window_ms);
    ASSERT(max_per == VOICE_BUDGET_RATION_MAX_DEFAULT);
    ASSERT(window_ms == VOICE_BUDGET_RATION_WINDOW_MS_DEFAULT);
    ASSERT(voice_budget_ceiling() == VOICE_BUDGET_MAX_REQUESTS_DEFAULT);
}

TEST(test_the_ration_refuses_vad_once_the_window_is_full)
{
    voice_budget_configure_ration(3u, 5u * MIN);
    voice_budget_configure_ceiling(0u);

    uint32_t t = 1000u;
    for (int i = 0; i < 3; i++) {
        ASSERT(voice_budget_check(t, true) == VOICE_BUDGET_OK);
        voice_budget_note_request(t, true);
        t += 10u * SEC;
    }
    ASSERT(voice_budget_check(t, true) == VOICE_BUDGET_RATIONED);
    ASSERT(voice_budget_ration_used(t) == 3u);
}

TEST(test_the_ration_lifts_when_the_oldest_request_ages_out)
{
    voice_budget_configure_ration(2u, 5u * MIN);
    voice_budget_configure_ceiling(0u);

    voice_budget_note_request(0u + 1u, true);
    voice_budget_note_request(1u * MIN, true);

    ASSERT(voice_budget_check(2u * MIN, true) == VOICE_BUDGET_RATIONED);
    /* The first request ages out at 5 min + 1 ms; the wait says so exactly. */
    ASSERT(voice_budget_ration_wait_ms(2u * MIN) == 3u * MIN + 1u);
    ASSERT(voice_budget_check(5u * MIN, true) == VOICE_BUDGET_RATIONED);
    ASSERT(voice_budget_check(5u * MIN + 1u, true) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_ration_wait_ms(5u * MIN + 1u) == 0u);
}

TEST(test_listen_is_exempt_from_the_ration)
{
    voice_budget_configure_ration(1u, 5u * MIN);
    voice_budget_configure_ceiling(0u);

    voice_budget_note_request(1000u, true);
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_RATIONED);
    ASSERT(voice_budget_check(2000u, false) == VOICE_BUDGET_OK);
}

TEST(test_listen_does_not_use_up_the_ration)
{
    voice_budget_configure_ration(1u, 5u * MIN);
    voice_budget_configure_ceiling(0u);

    for (uint32_t i = 0; i < 5u; i++) {
        voice_budget_note_request(1000u + i, false);
    }
    ASSERT(voice_budget_ration_used(2000u) == 0u);
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_OK);
}

TEST(test_listen_counts_toward_the_ceiling)
{
    voice_budget_configure_ration(0u, 0u);
    voice_budget_configure_ceiling(3u);

    voice_budget_note_request(1000u, false);
    voice_budget_note_request(2000u, false);
    voice_budget_note_request(3000u, true);

    ASSERT(voice_budget_requests() == 3u);
    ASSERT(voice_budget_vad_requests() == 1u);
    ASSERT(voice_budget_check(4000u, false) == VOICE_BUDGET_TRIP_CEILING);
    ASSERT(voice_budget_check(4000u, true) == VOICE_BUDGET_TRIP_CEILING);
}

TEST(test_the_ceiling_is_terminal_until_resume)
{
    voice_budget_configure_ration(0u, 0u);
    voice_budget_configure_ceiling(2u);

    voice_budget_note_request(1000u, true);
    voice_budget_note_request(2000u, true);

    /* No rolling window: a day later it is still shut. */
    ASSERT(voice_budget_check(2000u + 24u * 60u * MIN, true) == VOICE_BUDGET_TRIP_CEILING);
    ASSERT(voice_budget_check(2000u + 24u * 60u * MIN, false) == VOICE_BUDGET_TRIP_CEILING);

    voice_budget_resume();
    ASSERT(voice_budget_requests() == 0u);
    ASSERT(voice_budget_check(3000u, true) == VOICE_BUDGET_OK);
}

TEST(test_the_ceiling_outranks_the_ration)
{
    /* Both binding: name the one that needs a human, not the one that clears by
     * itself in a few minutes. */
    voice_budget_configure_ration(1u, 5u * MIN);
    voice_budget_configure_ceiling(1u);
    voice_budget_note_request(1000u, true);
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_TRIP_CEILING);
}

TEST(test_resume_clears_the_ration_window_and_counters)
{
    voice_budget_configure_ration(1u, 5u * MIN);
    voice_budget_note_request(1000u, true);
    voice_budget_note_ignored(true);
    voice_budget_note_ignored(false);
    voice_budget_note_refused();

    voice_budget_resume();
    ASSERT(voice_budget_ration_used(2000u) == 0u);
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_ignored_vad() == 0u);
    ASSERT(voice_budget_ignored_listen() == 0u);
    ASSERT(voice_budget_refused() == 0u);
    ASSERT(voice_budget_vad_requests() == 0u);
}

TEST(test_resume_keeps_the_configured_limits)
{
    voice_budget_configure_ration(5u, 60u * SEC);
    voice_budget_configure_ceiling(7u);
    voice_budget_resume();

    uint8_t max_per = 0;
    uint32_t window_ms = 0;
    voice_budget_get_ration(&max_per, &window_ms);
    ASSERT(max_per == 5u);
    ASSERT(window_ms == 60u * SEC);
    ASSERT(voice_budget_ceiling() == 7u);
}

TEST(test_raising_the_ceiling_reopens_it)
{
    voice_budget_configure_ration(0u, 0u);
    voice_budget_configure_ceiling(1u);
    voice_budget_note_request(1000u, false);
    ASSERT(voice_budget_check(2000u, false) == VOICE_BUDGET_TRIP_CEILING);

    voice_budget_configure_ceiling(2u);
    ASSERT(voice_budget_check(2000u, false) == VOICE_BUDGET_OK);

    voice_budget_configure_ceiling(0u); /* disabled */
    voice_budget_note_request(3000u, false);
    voice_budget_note_request(4000u, false);
    ASSERT(voice_budget_check(5000u, false) == VOICE_BUDGET_OK);
}

TEST(test_zero_removes_the_ration)
{
    voice_budget_configure_ceiling(0u);

    voice_budget_configure_ration(0u, 5u * MIN);
    for (uint32_t i = 0; i < 50u; i++) {
        voice_budget_note_request(1000u + i, true);
    }
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_ration_wait_ms(2000u) == 0u);

    voice_budget_configure_ration(3u, 0u);
    ASSERT(voice_budget_check(2000u, true) == VOICE_BUDGET_OK);
    ASSERT(voice_budget_ration_used(2000u) == 0u);
}

TEST(test_a_ration_above_the_history_is_clamped)
{
    voice_budget_configure_ration(255u, 5u * MIN);
    uint8_t max_per = 0;
    voice_budget_get_ration(&max_per, NULL);
    ASSERT(max_per == VOICE_BUDGET_HISTORY);
}

TEST(test_an_hour_of_a_talkative_room_is_bounded)
{
    /* The issue's scenario: the trigger fires at the listener's idle cooldown
     * (10 s) for an hour. Unrationed that is 360 requests; the default ration
     * must hold it to 12 per 5 minutes. */
    voice_budget_configure_ceiling(0u);

    unsigned sent = 0;
    unsigned refused = 0;
    for (uint32_t t = 10u * SEC; t <= 60u * MIN; t += 10u * SEC) {
        if (voice_budget_check(t, true) == VOICE_BUDGET_OK) {
            voice_budget_note_request(t, true);
            sent++;
        } else {
            voice_budget_note_refused();
            refused++;
        }
    }
    ASSERT(sent + refused == 360u);
    ASSERT(sent <= 12u * 12u);
    ASSERT(sent >= 12u * 12u - 12u);
    ASSERT(voice_budget_refused() == refused);
}

TEST(test_the_default_ceiling_stops_an_unattended_night)
{
    /* With the default ration saturated all night, the ceiling must trip and
     * stay tripped: 200 requests, then nothing until resume. */
    unsigned sent = 0;
    for (uint32_t t = 10u * SEC; t <= 12u * 60u * MIN; t += 10u * SEC) {
        if (voice_budget_check(t, true) == VOICE_BUDGET_OK) {
            voice_budget_note_request(t, true);
            sent++;
        }
    }
    ASSERT(sent == VOICE_BUDGET_MAX_REQUESTS_DEFAULT);
    ASSERT(voice_budget_check(12u * 60u * MIN + SEC, true) == VOICE_BUDGET_TRIP_CEILING);
}

TEST(test_ignored_replies_are_counted_by_kind)
{
    voice_budget_note_request(1000u, true);
    voice_budget_note_ignored(true);
    voice_budget_note_request(2000u, true);
    voice_budget_note_ignored(true);
    voice_budget_note_request(3000u, false);
    voice_budget_note_ignored(false);

    ASSERT(voice_budget_ignored_vad() == 2u);
    ASSERT(voice_budget_ignored_listen() == 1u);
    ASSERT(voice_budget_vad_requests() == 2u);
    ASSERT(voice_budget_requests() == 3u);
}

TEST(test_the_window_survives_the_uint32_millisecond_wrap)
{
    voice_budget_configure_ration(2u, 5u * MIN);
    voice_budget_configure_ceiling(0u);

    const uint32_t near_wrap = UINT32_MAX - 30u * SEC;
    voice_budget_note_request(near_wrap, true);
    voice_budget_note_request(near_wrap + 10u * SEC, true);

    /* 60 s later the clock has wrapped to a small number; both are in-window. */
    const uint32_t after = near_wrap + 60u * SEC;
    ASSERT(after < near_wrap);
    ASSERT(voice_budget_ration_used(after) == 2u);
    ASSERT(voice_budget_check(after, true) == VOICE_BUDGET_RATIONED);

    /* And they age out on schedule rather than never (or immediately). */
    ASSERT(voice_budget_check(near_wrap + 5u * MIN + 1u, true) == VOICE_BUDGET_OK);
}

TEST(test_a_disabled_ceiling_never_trips)
{
    voice_budget_configure_ration(0u, 0u);
    voice_budget_configure_ceiling(0u);
    for (uint32_t i = 0; i < 1000u; i++) {
        voice_budget_note_request(i, false);
    }
    ASSERT(voice_budget_requests() == 1000u);
    ASSERT(voice_budget_check(2000u, false) == VOICE_BUDGET_OK);
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    printf("voice_budget tests\n");
    run_test_a_fresh_budget_allows_both_kinds();
    run_test_defaults_are_the_documented_ones();
    run_test_the_ration_refuses_vad_once_the_window_is_full();
    run_test_the_ration_lifts_when_the_oldest_request_ages_out();
    run_test_listen_is_exempt_from_the_ration();
    run_test_listen_does_not_use_up_the_ration();
    run_test_listen_counts_toward_the_ceiling();
    run_test_the_ceiling_is_terminal_until_resume();
    run_test_the_ceiling_outranks_the_ration();
    run_test_resume_clears_the_ration_window_and_counters();
    run_test_resume_keeps_the_configured_limits();
    run_test_raising_the_ceiling_reopens_it();
    run_test_zero_removes_the_ration();
    run_test_a_ration_above_the_history_is_clamped();
    run_test_an_hour_of_a_talkative_room_is_bounded();
    run_test_the_default_ceiling_stops_an_unattended_night();
    run_test_ignored_replies_are_counted_by_kind();
    run_test_the_window_survives_the_uint32_millisecond_wrap();
    run_test_a_disabled_ceiling_never_trips();

    if (g_failures) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
