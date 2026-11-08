/**
 * @file test_ota_announce.c
 * @brief Host tests for the pre-OTA announcement text and cancel window
 *        (ota_announce.c).
 *
 * What matters and is hard to see on a bench: a note list that does not fit
 * must drop whole notes rather than end mid-word or mid-UTF-8 sequence, and a
 * postpone just after the window closes must not count. Plus the uint32 wrap.
 */

#include "ota_announce.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

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
    test_pass++;
    printf("     PASS\n");
}

static void test_facts_name_both_versions_and_every_note(void)
{
    const char *notes[] = {"robocar-unified: announce updates", "mechanism: animate the sweep"};
    char out[256];
    const size_t n = ota_announce_facts("0.2.18", "0.2.17", notes, 2, out, sizeof(out));
    ASSERT(n == strlen(out));
    ASSERT(strcmp(out, "Firmware version 0.2.18 is available (running 0.2.17). Changes: "
                       "robocar-unified: announce updates; mechanism: animate the sweep.") == 0);
}

static void test_no_notes_says_not_listed(void)
{
    char out[128];
    ota_announce_facts("1.0.0", "0.9.0", NULL, 0, out, sizeof(out));
    ASSERT(strcmp(out, "Firmware version 1.0.0 is available (running 0.9.0). "
                       "Changes: not listed.") == 0);
}

static void test_notes_that_do_not_fit_are_dropped_whole(void)
{
    const char *notes[] = {"first note", "a second note that is much too long to fit here"};
    char out[96];
    ota_announce_facts("1.0.0", "0.9.0", notes, 2, out, sizeof(out));
    ASSERT(strstr(out, "first note") != NULL);
    ASSERT(strstr(out, "second") == NULL);
    ASSERT(out[strlen(out) - 1] == '.');
}

static void test_a_short_note_after_a_skipped_long_one_still_fits(void)
{
    const char *notes[] = {"short", "a long note that cannot fit in the space left", "tiny"};
    char out[80];
    ota_announce_facts("1.0.0", "0.9.0", notes, 3, out, sizeof(out));
    ASSERT(strstr(out, "Changes: short; tiny.") != NULL);
}

static void test_a_note_with_utf8_is_never_split(void)
{
    /* "äää…" is two bytes per character; any cut inside it would leave a
     * broken sequence in the JSON request. Dropping it whole avoids that. */
    const char *notes[] = {"ok", "äääääääääääääääääääääääääääääää"};
    for (size_t len = 70; len < 140; ++len) {
        char out[160];
        memset(out, 0x7F, sizeof(out));
        ota_announce_facts("1.0.0", "0.9.0", notes, 2, out, len);
        ASSERT(strlen(out) < len);
        const char *a = strstr(out, "ä");
        if (a) {
            ASSERT(strstr(out, "äääääääääääääääääääääääääääääää") != NULL);
        }
    }
}

static void test_null_notes_are_skipped(void)
{
    const char *notes[] = {NULL, "real"};
    char out[128];
    ota_announce_facts("1.0.0", "0.9.0", notes, 2, out, sizeof(out));
    ASSERT(strstr(out, "Changes: real.") != NULL);
}

static void test_too_small_for_the_version_clause_writes_nothing(void)
{
    char out[16];
    memset(out, 'x', sizeof(out));
    ASSERT(ota_announce_facts("1.0.0", "0.9.0", NULL, 0, out, sizeof(out)) == 0);
    ASSERT(out[0] == '\0');
    ASSERT(ota_announce_facts(NULL, "0.9.0", NULL, 0, out, sizeof(out)) == 0);
}

static void test_fallback_lines(void)
{
    char out[128];
    ota_announce_fallback("0.2.18", true, out, sizeof(out));
    ASSERT(strstr(out, "0.2.18") != NULL);
    ASSERT(strstr(out, "päivitys") != NULL);
    ota_announce_fallback("0.2.18", false, out, sizeof(out));
    ASSERT(strstr(out, "0.2.18") != NULL);
    ASSERT(strstr(out, "update") != NULL);
}

static void test_the_pun_is_exact(void)
{
    ASSERT(strcmp(OTA_ANNOUNCE_PUN, "Menen tuonne toisaalle vähän päivettymään.") == 0);
}

static void test_window_open_postpone_expire(void)
{
    ota_window_t w;
    ota_window_open(&w, 1000, 60000);
    ASSERT(ota_window_state(&w, 1000) == OTA_WINDOW_OPEN);
    ASSERT(ota_window_state(&w, 60999) == OTA_WINDOW_OPEN);
    ASSERT(ota_window_state(&w, 61000) == OTA_WINDOW_EXPIRED);
    ASSERT(ota_window_postpone(&w, 61000) == false); /* too late */
    ASSERT(ota_window_state(&w, 61000) == OTA_WINDOW_EXPIRED);
}

static void test_postpone_inside_the_window_sticks(void)
{
    ota_window_t w;
    ota_window_open(&w, 0, 60000);
    ASSERT(ota_window_postpone(&w, 30000) == true);
    ASSERT(ota_window_state(&w, 30000) == OTA_WINDOW_POSTPONED);
    ASSERT(ota_window_state(&w, 120000) == OTA_WINDOW_POSTPONED);
}

static void test_postpone_without_a_window_is_refused(void)
{
    ota_window_t w = {0};
    ASSERT(ota_window_postpone(&w, 0) == false);
}

static void test_window_survives_the_wrap(void)
{
    ota_window_t w;
    ota_window_open(&w, UINT32_MAX - 1000u, 60000);
    ASSERT(ota_window_state(&w, UINT32_MAX - 500u) == OTA_WINDOW_OPEN); /* 0.5 s in, before it */
    ASSERT(ota_window_state(&w, 5000u) == OTA_WINDOW_OPEN);             /* 6 s in, after the wrap */
    ASSERT(ota_window_postpone(&w, 5000u) == true);
    ota_window_open(&w, UINT32_MAX - 1000u, 60000);
    ASSERT(ota_window_state(&w, 59000u) == OTA_WINDOW_EXPIRED); /* 60.001 s in */
}

int main(void)
{
    printf("=== ota_announce host tests ===\n\n");
    test_run("facts_name_both_versions_and_every_note",
             test_facts_name_both_versions_and_every_note);
    test_run("no_notes_says_not_listed", test_no_notes_says_not_listed);
    test_run("notes_that_do_not_fit_are_dropped_whole",
             test_notes_that_do_not_fit_are_dropped_whole);
    test_run("a_short_note_after_a_skipped_long_one_still_fits",
             test_a_short_note_after_a_skipped_long_one_still_fits);
    test_run("a_note_with_utf8_is_never_split", test_a_note_with_utf8_is_never_split);
    test_run("null_notes_are_skipped", test_null_notes_are_skipped);
    test_run("too_small_for_the_version_clause_writes_nothing",
             test_too_small_for_the_version_clause_writes_nothing);
    test_run("fallback_lines", test_fallback_lines);
    test_run("the_pun_is_exact", test_the_pun_is_exact);
    test_run("window_open_postpone_expire", test_window_open_postpone_expire);
    test_run("postpone_inside_the_window_sticks", test_postpone_inside_the_window_sticks);
    test_run("postpone_without_a_window_is_refused", test_postpone_without_a_window_is_refused);
    test_run("window_survives_the_wrap", test_window_survives_the_wrap);
    printf("\n=== Results ===\nPassed: %d / %d\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
