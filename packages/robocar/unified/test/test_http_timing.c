/**
 * @file test_http_timing.c
 * @brief Host tests for the HTTP phase stamps that split a voice turn's
 *        latency (issue #948).
 *
 * What a bench cannot isolate: an event that repeats (one ON_HEADER per header,
 * many ON_DATA) moving its stamp, a phase that never happened printed as a
 * number, and the uint32 millisecond wrap at day 49 turning a 300 ms phase into
 * 49 days.
 */

#include "http_timing.h"

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

static void full_run(http_timing_t *t, uint32_t base)
{
    http_timing_start(t, base);
    http_timing_note(t, HTTP_TIMING_CONNECTED, base + 900);    /* DNS + TCP + TLS */
    http_timing_note(t, HTTP_TIMING_HEADERS_SENT, base + 905); /* body follows */
    http_timing_note(t, HTTP_TIMING_FIRST_HEADER, base + 9905);
    http_timing_note(t, HTTP_TIMING_FIRST_DATA, base + 9910);
    http_timing_note(t, HTTP_TIMING_FINISHED, base + 10010);
}

static void test_phases_from_a_full_request(void)
{
    http_timing_t t;
    full_run(&t, 1000);
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=900 request=9000 download=105 total=10010 ms") == 0);
}

static void test_repeated_events_keep_the_first_stamp(void)
{
    /* ON_HEADER fires once per header and ON_DATA once per chunk; the phase
     * boundary is the first of each. */
    http_timing_t t;
    full_run(&t, 0);
    http_timing_note(&t, HTTP_TIMING_FIRST_HEADER, 20000);
    http_timing_note(&t, HTTP_TIMING_FIRST_DATA, 20000);
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=900 request=9000 download=105 total=10010 ms") == 0);
}

static void test_a_phase_that_did_not_happen_prints_a_dash(void)
{
    /* A connect that failed: no headers sent, no response. "0" would read as
     * an instant phase. */
    http_timing_t t;
    http_timing_start(&t, 500);
    http_timing_note(&t, HTTP_TIMING_FINISHED, 6500);
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=- request=- download=- total=6000 ms") == 0);
}

static void test_nothing_noted_prints_all_dashes(void)
{
    http_timing_t t;
    http_timing_start(&t, 500);
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=- request=- download=- total=- ms") == 0);
}

static void test_a_stamp_of_zero_counts_as_seen(void)
{
    /* esp_timer starts at boot; a request at millisecond 0 is legal. */
    http_timing_t t;
    http_timing_start(&t, 0);
    http_timing_note(&t, HTTP_TIMING_CONNECTED, 0);
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strncmp(buf, "connect=0 ", 10) == 0);
}

static void test_phases_survive_the_uint32_wrap(void)
{
    http_timing_t t;
    full_run(&t, 0xFFFFFFFFu - 400u); /* connect lands after the wrap */
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=900 request=9000 download=105 total=10010 ms") == 0);
}

static void test_events_before_start_are_ignored(void)
{
    http_timing_t t;
    memset(&t, 0, sizeof(t));
    http_timing_start(&t, 100);
    http_timing_note(&t, (http_timing_event_t)99, 200); /* unknown id: no effect */
    char buf[128];
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(strcmp(buf, "connect=- request=- download=- total=- ms") == 0);
}

static void test_format_never_overruns(void)
{
    http_timing_t t;
    full_run(&t, 0);
    char buf[12];
    memset(buf, 0x7F, sizeof(buf));
    http_timing_format(&t, buf, sizeof(buf));
    ASSERT(buf[sizeof(buf) - 1] == '\0');
    ASSERT(strlen(buf) == sizeof(buf) - 1);
}

int main(void)
{
    printf("=== http_timing host tests ===\n\n");
    test_run("phases_from_a_full_request", test_phases_from_a_full_request);
    test_run("repeated_events_keep_the_first_stamp", test_repeated_events_keep_the_first_stamp);
    test_run("a_phase_that_did_not_happen_prints_a_dash",
             test_a_phase_that_did_not_happen_prints_a_dash);
    test_run("nothing_noted_prints_all_dashes", test_nothing_noted_prints_all_dashes);
    test_run("a_stamp_of_zero_counts_as_seen", test_a_stamp_of_zero_counts_as_seen);
    test_run("phases_survive_the_uint32_wrap", test_phases_survive_the_uint32_wrap);
    test_run("events_before_start_are_ignored", test_events_before_start_are_ignored);
    test_run("format_never_overruns", test_format_never_overruns);
    printf("\n=== Results ===\nPassed: %d / %d\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
