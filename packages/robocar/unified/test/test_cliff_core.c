/**
 * @file test_cliff_core.c
 * @brief Pins the cliff reflex decisions in cliff_core.c.
 *
 * cliff_core.c is compiled unmodified. What is pinned:
 *
 *  - Fail closed: the state starts as "no floor", and a failed read counts as
 *    no floor, so nothing is permitted until the floor is confirmed.
 *  - The boot is not an edge: confirming the floor, then losing it, trips;
 *    never having had floor does not.
 *  - Debounce: one no-floor sample does not trip, CLIFF_TRIP_SAMPLES do;
 *    clearing takes CLIFF_CLEAR_SAMPLES.
 *  - Front edge: brake for CLIFF_BRAKE_MS, reverse for CLIFF_BACKOFF_MS, then
 *    hand back; rear edge mirrors it forward.
 *  - No back-off toward a side without floor, and no forward pull-away into an
 *    obstacle; a back-off stops the moment its side loses floor.
 *  - permits(): forward needs front floor, backward rear floor, rotate both.
 *  - The uint32 millisecond wrap does not stretch or cut a phase.
 */

#include "cliff_core.h"

#include <stdio.h>
#include <stdlib.h>

static int s_failures;

#define CHECK(cond, ...)                                           \
    do {                                                           \
        if (!(cond)) {                                             \
            s_failures++;                                          \
            fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                          \
            fputc('\n', stderr);                                   \
        }                                                          \
    } while (0)

#define TICK_MS 33U

static const cliff_sample_t BOTH_FLOOR = {.valid = true, .floor = {true, true}};
static const cliff_sample_t FRONT_GONE = {.valid = true, .floor = {false, true}};
static const cliff_sample_t REAR_GONE = {.valid = true, .floor = {true, false}};
static const cliff_sample_t BOTH_GONE = {.valid = true, .floor = {false, false}};
static const cliff_sample_t READ_FAILED = {.valid = false, .floor = {true, true}};

/** Feed `s` n times from *t, one tick apart; return the last action. */
static cliff_action_t feed(cliff_t *c, const cliff_sample_t *s, unsigned n, uint32_t *t,
                           bool obstacle)
{
    cliff_action_t a = CLIFF_ACT_NONE;
    for (unsigned i = 0; i < n; i++) {
        a = cliff_step(c, s, *t, obstacle);
        *t += TICK_MS;
    }
    return a;
}

/** A state with both sensors on confirmed floor, at time *t. */
static void on_floor(cliff_t *c, uint32_t *t)
{
    cliff_reset(c);
    (void)feed(c, &BOTH_FLOOR, CLIFF_CLEAR_SAMPLES, t, false);
}

static void test_starts_closed(void)
{
    cliff_t c;
    cliff_reset(&c);
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD), "forward permitted before floor seen");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_BACKWARD), "backward permitted before floor seen");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_ROTATE), "rotate permitted before floor seen");
    CHECK(cliff_permits(&c, CLIFF_MOTION_NONE), "stop refused");

    /* Never having floor is not an edge: no reflex, no trip. */
    uint32_t t = 0;
    CHECK(feed(&c, &BOTH_GONE, 20, &t, false) == CLIFF_ACT_NONE, "boot without floor acted");
    CHECK(c.trips == 0, "boot without floor counted a trip");
}

static void test_clear_needs_samples(void)
{
    cliff_t c;
    cliff_reset(&c);
    uint32_t t = 0;
    (void)feed(&c, &BOTH_FLOOR, CLIFF_CLEAR_SAMPLES - 1, &t, false);
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD), "cleared before CLIFF_CLEAR_SAMPLES");
    (void)feed(&c, &BOTH_FLOOR, 1, &t, false);
    CHECK(cliff_permits(&c, CLIFF_MOTION_FORWARD), "not cleared after CLIFF_CLEAR_SAMPLES");
    CHECK(cliff_permits(&c, CLIFF_MOTION_ROTATE), "rotate refused on floor");
}

static void test_single_glitch_ignored(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    CHECK(feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES - 1, &t, false) == CLIFF_ACT_NONE,
          "tripped on fewer than CLIFF_TRIP_SAMPLES");
    (void)feed(&c, &BOTH_FLOOR, 1, &t, false);
    CHECK(feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES - 1, &t, false) == CLIFF_ACT_NONE,
          "a floor sample did not reset the trip count");
    CHECK(c.trips == 0, "glitches counted a trip");
}

static void test_front_edge_brakes_then_reverses(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);

    const cliff_action_t a = feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES, &t, false);
    CHECK(a == CLIFF_ACT_BRAKE, "front edge: got %d, want BRAKE", a);
    CHECK(c.trips == 1, "trips %u, want 1", (unsigned)c.trips);
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD), "forward permitted over a front edge");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_ROTATE), "rotate permitted over a front edge");
    CHECK(cliff_permits(&c, CLIFF_MOTION_BACKWARD), "backward refused with floor behind");

    /* Brake holds for CLIFF_BRAKE_MS from the edge tick. */
    const uint32_t edge_ms = t - TICK_MS;
    cliff_action_t next;
    while ((next = feed(&c, &FRONT_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(next == CLIFF_ACT_REVERSE, "after brake: got %d, want REVERSE", next);
    const uint32_t reverse_ms = t - TICK_MS;
    CHECK(reverse_ms - edge_ms >= CLIFF_BRAKE_MS, "braked only %u ms",
          (unsigned)(reverse_ms - edge_ms));
    CHECK(reverse_ms - edge_ms < CLIFF_BRAKE_MS + TICK_MS, "braked %u ms, a tick too long",
          (unsigned)(reverse_ms - edge_ms));

    /* Reverse lasts CLIFF_BACKOFF_MS, then the wheels are handed back. */
    while ((next = feed(&c, &FRONT_GONE, 1, &t, false)) == CLIFF_ACT_REVERSE) {
    }
    CHECK(next == CLIFF_ACT_NONE, "after back-off: got %d, want NONE", next);
    const uint32_t done_ms = t - TICK_MS;
    CHECK(done_ms - reverse_ms >= CLIFF_BACKOFF_MS, "backed off only %u ms",
          (unsigned)(done_ms - reverse_ms));
    CHECK(done_ms - reverse_ms < CLIFF_BACKOFF_MS + TICK_MS, "backed off a tick too long");

    /* Still no floor ahead: no second reflex, forward still refused. */
    CHECK(feed(&c, &FRONT_GONE, 30, &t, false) == CLIFF_ACT_NONE, "re-tripped without an edge");
    CHECK(c.trips == 1, "trips %u after holding at the edge", (unsigned)c.trips);
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD), "forward permitted at the edge");
}

static void test_rear_edge_pulls_forward(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    CHECK(feed(&c, &REAR_GONE, CLIFF_TRIP_SAMPLES, &t, false) == CLIFF_ACT_BRAKE,
          "rear edge did not brake");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_BACKWARD), "backward permitted over a rear edge");
    CHECK(cliff_permits(&c, CLIFF_MOTION_FORWARD), "forward refused with floor ahead");
    cliff_action_t a;
    while ((a = feed(&c, &REAR_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_FORWARD, "rear edge: got %d, want FORWARD", a);
}

static void test_no_backoff_toward_missing_floor(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    /* Both edges at once (lifted, or both sensors unplugged): brake, no back-off. */
    CHECK(feed(&c, &BOTH_GONE, CLIFF_TRIP_SAMPLES, &t, false) == CLIFF_ACT_BRAKE,
          "both gone did not brake");
    cliff_action_t a;
    while ((a = feed(&c, &BOTH_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_NONE, "both gone: got %d, want NONE (no back-off)", a);
    CHECK(c.trips == 1, "simultaneous edges counted %u trips", (unsigned)c.trips);
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD) && !cliff_permits(&c, CLIFF_MOTION_BACKWARD),
          "motion permitted with no floor either side");
}

static void test_rear_pull_skipped_with_obstacle(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    (void)feed(&c, &REAR_GONE, CLIFF_TRIP_SAMPLES, &t, true);
    cliff_action_t a;
    while ((a = feed(&c, &REAR_GONE, 1, &t, true)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_NONE, "pulled forward into an obstacle: got %d", a);

    /* A front edge still reverses with an obstacle ahead: the obstacle is the
     * other way. */
    on_floor(&c, &t);
    (void)feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES, &t, true);
    while ((a = feed(&c, &FRONT_GONE, 1, &t, true)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_REVERSE, "front edge with obstacle: got %d, want REVERSE", a);
}

static void test_backoff_stops_at_second_edge(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    (void)feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES, &t, false);
    cliff_action_t a;
    while ((a = feed(&c, &FRONT_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_REVERSE, "no reverse");
    /* Reversing onto a rear edge: brake again, and no pull-away forward,
     * because the front still has no floor. */
    a = feed(&c, &BOTH_GONE, CLIFF_TRIP_SAMPLES, &t, false);
    CHECK(a == CLIFF_ACT_BRAKE, "second edge mid back-off: got %d, want BRAKE", a);
    CHECK(c.trips == 2, "trips %u, want 2", (unsigned)c.trips);
    while ((a = feed(&c, &BOTH_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    CHECK(a == CLIFF_ACT_NONE, "moved after both edges: got %d", a);
}

static void test_read_failure_fails_closed(void)
{
    cliff_t c;
    uint32_t t = 0;
    on_floor(&c, &t);
    CHECK(feed(&c, &READ_FAILED, CLIFF_TRIP_SAMPLES, &t, false) == CLIFF_ACT_BRAKE,
          "failed reads did not brake");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_FORWARD), "forward permitted after failed reads");
    CHECK(!cliff_permits(&c, CLIFF_MOTION_BACKWARD), "backward permitted after failed reads");
}

static void test_ms_wrap(void)
{
    cliff_t c;
    uint32_t t = UINT32_MAX - 2u * TICK_MS;
    on_floor(&c, &t);
    (void)feed(&c, &FRONT_GONE, CLIFF_TRIP_SAMPLES, &t, false);
    const uint32_t edge_ms = t - TICK_MS;
    cliff_action_t a;
    while ((a = feed(&c, &FRONT_GONE, 1, &t, false)) == CLIFF_ACT_BRAKE) {
    }
    const uint32_t reverse_ms = t - TICK_MS;
    CHECK(a == CLIFF_ACT_REVERSE, "across the wrap: got %d, want REVERSE", a);
    CHECK((uint32_t)(reverse_ms - edge_ms) < CLIFF_BRAKE_MS + TICK_MS,
          "brake stretched across the wrap: %u ms", (unsigned)(reverse_ms - edge_ms));
}

int main(void)
{
    printf("=== cliff_core host tests ===\n");
    test_starts_closed();
    test_clear_needs_samples();
    test_single_glitch_ignored();
    test_front_edge_brakes_then_reverses();
    test_rear_edge_pulls_forward();
    test_no_backoff_toward_missing_floor();
    test_rear_pull_skipped_with_obstacle();
    test_backoff_stops_at_second_edge();
    test_read_failure_fails_closed();
    test_ms_wrap();
    if (s_failures) {
        printf("FAILED: %d check(s)\n", s_failures);
        return 1;
    }
    printf("All cliff_core tests passed\n");
    return 0;
}
