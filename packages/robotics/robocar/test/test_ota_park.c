/**
 * @file test_ota_park.c
 * @brief Host tests for the pre-OTA clearance park (ota_park.c).
 *
 * The cases a bench cannot stage on demand: a room with no open heading, a
 * front sensor that fails mid-turn, a wall right behind, and the uint32
 * millisecond wrap in the middle of a turning step.
 */

#include "ota_park.h"

#include <assert.h>
#include <stdio.h>

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

static const ota_park_config_t CFG = {.open_cm = 80,
                                      .rear_min_cm = 30,
                                      .settle_ms = 700,
                                      .rotate_ms = 250,
                                      .nudge_ms = 400,
                                      .max_steps = 4};

/* A room whose front clearance depends on how many turning steps were taken. */
typedef struct {
    const uint16_t *front_by_step; /* indexed by steps taken so far */
    uint16_t rear_cm;
    bool rear_ok;
    int fail_front_at_step; /* -1: never */
} room_t;

typedef struct {
    int rotate_ms, forward_ms, stop_ms;
    uint32_t end_ms;
} trace_t;

/* Step every 10 ms until done or the time budget runs out. */
static trace_t drive(ota_park_t *p, const room_t *room, uint32_t start_ms, uint32_t budget_ms)
{
    trace_t tr = {0};
    ota_park_start(p, &CFG, start_ms);
    uint32_t t = start_ms;
    for (uint32_t elapsed = 0; elapsed <= budget_ms; elapsed += 10, t += 10) {
        const int s = p->steps;
        const bool front_ok = room->fail_front_at_step < 0 || s < room->fail_front_at_step;
        const ota_park_cmd_t cmd =
            ota_park_step(p, t, room->front_by_step[s], front_ok, room->rear_cm, room->rear_ok);
        if (ota_park_result(p) != OTA_PARK_RESULT_NONE) {
            ASSERT(cmd == OTA_PARK_CMD_STOP);
            tr.end_ms = t;
            return tr;
        }
        if (cmd == OTA_PARK_CMD_ROTATE) {
            tr.rotate_ms += 10;
        } else if (cmd == OTA_PARK_CMD_FORWARD) {
            tr.forward_ms += 10;
        } else {
            tr.stop_ms += 10;
        }
    }
    tr.end_ms = t;
    return tr;
}

static void test_already_open_parks_without_turning(void)
{
    const uint16_t front[] = {200, 200, 200, 200, 200};
    const room_t room = {front, 100, true, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 1000, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
    ASSERT(tr.rotate_ms == 0);
    ASSERT(tr.forward_ms == 0);
    ASSERT(tr.stop_ms >= 700); /* it waited for the smoothed reading */
}

static void test_turns_until_the_front_is_open(void)
{
    const uint16_t front[] = {20, 30, 150, 20, 20};
    const room_t room = {front, 100, true, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
    ASSERT(p.steps == 2);
    ASSERT(tr.rotate_ms == 2 * 250);
}

static void test_no_open_heading_gives_up_after_max_steps(void)
{
    const uint16_t front[] = {20, 20, 20, 20, 20};
    const room_t room = {front, 100, true, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 20000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_BEST_EFFORT);
    ASSERT(p.steps == CFG.max_steps);
    ASSERT(tr.rotate_ms == CFG.max_steps * 250);
    ASSERT(tr.forward_ms == 0);
}

static void test_a_failed_front_sensor_stays_put(void)
{
    /* A failed sensor reads as max range; it must not count as open floor. */
    const uint16_t front[] = {400, 400, 400, 400, 400};
    const room_t room = {front, 100, true, 0};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_NO_SENSOR);
    ASSERT(tr.rotate_ms == 0 && tr.forward_ms == 0);
}

static void test_a_front_sensor_failing_mid_turn_stops(void)
{
    const uint16_t front[] = {20, 20, 400, 400, 400};
    const room_t room = {front, 100, true, 2};
    ota_park_t p;
    drive(&p, &room, 0, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_NO_SENSOR);
    ASSERT(p.steps == 2);
}

static void test_something_close_behind_nudges_forward(void)
{
    const uint16_t front[] = {200, 200, 200, 200, 200};
    const room_t room = {front, 10, true, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
    ASSERT(tr.forward_ms == 400);
}

static void test_a_failed_rear_sensor_does_not_nudge(void)
{
    const uint16_t front[] = {200, 200, 200, 200, 200};
    const room_t room = {front, 0, false, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
    ASSERT(tr.forward_ms == 0);
}

static void test_survives_the_millisecond_wrap(void)
{
    const uint16_t front[] = {20, 20, 150, 20, 20};
    const room_t room = {front, 100, true, -1};
    ota_park_t p;
    /* Start 600 ms before the wrap: the first settle and the first turn
     * straddle it. */
    const trace_t tr = drive(&p, &room, UINT32_MAX - 600u, 10000);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
    ASSERT(p.steps == 2);
    ASSERT(tr.rotate_ms == 2 * 250);
}

static void test_done_stays_done(void)
{
    const uint16_t front[] = {200, 200, 200, 200, 200};
    const room_t room = {front, 100, true, -1};
    ota_park_t p;
    const trace_t tr = drive(&p, &room, 0, 10000);
    ASSERT(ota_park_step(&p, tr.end_ms + 5000, 10, true, 5, true) == OTA_PARK_CMD_STOP);
    ASSERT(ota_park_result(&p) == OTA_PARK_RESULT_OPEN);
}

static void test_result_names(void)
{
    ASSERT(ota_park_result_name(OTA_PARK_RESULT_OPEN)[0] == 'o');
    ASSERT(ota_park_result_name(OTA_PARK_RESULT_BEST_EFFORT)[0] == 'b');
    ASSERT(ota_park_result_name(OTA_PARK_RESULT_NO_SENSOR)[0] == 'n');
    ASSERT(ota_park_result_name(OTA_PARK_RESULT_NONE)[0] == 'p');
}

int main(void)
{
    printf("=== ota_park host tests ===\n\n");
    test_run("already_open_parks_without_turning", test_already_open_parks_without_turning);
    test_run("turns_until_the_front_is_open", test_turns_until_the_front_is_open);
    test_run("no_open_heading_gives_up_after_max_steps",
             test_no_open_heading_gives_up_after_max_steps);
    test_run("a_failed_front_sensor_stays_put", test_a_failed_front_sensor_stays_put);
    test_run("a_front_sensor_failing_mid_turn_stops", test_a_front_sensor_failing_mid_turn_stops);
    test_run("something_close_behind_nudges_forward", test_something_close_behind_nudges_forward);
    test_run("a_failed_rear_sensor_does_not_nudge", test_a_failed_rear_sensor_does_not_nudge);
    test_run("survives_the_millisecond_wrap", test_survives_the_millisecond_wrap);
    test_run("done_stays_done", test_done_stays_done);
    test_run("result_names", test_result_names);
    printf("\n=== Results ===\nPassed: %d / %d\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
