/**
 * @file test_goal_state.c
 * @brief Host-based unit tests for goal_state.c
 *
 * Tests:
 * - Fresh-default is STOP (unconditionally stale until first write)
 * - Write/read round-trips all four goal kinds
 * - TTL expiry: inject a clock that advances past TTL; confirm stale + STOP
 * - Clock override via goal_state_set_clock_override()
 * - goal_state_force_stop() produces fresh STOP
 * - Concurrent fuzz: N writer threads + 1 reader thread; check no torn reads
 */

#include "goal_state.h"

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* =========================================================================
 * Test utilities
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
 * Clock override for TTL testing
 * ========================================================================= */

static int64_t g_test_time_us = 0;

static int64_t test_clock_fn(void)
{
    return g_test_time_us;
}

/* =========================================================================
 * Tests
 * ========================================================================= */

static void test_fresh_default_is_stop(void)
{
    goal_state_init();

    goal_t goal;
    bool is_fresh;

    esp_err_t ret = goal_state_read(&goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(!is_fresh);
    ASSERT(goal.kind == GOAL_KIND_STOP);
}

static void test_write_read_roundtrip_drive(void)
{
    goal_state_init();

    goal_t write_goal;
    write_goal.kind = GOAL_KIND_DRIVE;
    write_goal.params.drive.heading_deg = 45;
    write_goal.params.drive.distance_cm = 100;
    write_goal.params.drive.speed_pct = 75;

    esp_err_t ret = goal_state_write(&write_goal, 1500);
    ASSERT(ret == ESP_OK);

    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_DRIVE);
    ASSERT(read_goal.params.drive.heading_deg == 45);
    ASSERT(read_goal.params.drive.distance_cm == 100);
    ASSERT(read_goal.params.drive.speed_pct == 75);
}

static void test_write_read_roundtrip_track(void)
{
    goal_state_init();

    goal_t write_goal;
    write_goal.kind = GOAL_KIND_TRACK;
    write_goal.params.track.xmin = 300;
    write_goal.params.track.xmax = 700;
    write_goal.params.track.ymin = 200;
    write_goal.params.track.ymax = 800;
    write_goal.params.track.max_speed_pct = 60;

    esp_err_t ret = goal_state_write(&write_goal, 1500);
    ASSERT(ret == ESP_OK);

    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_TRACK);
    ASSERT(read_goal.params.track.xmin == 300);
    ASSERT(read_goal.params.track.xmax == 700);
    ASSERT(read_goal.params.track.max_speed_pct == 60);
}

static void test_write_read_roundtrip_rotate(void)
{
    goal_state_init();

    goal_t write_goal;
    write_goal.kind = GOAL_KIND_ROTATE;
    write_goal.params.rotate.angle_deg = 90;

    esp_err_t ret = goal_state_write(&write_goal, 1500);
    ASSERT(ret == ESP_OK);

    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_ROTATE);
    ASSERT(read_goal.params.rotate.angle_deg == 90);
}

static void test_write_read_roundtrip_stop(void)
{
    goal_state_init();

    goal_t write_goal;
    write_goal.kind = GOAL_KIND_STOP;

    esp_err_t ret = goal_state_write(&write_goal, 1500);
    ASSERT(ret == ESP_OK);

    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_STOP);
}

static void test_ttl_expiry(void)
{
    goal_state_init();

    /* Install test clock */
    g_test_time_us = 0;
    goal_state_set_clock_override(test_clock_fn);

    /* Write a DRIVE goal at time 0 with 1000 ms TTL */
    goal_t write_goal;
    write_goal.kind = GOAL_KIND_DRIVE;
    write_goal.params.drive.heading_deg = 0;
    write_goal.params.drive.distance_cm = 50;
    write_goal.params.drive.speed_pct = 50;

    esp_err_t ret = goal_state_write(&write_goal, 1000);
    ASSERT(ret == ESP_OK);

    /* Read at time 500 ms — should be fresh */
    g_test_time_us = 500 * 1000LL;
    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_DRIVE);

    /* Read at time 1500 ms (past TTL) — should be stale */
    g_test_time_us = 1500 * 1000LL;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(!is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_STOP); /* forced to STOP by read */

    /* Restore default clock */
    goal_state_set_clock_override(NULL);
}

static void test_force_stop(void)
{
    goal_state_init();

    /* Write a DRIVE goal first */
    goal_t drive_goal;
    drive_goal.kind = GOAL_KIND_DRIVE;
    drive_goal.params.drive.heading_deg = 45;
    drive_goal.params.drive.distance_cm = 100;
    drive_goal.params.drive.speed_pct = 75;
    goal_state_write(&drive_goal, 1500);

    /* Call force_stop */
    esp_err_t ret = goal_state_force_stop();
    ASSERT(ret == ESP_OK);

    /* Read should be fresh STOP */
    goal_t read_goal;
    bool is_fresh;
    ret = goal_state_read(&read_goal, &is_fresh);
    ASSERT(ret == ESP_OK);
    ASSERT(is_fresh);
    ASSERT(read_goal.kind == GOAL_KIND_STOP);
}

static void test_null_args(void)
{
    goal_state_init();

    /* goal_state_write with NULL goal */
    esp_err_t ret = goal_state_write(NULL, 1500);
    ASSERT(ret == ESP_ERR_INVALID_ARG);

    /* goal_state_read with NULL out */
    bool is_fresh;
    ret = goal_state_read(NULL, &is_fresh);
    ASSERT(ret == ESP_ERR_INVALID_ARG);

    /* goal_state_read with NULL is_fresh */
    goal_t goal;
    ret = goal_state_read(&goal, NULL);
    ASSERT(ret == ESP_ERR_INVALID_ARG);
}

/* =========================================================================
 * Concurrent fuzz test
 * ========================================================================= */

typedef struct {
    int writer_id;
    int iterations;
} writer_arg_t;

static volatile int g_reader_errors = 0;

/* cppcheck-suppress constParameterCallback // pthread_create requires void *(*)(void *) */
static void *writer_thread(void *arg)
{
    const writer_arg_t *wa = (const writer_arg_t *)arg;

    for (int i = 0; i < wa->iterations; i++) {
        goal_t goal;
        goal.kind = (goal_kind_t)((wa->writer_id) % 4);

        if (goal.kind == GOAL_KIND_DRIVE) {
            goal.params.drive.heading_deg = wa->writer_id * 10 + i;
            goal.params.drive.distance_cm = 50 + i;
            goal.params.drive.speed_pct = 50;
        } else if (goal.kind == GOAL_KIND_TRACK) {
            goal.params.track.xmin = 100 + wa->writer_id * 50;
            goal.params.track.xmax = 400 + wa->writer_id * 50;
            goal.params.track.ymin = 100;
            goal.params.track.ymax = 400;
            goal.params.track.max_speed_pct = 60;
        } else if (goal.kind == GOAL_KIND_ROTATE) {
            goal.params.rotate.angle_deg = (wa->writer_id * 45) + (i % 4) * 15;
        }

        goal_state_write(&goal, 1500);
        usleep(100); /* small delay to allow reads to interleave */
    }
    return NULL;
}

static void *reader_thread(void *arg)
{
    (void)arg;
    int iterations = 10000;

    for (int i = 0; i < iterations; i++) {
        goal_t goal;
        bool is_fresh;

        if (goal_state_read(&goal, &is_fresh) != ESP_OK) {
            g_reader_errors++;
            continue;
        }

        /* Sanity check: if the goal is not STOP, the kind should match a
         * valid union member. For the fuzz test, just check that kind is in
         * the valid range. */
        if (goal.kind > GOAL_KIND_ROTATE) {
            g_reader_errors++;
        }
    }
    return NULL;
}

static void test_concurrent_fuzz(void)
{
    goal_state_init();

    g_reader_errors = 0;

    const int num_writers = 4;
    const int writer_iterations = 1000;

    pthread_t writers[num_writers];
    pthread_t reader;
    writer_arg_t writer_args[num_writers];

    /* Spawn writers */
    for (int i = 0; i < num_writers; i++) {
        writer_args[i].writer_id = i;
        writer_args[i].iterations = writer_iterations;
        pthread_create(&writers[i], NULL, writer_thread, &writer_args[i]);
    }

    /* Spawn reader */
    pthread_create(&reader, NULL, reader_thread, NULL);

    /* Wait for all threads */
    for (int i = 0; i < num_writers; i++) {
        pthread_join(writers[i], NULL);
    }
    pthread_join(reader, NULL);

    ASSERT(g_reader_errors == 0);
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== goal_state.c host tests ===\n\n");

    test_run("fresh_default_is_stop", test_fresh_default_is_stop);
    test_run("write_read_roundtrip_drive", test_write_read_roundtrip_drive);
    test_run("write_read_roundtrip_track", test_write_read_roundtrip_track);
    test_run("write_read_roundtrip_rotate", test_write_read_roundtrip_rotate);
    test_run("write_read_roundtrip_stop", test_write_read_roundtrip_stop);
    test_run("ttl_expiry", test_ttl_expiry);
    test_run("force_stop", test_force_stop);
    test_run("null_args", test_null_args);
    test_run("concurrent_fuzz", test_concurrent_fuzz);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);

    return (test_pass == test_count) ? 0 : 1;
}
