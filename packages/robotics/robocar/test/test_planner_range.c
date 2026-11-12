/**
 * @file test_planner_range.c
 * @brief Host tests for the planner prompt's rangefinder clause.
 */
#include "planner_range.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ultrasonic_core.h"

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        fflush(stdout);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static const planner_range_t k_clear = {
    .front_cm = 120,
    .front_failed = false,
    .reflex_active = false,
    .rear_cm = 45,
    .rear_failed = false,
};

static void test_states_both_distances(void)
{
    char out[PLANNER_RANGE_CLAUSE_MAX];
    const size_t n = planner_range_clause(&k_clear, out, sizeof(out));
    ASSERT(n == strlen(out));
    ASSERT(strstr(out, "ahead 120 cm") != NULL);
    ASSERT(strstr(out, "behind 45 cm") != NULL);
    ASSERT(strstr(out, "not from how large things look") != NULL);
    ASSERT(strstr(out, "reflex") == NULL);
}

static void test_reflex_names_what_it_holds(void)
{
    planner_range_t r = k_clear;
    r.front_cm = 12;
    r.reflex_active = true;
    char out[PLANNER_RANGE_CLAUSE_MAX];
    ASSERT(planner_range_clause(&r, out, sizeof(out)) > 0);
    ASSERT(strstr(out, "ahead 12 cm") != NULL);
    ASSERT(strstr(out, "holding forward motion") != NULL);
}

/* The filter maps a failed read to max range; the clause must not pass that
 * on as a clear path. */
static void test_failed_sensor_is_unavailable_not_clear(void)
{
    planner_range_t r = k_clear;
    r.front_cm = ULTRASONIC_MAX_RANGE_CM;
    r.front_failed = true;
    r.rear_cm = ULTRASONIC_DIST_ERROR;
    char out[PLANNER_RANGE_CLAUSE_MAX];
    ASSERT(planner_range_clause(&r, out, sizeof(out)) > 0);
    ASSERT(strstr(out, "ahead unavailable") != NULL);
    ASSERT(strstr(out, "behind unavailable") != NULL);
    ASSERT(strstr(out, "no echo") == NULL);
}

static void test_max_range_is_no_echo(void)
{
    planner_range_t r = k_clear;
    r.front_cm = ULTRASONIC_MAX_RANGE_CM;
    char out[PLANNER_RANGE_CLAUSE_MAX];
    ASSERT(planner_range_clause(&r, out, sizeof(out)) > 0);
    ASSERT(strstr(out, "ahead no echo within 400 cm") != NULL);
}

/* The longest clause (both sensors failed, reflex active) fits the bound the
 * prompt reserves for it. */
static void test_longest_clause_fits_the_max(void)
{
    planner_range_t r = k_clear;
    r.front_failed = true;
    r.rear_failed = true;
    r.reflex_active = true;
    char out[PLANNER_RANGE_CLAUSE_MAX];
    ASSERT(planner_range_clause(&r, out, sizeof(out)) > 0);
}

static void test_short_buffer_writes_nothing(void)
{
    char out[32];
    memset(out, 'x', sizeof(out));
    ASSERT(planner_range_clause(&k_clear, out, sizeof(out)) == 0);
    ASSERT(out[0] == '\0');
    ASSERT(planner_range_clause(NULL, out, sizeof(out)) == 0);
    ASSERT(out[0] == '\0');
}

int main(void)
{
    test_states_both_distances();
    test_reflex_names_what_it_holds();
    test_failed_sensor_is_unavailable_not_clear();
    test_max_range_is_no_echo();
    test_longest_clause_fits_the_max();
    test_short_buffer_writes_nothing();
    printf("planner_range: all tests passed\n");
    return 0;
}
