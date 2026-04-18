/**
 * @file test_gemini_parse.c
 * @brief Host-based unit tests for gemini_parse_function_call().
 *
 * Loads JSON fixtures from test/fixtures/ (see fixtures/README.md for the
 * capture protocol) and asserts each parses into the expected goal_t plus
 * the fail-safe contract for corrupt/empty responses.
 *
 * The FIXTURE_DIR macro is injected from CMake so the binary can locate its
 * fixtures regardless of the working directory ctest uses.
 */

#include "gemini_parse.h"
#include "goal_state.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FIXTURE_DIR
#error "FIXTURE_DIR must be defined by the build system"
#endif

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
 * Fixture loader
 * ========================================================================= */

static char *load_fixture(const char *filename)
{
    char path[1024];
    int n = snprintf(path, sizeof(path), "%s/%s", FIXTURE_DIR, filename);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        printf("fixture path too long: %s/%s\n", FIXTURE_DIR, filename);
        return NULL;
    }

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        printf("failed to open fixture: %s\n", path);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (size <= 0) {
        fclose(fp);
        return NULL;
    }

    char *buf = malloc((size_t)size + 1);
    if (!buf) {
        fclose(fp);
        return NULL;
    }

    size_t read_n = fread(buf, 1, (size_t)size, fp);
    fclose(fp);
    buf[read_n] = '\0';
    return buf;
}

/* =========================================================================
 * Tests
 * ========================================================================= */

static void test_parse_drive(void)
{
    char *json = load_fixture("er_1_6_drive.json");
    ASSERT(json != NULL);

    goal_t goal = {0};
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_DRIVE);
    ASSERT(goal.params.drive.heading_deg == 0);
    ASSERT(goal.params.drive.distance_cm == 100);
    ASSERT(goal.params.drive.speed_pct == 60);

    free(json);
}

static void test_parse_track(void)
{
    char *json = load_fixture("er_1_6_track.json");
    ASSERT(json != NULL);

    goal_t goal = {0};
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_TRACK);
    /* Fixture box_2d: [380, 410, 620, 590] → [ymin, xmin, ymax, xmax] */
    ASSERT(goal.params.track.ymin == 380);
    ASSERT(goal.params.track.xmin == 410);
    ASSERT(goal.params.track.ymax == 620);
    ASSERT(goal.params.track.xmax == 590);
    ASSERT(goal.params.track.max_speed_pct == 55);

    free(json);
}

static void test_parse_rotate(void)
{
    char *json = load_fixture("er_1_6_rotate.json");
    ASSERT(json != NULL);

    goal_t goal = {0};
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_ROTATE);
    ASSERT(goal.params.rotate.angle_deg == -45);

    free(json);
}

static void test_parse_stop(void)
{
    char *json = load_fixture("er_1_6_stop.json");
    ASSERT(json != NULL);

    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE; /* poison — parser must overwrite */
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_STOP);

    free(json);
}

/* Corrupt response (no functionCall) → ESP_FAIL, kind forced to STOP. */
static void test_parse_corrupt_falls_back_to_stop(void)
{
    char *json = load_fixture("er_1_6_corrupt.json");
    ASSERT(json != NULL);

    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE;
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_FAIL);
    ASSERT(goal.kind == GOAL_KIND_STOP);

    free(json);
}

/* Completely invalid JSON → ESP_FAIL, fail-safe STOP. */
static void test_parse_invalid_json(void)
{
    const char *garbage = "this is not json at all };{[";
    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE;
    esp_err_t ret = gemini_parse_function_call(garbage, &goal);
    ASSERT(ret == ESP_FAIL);
    ASSERT(goal.kind == GOAL_KIND_STOP);
}

/* NULL pointer guard. */
static void test_parse_null_args(void)
{
    goal_t goal;
    ASSERT(gemini_parse_function_call(NULL, &goal) == ESP_ERR_INVALID_ARG);
    ASSERT(gemini_parse_function_call("{}", NULL) == ESP_ERR_INVALID_ARG);
}

/* Drive with missing speed_pct → fail-safe STOP. */
static void test_parse_drive_missing_field(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"functionCall\":{"
                       "\"name\":\"drive\",\"args\":{\"heading_deg\":0,\"distance_cm\":50}}}]}}]}";
    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE;
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_FAIL);
    ASSERT(goal.kind == GOAL_KIND_STOP);
}

/* Track with wrong-sized box_2d → fail-safe STOP. */
static void test_parse_track_wrong_box_length(void)
{
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":[{\"functionCall\":{"
        "\"name\":\"track\",\"args\":{\"box_2d\":[1,2,3],\"max_speed_pct\":50}}}]}}]}";
    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE;
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_FAIL);
    ASSERT(goal.kind == GOAL_KIND_STOP);
}

/* Unknown function name → ESP_OK with STOP goal (per parser contract). */
static void test_parse_unknown_function_defaults_to_stop(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"functionCall\":{"
                       "\"name\":\"launch_missiles\",\"args\":{}}}]}}]}";
    goal_t goal;
    goal.kind = GOAL_KIND_DRIVE;
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_STOP);
}

/* =========================================================================
 * Main
 * ========================================================================= */

int main(void)
{
    printf("=== gemini_parse host tests ===\n\n");

    test_run("parse_drive", test_parse_drive);
    test_run("parse_track", test_parse_track);
    test_run("parse_rotate", test_parse_rotate);
    test_run("parse_stop", test_parse_stop);
    test_run("parse_corrupt_falls_back_to_stop", test_parse_corrupt_falls_back_to_stop);
    test_run("parse_invalid_json", test_parse_invalid_json);
    test_run("parse_null_args", test_parse_null_args);
    test_run("parse_drive_missing_field", test_parse_drive_missing_field);
    test_run("parse_track_wrong_box_length", test_parse_track_wrong_box_length);
    test_run("parse_unknown_function_defaults_to_stop",
             test_parse_unknown_function_defaults_to_stop);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
