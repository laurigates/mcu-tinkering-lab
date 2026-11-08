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
 * speak extraction — parallel function calls
 * ========================================================================= */

/* A `speak` call sitting in parts[1] alongside a motion call in parts[0].
 * The pre-speech parser only looked at parts[0], so this is the regression
 * that guards the multi-part scan. */
static void test_parse_speak_alongside_motion(void)
{
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":["
        "{\"functionCall\":{\"name\":\"drive\",\"args\":{\"heading_deg\":0,"
        "\"distance_cm\":50,\"speed_pct\":60}}},"
        "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"I see a red ball\"}}}"
        "]}}]}";
    goal_t goal;
    char speech[160];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_DRIVE);
    ASSERT(goal.params.drive.distance_cm == 50);
    ASSERT(strcmp(speech, "I see a red ball") == 0);
}

/* Order must not matter — speech first, motion second. */
static void test_parse_speak_before_motion(void)
{
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":["
        "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"Turning around\"}}},"
        "{\"functionCall\":{\"name\":\"rotate\",\"args\":{\"angle_deg\":180}}}"
        "]}}]}";
    goal_t goal;
    char speech[160];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_ROTATE);
    ASSERT(goal.params.rotate.angle_deg == 180);
    ASSERT(strcmp(speech, "Turning around") == 0);
}

/* Speech with no motion call: fail-safe STOP, but the utterance survives so
 * the robot can talk while holding position. */
static void test_parse_speak_only_still_yields_text(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"Hello\"}}}"
                       "]}}]}";
    goal_t goal;
    char speech[160];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_FAIL);
    ASSERT(goal.kind == GOAL_KIND_STOP);
    ASSERT(strcmp(speech, "Hello") == 0);
}

/* A `sound` call puts its tag at the front of the line, whichever order the
 * calls came in, and is never taken for the motion goal. */
static void test_parse_sound_leads_the_line(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"Pardon me\"}}},"
                       "{\"functionCall\":{\"name\":\"sound\",\"args\":{\"name\":\"horn\"}}},"
                       "{\"functionCall\":{\"name\":\"stop\",\"args\":{}}}"
                       "]}}]}";
    goal_t goal;
    char speech[160];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_STOP);
    ASSERT(strcmp(speech, "[horn] Pardon me") == 0);
}

/* A sound with no speech is a line of just the tag; the TTS task plays it
 * without a request. */
static void test_parse_sound_alone(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"functionCall\":{\"name\":\"sound\",\"args\":{\"name\":\"aoogah\"}}},"
                       "{\"functionCall\":{\"name\":\"drive\",\"args\":{\"heading_deg\":0,"
                       "\"distance_cm\":30,\"speed_pct\":50}}}"
                       "]}}]}";
    goal_t goal;
    char speech[160];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_DRIVE);
    ASSERT(strcmp(speech, "[aoogah]") == 0);
}

/* A name outside the enum plays nothing and leaves the speech alone. */
static void test_parse_unknown_sound_ignored(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"functionCall\":{\"name\":\"sound\",\"args\":{\"name\":\"trumpet\"}}},"
                       "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"Hello\"}}},"
                       "{\"functionCall\":{\"name\":\"stop\",\"args\":{}}}"
                       "]}}]}";
    goal_t goal;
    char speech[160];
    ASSERT(gemini_parse_response(json, &goal, speech, sizeof(speech)) == ESP_OK);
    ASSERT(strcmp(speech, "Hello") == 0);
}

/* No speak call → empty string, not stale contents. */
static void test_parse_no_speak_clears_buffer(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"functionCall\":{"
                       "\"name\":\"stop\",\"args\":{}}}]}}]}";
    goal_t goal;
    char speech[160];
    memcpy(speech, "stale", 6);
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_STOP);
    ASSERT(speech[0] == '\0');
}

/* Over-long utterances are truncated, not overflowed. */
static void test_parse_speak_truncates(void)
{
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":["
        "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"abcdefghijklmnop\"}}},"
        "{\"functionCall\":{\"name\":\"stop\",\"args\":{}}}"
        "]}}]}";
    goal_t goal;
    char speech[8];
    esp_err_t ret = gemini_parse_response(json, &goal, speech, sizeof(speech));
    ASSERT(ret == ESP_OK);
    ASSERT(strlen(speech) == 7);
    ASSERT(strcmp(speech, "abcdefg") == 0);
}

/* The legacy two-arg entry point must keep behaving exactly as before. */
static void test_legacy_wrapper_unchanged(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"functionCall\":{\"name\":\"speak\",\"args\":{\"text\":\"ignored\"}}},"
                       "{\"functionCall\":{\"name\":\"rotate\",\"args\":{\"angle_deg\":-45}}}"
                       "]}}]}";
    goal_t goal;
    esp_err_t ret = gemini_parse_function_call(json, &goal);
    ASSERT(ret == ESP_OK);
    ASSERT(goal.kind == GOAL_KIND_ROTATE);
    ASSERT(goal.params.rotate.angle_deg == -45);
}

/* =========================================================================
 * Main
 * ========================================================================= */

/* =========================================================================
 * gemini_parse_text — plain TEXT replies (the narrate + voice-turn path)
 *
 * This path had no host test at all while it lived static inside
 * gemini_backend.c. It is worth one now for a specific reason: a reply that
 * sanitises to nothing must be reported as FAILURE, not as an empty success,
 * or the caller cheerfully hands "" to the TTS renderer and the robot performs
 * a silence that looks exactly like a broken speaker.
 * ========================================================================= */

static void test_text_single_part(void)
{
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"Hei, mita kuuluu?\"}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(strcmp(out, "Hei, mita kuuluu?") == 0);
}

static void test_text_concatenates_parts_in_order(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"one \"},{\"text\":\"two \"},{\"text\":\"three\"}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(strcmp(out, "one two three") == 0);
}

static void test_text_collapses_layout_to_one_line(void)
{
    /* The TTS renderer pronounces layout: a leading newline is an audible pause
     * before the robot says anything. */
    const char *json =
        "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"\\n  a\\tb\\r\\n  \"}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(strcmp(out, "a b") == 0);
}

static void test_text_whitespace_only_is_a_failure(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"  \\n \"}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) != ESP_OK);
}

static void test_text_missing_parts_is_a_failure(void)
{
    /* What a MAX_TOKENS truncation actually looks like: a candidate with no
     * parts array at all. Must not read as an empty success. */
    const char *json = "{\"candidates\":[{\"finishReason\":\"MAX_TOKENS\"}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) != ESP_OK);
}

static void test_text_truncates_without_overrunning(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"aaaaaaaaaa\"},{\"text\":\"bbbbbbbbbb\"}]}}]}";
    char out[8];
    memset(out, 0x7F, sizeof(out));
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(out[sizeof(out) - 1] == '\0');
    ASSERT(strlen(out) == sizeof(out) - 1);
}

static void test_text_rejects_null_and_bad_json(void)
{
    char out[16] = {0};
    ASSERT(gemini_parse_text(NULL, out, sizeof(out)) == ESP_ERR_INVALID_ARG);
    ASSERT(gemini_parse_text("{}", NULL, sizeof(out)) == ESP_ERR_INVALID_ARG);
    ASSERT(gemini_parse_text("{}", out, 0) == ESP_ERR_INVALID_ARG);
    ASSERT(gemini_parse_text("not json", out, sizeof(out)) == ESP_FAIL);
}

/* =========================================================================
 * Thought parts (issue #950)
 *
 * On 2026-11-05 a voice turn spoke 34.8 s of the model's English analysis
 * ("The user asks ...") instead of a one-line Finnish answer. A part flagged
 * "thought": true is reasoning, never speech, whatever the request asked for.
 * ========================================================================= */

static void test_text_skips_thought_parts(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"The user asks what I see.\",\"thought\":true},"
                       "{\"text\":\"Näen pallon.\"}]},\"finishReason\":\"STOP\"}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(strcmp(out, "Näen pallon.") == 0);
}

static void test_text_only_thought_parts_is_a_failure(void)
{
    /* Reasoning with no answer must not be spoken as the answer. */
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"The user asks what I see.\",\"thought\":true}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) != ESP_OK);
}

static void test_text_keeps_parts_with_thought_false(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"Hei.\",\"thought\":false}]}}]}";
    char out[64] = {0};
    ASSERT(gemini_parse_text(json, out, sizeof(out)) == ESP_OK);
    ASSERT(strcmp(out, "Hei.") == 0);
}

static void test_text_meta_counts_parts_and_reads_finish_reason(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"plan\",\"thought\":true},"
                       "{\"text\":\"Hei \"},{\"thoughtSignature\":\"abc\"},{\"text\":\"vaan.\"}]},"
                       "\"finishReason\":\"STOP\"}]}";
    char out[64] = {0};
    gemini_text_meta_t meta;
    memset(&meta, 0x5A, sizeof(meta));
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) == ESP_OK);
    ASSERT(strcmp(out, "Hei vaan.") == 0);
    ASSERT(meta.parts == 4);
    ASSERT(meta.thought_parts == 1);
    ASSERT(meta.text_parts == 2);
    ASSERT(strcmp(meta.finish_reason, "STOP") == 0);
}

static void test_text_meta_is_filled_on_failure(void)
{
    /* The failure log is where the cause is needed most. */
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"reasoning\",\"thought\":true}]},"
                       "\"finishReason\":\"MAX_TOKENS\"}]}";
    char out[64] = {0};
    gemini_text_meta_t meta;
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) != ESP_OK);
    ASSERT(meta.parts == 1);
    ASSERT(meta.thought_parts == 1);
    ASSERT(meta.text_parts == 0);
    ASSERT(strcmp(meta.finish_reason, "MAX_TOKENS") == 0);
}

static void test_text_meta_truncates_a_long_finish_reason(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"x\"}]},"
                       "\"finishReason\":\"A_FINISH_REASON_LONGER_THAN_ANY_BUFFER_WOULD_HOLD\"}]}";
    char out[16] = {0};
    gemini_text_meta_t meta;
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) == ESP_OK);
    ASSERT(strlen(meta.finish_reason) == sizeof(meta.finish_reason) - 1);
    ASSERT(strncmp(meta.finish_reason, "A_FINISH_REASON", 15) == 0);
}

static void test_text_meta_counts_parts_after_the_buffer_fills(void)
{
    /* The log line reports the reply's shape even when the text is cut: a
     * part after the buffer filled still counts. */
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":["
                       "{\"text\":\"aaaaaaaaaa\"},{\"text\":\"bbb\"},"
                       "{\"text\":\"why\",\"thought\":true},{\"text\":\"ccc\"}]}}]}";
    char out[8];
    gemini_text_meta_t meta;
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) == ESP_OK);
    ASSERT(strcmp(out, "aaaaaaa") == 0);
    ASSERT(meta.parts == 4);
    ASSERT(meta.text_parts == 3);
    ASSERT(meta.thought_parts == 1);
}

static void test_text_meta_keeps_every_real_finish_reason_whole(void)
{
    /* The longest FinishReason values in google-genai 2.34; the log must name
     * the cause, not a prefix shared with another one (PROHIBITED_CONT...). */
    const char *reasons[] = {"FINISH_REASON_UNSPECIFIED", "IMAGE_PROHIBITED_CONTENT",
                             "MALFORMED_FUNCTION_CALL", "PROHIBITED_CONTENT"};
    for (size_t i = 0; i < sizeof(reasons) / sizeof(reasons[0]); ++i) {
        char json[160];
        snprintf(json, sizeof(json),
                 "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"x\"}]},"
                 "\"finishReason\":\"%s\"}]}",
                 reasons[i]);
        char out[16] = {0};
        gemini_text_meta_t meta;
        ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) == ESP_OK);
        ASSERT(strcmp(meta.finish_reason, reasons[i]) == 0);
    }
}

static void test_text_meta_without_finish_reason_is_empty(void)
{
    const char *json = "{\"candidates\":[{\"content\":{\"parts\":[{\"text\":\"x\"}]}}]}";
    char out[16] = {0};
    gemini_text_meta_t meta;
    memset(&meta, 0x5A, sizeof(meta));
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), &meta) == ESP_OK);
    ASSERT(meta.finish_reason[0] == '\0');
    /* And a NULL meta is allowed: that is what gemini_parse_text() passes. */
    ASSERT(gemini_parse_text_meta(json, out, sizeof(out), NULL) == ESP_OK);
}

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
    test_run("parse_speak_alongside_motion", test_parse_speak_alongside_motion);
    test_run("parse_speak_before_motion", test_parse_speak_before_motion);
    test_run("parse_speak_only_still_yields_text", test_parse_speak_only_still_yields_text);
    test_run("parse_sound_leads_the_line", test_parse_sound_leads_the_line);
    test_run("parse_sound_alone", test_parse_sound_alone);
    test_run("parse_unknown_sound_ignored", test_parse_unknown_sound_ignored);
    test_run("parse_no_speak_clears_buffer", test_parse_no_speak_clears_buffer);
    test_run("parse_speak_truncates", test_parse_speak_truncates);
    test_run("legacy_wrapper_unchanged", test_legacy_wrapper_unchanged);

    test_run("text_single_part", test_text_single_part);
    test_run("text_concatenates_parts_in_order", test_text_concatenates_parts_in_order);
    test_run("text_collapses_layout_to_one_line", test_text_collapses_layout_to_one_line);
    test_run("text_whitespace_only_is_a_failure", test_text_whitespace_only_is_a_failure);
    test_run("text_missing_parts_is_a_failure", test_text_missing_parts_is_a_failure);
    test_run("text_truncates_without_overrunning", test_text_truncates_without_overrunning);
    test_run("text_rejects_null_and_bad_json", test_text_rejects_null_and_bad_json);
    test_run("text_skips_thought_parts", test_text_skips_thought_parts);
    test_run("text_only_thought_parts_is_a_failure", test_text_only_thought_parts_is_a_failure);
    test_run("text_keeps_parts_with_thought_false", test_text_keeps_parts_with_thought_false);
    test_run("text_meta_counts_parts_and_reads_finish_reason",
             test_text_meta_counts_parts_and_reads_finish_reason);
    test_run("text_meta_is_filled_on_failure", test_text_meta_is_filled_on_failure);
    test_run("text_meta_truncates_a_long_finish_reason",
             test_text_meta_truncates_a_long_finish_reason);
    test_run("text_meta_counts_parts_after_the_buffer_fills",
             test_text_meta_counts_parts_after_the_buffer_fills);
    test_run("text_meta_keeps_every_real_finish_reason_whole",
             test_text_meta_keeps_every_real_finish_reason_whole);
    test_run("text_meta_without_finish_reason_is_empty",
             test_text_meta_without_finish_reason_is_empty);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
