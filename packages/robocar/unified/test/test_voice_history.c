/**
 * @file test_voice_history.c
 * @brief Host-based unit tests for voice_history conversational memory and request formatting.
 */

#include "voice_history.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

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

#define ASSERT_REPLY_EQ(index, now, expected)                     \
    do {                                                          \
        const char *r_ = voice_history_get_reply((index), (now)); \
        ASSERT(r_ != NULL);                                       \
        ASSERT(strcmp(r_, (expected)) == 0);                      \
    } while (0)

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
 * Ring buffer tests
 * ========================================================================= */

static void test_initial_state_empty(void)
{
    voice_history_reset();
    ASSERT(voice_history_count(0) == 0);
    ASSERT(voice_history_get_reply(0, 0) == NULL);
}

static void test_single_turn_recording(void)
{
    voice_history_reset();
    voice_history_record("Hello world!", 1000);
    ASSERT(voice_history_count(1000) == 1);
    const char *reply = voice_history_get_reply(0, 1000);
    ASSERT(reply != NULL);
    ASSERT(strcmp(reply, "Hello world!") == 0);
    ASSERT(voice_history_get_reply(1, 1000) == NULL);
}

static void test_rolling_buffer_up_to_max(void)
{
    voice_history_reset();
    voice_history_record("Turn 1", 1000);
    voice_history_record("Turn 2", 2000);
    voice_history_record("Turn 3", 3000);
    voice_history_record("Turn 4", 4000);

    ASSERT(voice_history_count(4000) == 4);
    ASSERT_REPLY_EQ(0, 4000, "Turn 1");
    ASSERT_REPLY_EQ(1, 4000, "Turn 2");
    ASSERT_REPLY_EQ(2, 4000, "Turn 3");
    ASSERT_REPLY_EQ(3, 4000, "Turn 4");
    ASSERT(voice_history_get_reply(4, 4000) == NULL);
}

static void test_rolling_buffer_eviction(void)
{
    voice_history_reset();
    voice_history_record("Turn 1", 1000);
    voice_history_record("Turn 2", 2000);
    voice_history_record("Turn 3", 3000);
    voice_history_record("Turn 4", 4000);
    voice_history_record("Turn 5", 5000);

    /* Turn 1 evicted; buffer has Turns 2, 3, 4, 5 */
    ASSERT(voice_history_count(5000) == 4);
    ASSERT_REPLY_EQ(0, 5000, "Turn 2");
    ASSERT_REPLY_EQ(1, 5000, "Turn 3");
    ASSERT_REPLY_EQ(2, 5000, "Turn 4");
    ASSERT_REPLY_EQ(3, 5000, "Turn 5");

    /* Push another */
    voice_history_record("Turn 6", 6000);
    ASSERT(voice_history_count(6000) == 4);
    ASSERT_REPLY_EQ(0, 6000, "Turn 3");
    ASSERT_REPLY_EQ(1, 6000, "Turn 4");
    ASSERT_REPLY_EQ(2, 6000, "Turn 5");
    ASSERT_REPLY_EQ(3, 6000, "Turn 6");
}

static void test_idle_timeout_expiration(void)
{
    voice_history_reset();
    voice_history_record("Turn 1", 1000);

    /* Within 60 seconds (59 seconds elapsed): still valid */
    ASSERT(voice_history_count(60000) == 1);
    ASSERT(voice_history_get_reply(0, 60000) != NULL);

    /* Exactly 60 seconds elapsed: still valid */
    ASSERT(voice_history_count(61000) == 1);

    /* After 60 seconds (60001 ms elapsed): expired */
    ASSERT(voice_history_count(61001) == 0);
    ASSERT(voice_history_get_reply(0, 61001) == NULL);
}

static void test_record_after_expiration(void)
{
    voice_history_reset();
    voice_history_record("Old turn", 1000);

    /* Jump forward past expiration window */
    voice_history_record("New turn", 70000);

    /* Old turn must be cleared, only New turn present */
    ASSERT(voice_history_count(70000) == 1);
    ASSERT(strcmp(voice_history_get_reply(0, 70000), "New turn") == 0);
}

static void test_clock_wrap_around(void)
{
    voice_history_reset();
    /* Record near 32-bit unsigned wrap */
    const uint32_t t_record = 0xFFFFFFF0;
    voice_history_record("Wrap turn", t_record);

    /* Query at 1000 ms: elapsed is 1000 - 0xFFFFFFF0 = 1016 ms <= 60000 */
    ASSERT(voice_history_count(1000) == 1);
    ASSERT(voice_history_get_reply(0, 1000) != NULL);

    /* Query at 60000 ms: elapsed is 60000 - 0xFFFFFFF0 = 60016 ms > 60000 -> expired */
    ASSERT(voice_history_count(60000) == 0);
}

static void test_manual_reset(void)
{
    voice_history_reset();
    voice_history_record("A", 1000);
    voice_history_record("B", 2000);
    ASSERT(voice_history_count(2000) == 2);

    voice_history_reset();
    ASSERT(voice_history_count(2000) == 0);
    ASSERT(voice_history_get_reply(0, 2000) == NULL);
}

/* =========================================================================
 * Ignore filtering and active conversation window tests
 * ========================================================================= */

static void test_ignore_tag_detection(void)
{
    ASSERT(voice_history_is_ignore(NULL));
    ASSERT(voice_history_is_ignore(""));
    ASSERT(voice_history_is_ignore("   \t\r\n"));
    ASSERT(voice_history_is_ignore("__IGNORE__"));
    ASSERT(voice_history_is_ignore("__IGNORE__."));
    ASSERT(voice_history_is_ignore("  __IGNORE__"));
    ASSERT(voice_history_is_ignore("  __IGNORE__ (noise)"));
    ASSERT(voice_history_is_ignore("\n__IGNORE__"));

    ASSERT(!voice_history_is_ignore("Hello robot!"));
    ASSERT(!voice_history_is_ignore("I said __IGNORE__"));
    ASSERT(!voice_history_is_ignore("__IGNOR"));
    ASSERT(!voice_history_is_ignore("__ignore__"));
}

static void test_record_with_ignore_sentinel(void)
{
    voice_history_reset();
    voice_history_record("__IGNORE__", 1000);
    ASSERT(voice_history_count(1000) == 0);

    voice_history_record("  __IGNORE__ not addressed to robot", 2000);
    ASSERT(voice_history_count(2000) == 0);

    voice_history_record("A real reply.", 3000);
    ASSERT(voice_history_count(3000) == 1);
    ASSERT(strcmp(voice_history_get_reply(0, 3000), "A real reply.") == 0);

    /* Further __IGNORE__ does not affect count or eviction */
    voice_history_record("__IGNORE__", 4000);
    ASSERT(voice_history_count(4000) == 1);
    ASSERT(strcmp(voice_history_get_reply(0, 4000), "A real reply.") == 0);
}

static void test_active_conversation_window(void)
{
    voice_history_reset();
    ASSERT(!voice_history_in_conversation(1000));

    voice_history_mark_conversation_active(1000);
    ASSERT(voice_history_in_conversation(1000));
    ASSERT(voice_history_in_conversation(5000));
    ASSERT(voice_history_in_conversation(7999));

    /* 7000 ms elapsed -> window closed */
    ASSERT(!voice_history_in_conversation(8000));
    ASSERT(!voice_history_in_conversation(8001));
    ASSERT(!voice_history_in_conversation(15000));
}

static void test_conversation_window_wrap_around(void)
{
    voice_history_reset();
    const uint32_t t_record = 0xFFFFFFF0;
    voice_history_mark_conversation_active(t_record);

    ASSERT(voice_history_in_conversation(t_record));
    /* Elapsed is 1000 - 0xFFFFFFF0 = 1016 ms <= 7000 */
    ASSERT(voice_history_in_conversation(1000));
    /* Elapsed is 7000 - 0xFFFFFFF0 = 7016 ms > 7000 -> expired */
    ASSERT(!voice_history_in_conversation(7000));
}

static void test_conversation_window_reset(void)
{
    voice_history_reset();
    voice_history_mark_conversation_active(1000);
    ASSERT(voice_history_in_conversation(1000));

    voice_history_reset();
    ASSERT(!voice_history_in_conversation(1000));
}

/* =========================================================================
 * Telemetry and prompt formatting tests
 * ========================================================================= */

static void test_telemetry_formatting(void)
{
    char buf[128];

    /* NULL telemetry */
    ASSERT(!voice_history_format_telemetry(NULL, buf, sizeof(buf)));

    /* Normal obstacle reading */
    reactive_telemetry_t tele = {.distance_cm = 45, .reflex_active = false, .sensor_failed = false};
    ASSERT(voice_history_format_telemetry(&tele, buf, sizeof(buf)));
    ASSERT(strcmp(buf, "Physical state: distance to obstacle ahead is 45 cm.") == 0);

    /* Reflex active reading */
    tele.distance_cm = 8;
    tele.reflex_active = true;
    ASSERT(voice_history_format_telemetry(&tele, buf, sizeof(buf)));
    ASSERT(strcmp(buf,
                  "Physical state: distance to obstacle ahead is 8 cm (obstacle reflex active).") ==
           0);

    /* Sensor failed reading */
    tele.sensor_failed = true;
    ASSERT(voice_history_format_telemetry(&tele, buf, sizeof(buf)));
    ASSERT(strcmp(buf, "Physical state: ultrasonic sensor unavailable.") == 0);
}

static void test_prompt_building(void)
{
    char prompt[512];

    /* Default name, no image, no telemetry */
    voice_history_build_prompt(NULL, false, NULL, prompt, sizeof(prompt));
    ASSERT(strstr(prompt, "You are a small wheeled robot named Robocar.") != NULL);
    ASSERT(strstr(prompt, "the audio follows.") != NULL);
    ASSERT(strstr(prompt, "Physical state:") == NULL);

    /* Persona name Teuvo, with image, no telemetry */
    voice_history_build_prompt("Teuvo", true, "", prompt, sizeof(prompt));
    ASSERT(strstr(prompt, "You are a small wheeled robot named Teuvo.") != NULL);
    ASSERT(strstr(prompt, "the camera view and audio follow.") != NULL);
    ASSERT(strstr(prompt, "Physical state:") == NULL);

    /* With image and telemetry */
    const char *tele_str = "Physical state: distance to obstacle ahead is 25 cm.";
    voice_history_build_prompt("Teuvo", true, tele_str, prompt, sizeof(prompt));
    ASSERT(strstr(prompt, "the camera view and audio follow.") != NULL);
    ASSERT(strstr(prompt, tele_str) != NULL);
}

/* =========================================================================
 * Gemini contents array tests
 * ========================================================================= */

static void test_contents_without_history(void)
{
    voice_history_reset();

    cJSON *contents = cJSON_CreateArray();
    ASSERT(contents != NULL);

    const char *prompt = "Test prompt text";
    const char *audio_b64 = "dGVzdGF1ZGlv";  // "testaudio"
    ASSERT(voice_history_build_contents(contents, prompt, NULL, audio_b64, 1000));

    /* With no history, contents should contain exactly 1 turn */
    ASSERT(cJSON_GetArraySize(contents) == 1);

    cJSON *turn = cJSON_GetArrayItem(contents, 0);
    const cJSON *role = cJSON_GetObjectItem(turn, "role");
    ASSERT(role != NULL && strcmp(role->valuestring, "user") == 0);

    cJSON *parts = cJSON_GetObjectItem(turn, "parts");
    ASSERT(parts != NULL && cJSON_GetArraySize(parts) == 2);

    /* Part 0: prompt text */
    cJSON *p0 = cJSON_GetArrayItem(parts, 0);
    const cJSON *text = cJSON_GetObjectItem(p0, "text");
    ASSERT(text != NULL && strcmp(text->valuestring, prompt) == 0);

    /* Part 1: audio inlineData */
    cJSON *p1 = cJSON_GetArrayItem(parts, 1);
    cJSON *inline_data = cJSON_GetObjectItem(p1, "inlineData");
    ASSERT(inline_data != NULL);
    const cJSON *mime = cJSON_GetObjectItem(inline_data, "mimeType");
    const cJSON *data = cJSON_GetObjectItem(inline_data, "data");
    ASSERT(mime != NULL && strcmp(mime->valuestring, "audio/wav") == 0);
    ASSERT(data != NULL && strcmp(data->valuestring, audio_b64) == 0);

    cJSON_Delete(contents);
}

static void test_contents_with_history_and_image(void)
{
    voice_history_reset();
    voice_history_record("I see a chair.", 1000);
    voice_history_record("Obstacle is near.", 2000);

    cJSON *contents = cJSON_CreateArray();
    ASSERT(contents != NULL);

    const char *prompt = "What is ahead?";
    const char *jpeg_b64 = "dGVzdGpwZWc=";  // "testjpeg"
    const char *audio_b64 = "dGVzdGF1ZGlv";
    ASSERT(voice_history_build_contents(contents, prompt, jpeg_b64, audio_b64, 3000));

    /* 2 past turns * 2 items each + 1 current turn = 5 items in contents */
    ASSERT(cJSON_GetArraySize(contents) == 5);

    /* Turn 0: past user */
    cJSON *t0 = cJSON_GetArrayItem(contents, 0);
    ASSERT(strcmp(cJSON_GetObjectItem(t0, "role")->valuestring, "user") == 0);
    cJSON *t0_parts = cJSON_GetObjectItem(t0, "parts");
    ASSERT(cJSON_GetArraySize(t0_parts) == 1);
    ASSERT(strcmp(cJSON_GetObjectItem(cJSON_GetArrayItem(t0_parts, 0), "text")->valuestring,
                  "(User spoke to you)") == 0);

    /* Turn 1: past model reply 1 */
    cJSON *t1 = cJSON_GetArrayItem(contents, 1);
    ASSERT(strcmp(cJSON_GetObjectItem(t1, "role")->valuestring, "model") == 0);
    cJSON *t1_parts = cJSON_GetObjectItem(t1, "parts");
    ASSERT(cJSON_GetArraySize(t1_parts) == 1);
    ASSERT(strcmp(cJSON_GetObjectItem(cJSON_GetArrayItem(t1_parts, 0), "text")->valuestring,
                  "I see a chair.") == 0);

    /* Turn 2: past user */
    cJSON *t2 = cJSON_GetArrayItem(contents, 2);
    ASSERT(strcmp(cJSON_GetObjectItem(t2, "role")->valuestring, "user") == 0);

    /* Turn 3: past model reply 2 */
    cJSON *t3 = cJSON_GetArrayItem(contents, 3);
    ASSERT(strcmp(cJSON_GetObjectItem(t3, "role")->valuestring, "model") == 0);
    ASSERT(
        strcmp(cJSON_GetObjectItem(cJSON_GetArrayItem(cJSON_GetObjectItem(t3, "parts"), 0), "text")
                   ->valuestring,
               "Obstacle is near.") == 0);

    /* Turn 4: current user turn with text, image, audio */
    cJSON *t4 = cJSON_GetArrayItem(contents, 4);
    ASSERT(strcmp(cJSON_GetObjectItem(t4, "role")->valuestring, "user") == 0);
    cJSON *t4_parts = cJSON_GetObjectItem(t4, "parts");
    ASSERT(cJSON_GetArraySize(t4_parts) == 3);

    /* Part 0: text */
    ASSERT(strcmp(cJSON_GetObjectItem(cJSON_GetArrayItem(t4_parts, 0), "text")->valuestring,
                  prompt) == 0);

    /* Part 1: image inlineData */
    cJSON *img_inline = cJSON_GetObjectItem(cJSON_GetArrayItem(t4_parts, 1), "inlineData");
    ASSERT(img_inline != NULL);
    ASSERT(strcmp(cJSON_GetObjectItem(img_inline, "mimeType")->valuestring, "image/jpeg") == 0);
    ASSERT(strcmp(cJSON_GetObjectItem(img_inline, "data")->valuestring, jpeg_b64) == 0);

    /* Part 2: audio inlineData */
    cJSON *aud_inline = cJSON_GetObjectItem(cJSON_GetArrayItem(t4_parts, 2), "inlineData");
    ASSERT(aud_inline != NULL);
    ASSERT(strcmp(cJSON_GetObjectItem(aud_inline, "mimeType")->valuestring, "audio/wav") == 0);
    ASSERT(strcmp(cJSON_GetObjectItem(aud_inline, "data")->valuestring, audio_b64) == 0);

    cJSON_Delete(contents);
}

/* =========================================================================
 * Full request body JSON tests
 * ========================================================================= */

static void test_full_request_body_generation(void)
{
    voice_history_reset();
    voice_history_record("First answer.", 1000);

    reactive_telemetry_t tele = {.distance_cm = 50, .reflex_active = false, .sensor_failed = false};

    char *json_str = voice_history_build_request_body("Teuvo", "Speak like a 1950s gentleman.",
                                                      &tele, true, "b64img", "b64wav", 2000);
    ASSERT(json_str != NULL);

    cJSON *root = cJSON_Parse(json_str);
    ASSERT(root != NULL);

    /* Contents verification */
    cJSON *contents = cJSON_GetObjectItem(root, "contents");
    ASSERT(contents != NULL);
    /* 1 past turn (user + model) + 1 current turn = 3 items */
    ASSERT(cJSON_GetArraySize(contents) == 3);

    /* System instruction */
    cJSON *sys = cJSON_GetObjectItem(root, "systemInstruction");
    ASSERT(sys != NULL);
    cJSON *sys_parts = cJSON_GetObjectItem(sys, "parts");
    ASSERT(sys_parts != NULL);
    cJSON *sys_p0 = cJSON_GetArrayItem(sys_parts, 0);
    ASSERT(strcmp(cJSON_GetObjectItem(sys_p0, "text")->valuestring,
                  "Speak like a 1950s gentleman.") == 0);

    /* Generation config */
    cJSON *gen = cJSON_GetObjectItem(root, "generationConfig");
    ASSERT(gen != NULL);
    const cJSON *tokens = cJSON_GetObjectItem(gen, "maxOutputTokens");
    ASSERT(tokens != NULL && tokens->valueint == 2048);
    cJSON *thinking = cJSON_GetObjectItem(gen, "thinkingConfig");
    ASSERT(thinking != NULL);
    ASSERT(strcmp(cJSON_GetObjectItem(thinking, "thinkingLevel")->valuestring, "low") == 0);

    cJSON_Delete(root);
    free(json_str);
}

/* =========================================================================
 * Main entry point
 * ========================================================================= */

int main(void)
{
    printf("=== Starting voice_history tests ===\n");

    test_run("initial_state_empty", test_initial_state_empty);
    test_run("single_turn_recording", test_single_turn_recording);
    test_run("rolling_buffer_up_to_max", test_rolling_buffer_up_to_max);
    test_run("rolling_buffer_eviction", test_rolling_buffer_eviction);
    test_run("idle_timeout_expiration", test_idle_timeout_expiration);
    test_run("record_after_expiration", test_record_after_expiration);
    test_run("clock_wrap_around", test_clock_wrap_around);
    test_run("manual_reset", test_manual_reset);
    test_run("ignore_tag_detection", test_ignore_tag_detection);
    test_run("record_with_ignore_sentinel", test_record_with_ignore_sentinel);
    test_run("active_conversation_window", test_active_conversation_window);
    test_run("conversation_window_wrap_around", test_conversation_window_wrap_around);
    test_run("conversation_window_reset", test_conversation_window_reset);
    test_run("telemetry_formatting", test_telemetry_formatting);
    test_run("prompt_building", test_prompt_building);
    test_run("contents_without_history", test_contents_without_history);
    test_run("contents_with_history_and_image", test_contents_with_history_and_image);
    test_run("full_request_body_generation", test_full_request_body_generation);

    printf("=== %d/%d tests passed ===\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
