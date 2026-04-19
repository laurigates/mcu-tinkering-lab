/**
 * @file test_nfc_story_sounds.c
 * @brief Host-based unit tests for the NFC story sounds parser + dispatcher.
 */

#include <string.h>

#include "nfc_story_sounds.h"
#include "unity_compat.h"

/* ------------------------------------------------------------------ */
/* Parser — happy path                                                 */
/* ------------------------------------------------------------------ */

static void test_parse_single_tone(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"tone\",\"param\":440}]}";
    story_sequence_t out;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(1, out.count);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_TONE, out.steps[0].kind);
    TEST_ASSERT_EQUAL(440, out.steps[0].param);
}

static void test_parse_all_kinds(void)
{
    const char *j = "{\"sequence\":["
                    "{\"kind\":\"tone\",\"param\":440},"
                    "{\"kind\":\"wait\",\"param\":120},"
                    "{\"kind\":\"clip\",\"param\":3}"
                    "]}";
    story_sequence_t out;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(3, out.count);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_TONE, out.steps[0].kind);
    TEST_ASSERT_EQUAL(440, out.steps[0].param);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_WAIT, out.steps[1].kind);
    TEST_ASSERT_EQUAL(120, out.steps[1].param);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_CLIP, out.steps[2].kind);
    TEST_ASSERT_EQUAL(3, out.steps[2].param);
}

static void test_parse_whitespace_and_negative_param(void)
{
    const char *j = "{ \"sequence\" : [ {\n"
                    "  \"kind\" : \"wait\" , \"param\" : -10\n"
                    "} ] }";
    story_sequence_t out;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(1, out.count);
    TEST_ASSERT_EQUAL(-10, out.steps[0].param);
}

static void test_parse_param_before_kind(void)
{
    const char *j = "{\"sequence\":[{\"param\":880,\"kind\":\"tone\"}]}";
    story_sequence_t out;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(1, out.count);
    TEST_ASSERT_EQUAL(880, out.steps[0].param);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_TONE, out.steps[0].kind);
}

/* ------------------------------------------------------------------ */
/* Parser — failure modes                                              */
/* ------------------------------------------------------------------ */

static void test_parse_rejects_null_inputs(void)
{
    story_sequence_t out;
    memset(&out, 0xCC, sizeof(out));
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(NULL, 0, &out));
    TEST_ASSERT_EQUAL(0, out.count);
    TEST_ASSERT_FALSE(nfc_story_sounds_parse("anything", 0, &out));
    TEST_ASSERT_FALSE(nfc_story_sounds_parse("{", 1, NULL));
}

static void test_parse_rejects_malformed_brace(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"tone\",\"param\":440}";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(0, out.count);
}

static void test_parse_rejects_truncated(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"ton";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(0, out.count);
}

static void test_parse_rejects_wrong_root_key(void)
{
    const char *j = "{\"seq\":[{\"kind\":\"tone\",\"param\":440}]}";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(0, out.count);
}

static void test_parse_rejects_unknown_kind(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"screech\",\"param\":1}]}";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
}

static void test_parse_rejects_missing_param(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"tone\"}]}";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
}

static void test_parse_rejects_empty_sequence(void)
{
    const char *j = "{\"sequence\":[]}";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
    TEST_ASSERT_EQUAL(0, out.count);
}

static void test_parse_rejects_too_many_steps(void)
{
    /* Build a sequence with MAX+1 steps. */
    char j[1024];
    size_t pos = 0;
    pos += (size_t)snprintf(j + pos, sizeof(j) - pos, "{\"sequence\":[");
    for (int i = 0; i < NFC_STORY_SOUNDS_MAX_STEPS + 1; i++) {
        pos += (size_t)snprintf(j + pos, sizeof(j) - pos, "%s{\"kind\":\"wait\",\"param\":%d}",
                                i == 0 ? "" : ",", i);
    }
    pos += (size_t)snprintf(j + pos, sizeof(j) - pos, "]}");

    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, pos, &out));
    TEST_ASSERT_EQUAL(0, out.count);
}

static void test_parse_rejects_trailing_garbage(void)
{
    const char *j = "{\"sequence\":[{\"kind\":\"tone\",\"param\":440}]}XYZ";
    story_sequence_t out;
    TEST_ASSERT_FALSE(nfc_story_sounds_parse(j, strlen(j), &out));
}

/* ------------------------------------------------------------------ */
/* Execute                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    int call_count;
    uint8_t kinds[NFC_STORY_SOUNDS_MAX_STEPS];
    int32_t params[NFC_STORY_SOUNDS_MAX_STEPS];
    int abort_after; /**< -1 = never abort */
} exec_ctx_t;

static int mock_dispatch(uint8_t kind, int32_t param, void *user_ctx)
{
    exec_ctx_t *c = (exec_ctx_t *)user_ctx;
    c->kinds[c->call_count] = kind;
    c->params[c->call_count] = param;
    c->call_count++;
    if (c->abort_after >= 0 && c->call_count > c->abort_after) {
        return 42;
    }
    return 0;
}

static void test_execute_invokes_each_step_in_order(void)
{
    const char *j = "{\"sequence\":["
                    "{\"kind\":\"tone\",\"param\":100},"
                    "{\"kind\":\"wait\",\"param\":200},"
                    "{\"kind\":\"clip\",\"param\":300}"
                    "]}";
    story_sequence_t seq;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &seq));

    exec_ctx_t ctx = {0};
    ctx.abort_after = -1;
    int rc = nfc_story_sounds_execute(&seq, mock_dispatch, &ctx);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(3, ctx.call_count);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_TONE, ctx.kinds[0]);
    TEST_ASSERT_EQUAL(100, ctx.params[0]);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_WAIT, ctx.kinds[1]);
    TEST_ASSERT_EQUAL(200, ctx.params[1]);
    TEST_ASSERT_EQUAL((int)NFC_STORY_KIND_CLIP, ctx.kinds[2]);
    TEST_ASSERT_EQUAL(300, ctx.params[2]);
}

static void test_execute_stops_on_dispatch_error(void)
{
    const char *j = "{\"sequence\":["
                    "{\"kind\":\"tone\",\"param\":1},"
                    "{\"kind\":\"tone\",\"param\":2},"
                    "{\"kind\":\"tone\",\"param\":3}"
                    "]}";
    story_sequence_t seq;
    TEST_ASSERT_TRUE(nfc_story_sounds_parse(j, strlen(j), &seq));

    exec_ctx_t ctx = {0};
    ctx.abort_after = 1; /* dispatcher rejects after the first call */
    int rc = nfc_story_sounds_execute(&seq, mock_dispatch, &ctx);
    TEST_ASSERT_EQUAL(42, rc);
    TEST_ASSERT_EQUAL(2, ctx.call_count);
}

static void test_execute_rejects_null_args(void)
{
    exec_ctx_t ctx = {0};
    ctx.abort_after = -1;
    TEST_ASSERT_EQUAL(-1, nfc_story_sounds_execute(NULL, mock_dispatch, &ctx));
    story_sequence_t seq = {0};
    TEST_ASSERT_EQUAL(-1, nfc_story_sounds_execute(&seq, NULL, &ctx));
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_parse_single_tone);
    RUN_TEST(test_parse_all_kinds);
    RUN_TEST(test_parse_whitespace_and_negative_param);
    RUN_TEST(test_parse_param_before_kind);
    RUN_TEST(test_parse_rejects_null_inputs);
    RUN_TEST(test_parse_rejects_malformed_brace);
    RUN_TEST(test_parse_rejects_truncated);
    RUN_TEST(test_parse_rejects_wrong_root_key);
    RUN_TEST(test_parse_rejects_unknown_kind);
    RUN_TEST(test_parse_rejects_missing_param);
    RUN_TEST(test_parse_rejects_empty_sequence);
    RUN_TEST(test_parse_rejects_too_many_steps);
    RUN_TEST(test_parse_rejects_trailing_garbage);
    RUN_TEST(test_execute_invokes_each_step_in_order);
    RUN_TEST(test_execute_stops_on_dispatch_error);
    RUN_TEST(test_execute_rejects_null_args);
    return UNITY_END();
}
