/**
 * @file test_live_proto.c
 * @brief Host tests for the Gemini Live message builders and parser (ADR-032).
 *
 * The client messages are checked against the shapes google-genai 2.34 puts on
 * the wire, captured offline by patching its WebSocket connect (the session
 * tools/live-probe measured on 2026-11-07). A field renamed here would surface
 * on the robot only as a close code from a remote server, so it is pinned here.
 */

#include "live_proto.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

static int test_count = 0;

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
    printf("     PASS\n");
}

static cJSON *path(cJSON *root, const char *const *keys)
{
    cJSON *node = root;
    for (; *keys && node; keys++) {
        node = cJSON_GetObjectItemCaseSensitive(node, *keys);
    }
    return node;
}

#define PATH(root, ...) path((root), (const char *const[]){__VA_ARGS__, NULL})

/* ---------------------------------------------------------------- setup */

static void test_setup_has_the_sdk_shape(void)
{
    char *json = live_proto_build_setup(LIVE_PROTO_MODEL, "Schedar", "SYS");
    ASSERT(json != NULL);
    cJSON *root = cJSON_Parse(json);
    ASSERT(root != NULL);

    ASSERT(strcmp(cJSON_GetStringValue(PATH(root, "setup", "model")), "models/gemini-3.8-live") ==
           0);
    cJSON *mods = PATH(root, "setup", "generationConfig", "responseModalities");
    ASSERT(cJSON_IsArray(mods) && cJSON_GetArraySize(mods) == 1);
    ASSERT(strcmp(cJSON_GetStringValue(cJSON_GetArrayItem(mods, 0)), "AUDIO") == 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(root, "setup", "generationConfig", "speechConfig",
                                            "voice_config", "prebuilt_voice_config", "voice_name")),
                  "Schedar") == 0);

    cJSON *parts = PATH(root, "setup", "systemInstruction", "parts");
    ASSERT(cJSON_IsArray(parts));
    ASSERT(strcmp(cJSON_GetStringValue(PATH(cJSON_GetArrayItem(parts, 0), "text")), "SYS") == 0);

    ASSERT(cJSON_IsObject(PATH(root, "setup", "inputAudioTranscription")));
    ASSERT(cJSON_IsObject(PATH(root, "setup", "outputAudioTranscription")));
    /* Without compression an audio session ends at 15 min, audio+video at 2. */
    ASSERT(cJSON_IsObject(PATH(root, "setup", "contextWindowCompression", "sliding_window")));

    cJSON *decls =
        PATH(cJSON_GetArrayItem(PATH(root, "setup", "tools"), 0), "functionDeclarations");
    ASSERT(cJSON_IsArray(decls) && cJSON_GetArraySize(decls) == 1);
    cJSON *wait = cJSON_GetArrayItem(decls, 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(wait, "name")), "wait") == 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(wait, "behavior")), "NON_BLOCKING") == 0);
    ASSERT(cJSON_GetStringValue(PATH(wait, "description"))[0] != '\0');

    /* Affective dialog closed the session with 1007 on 2026-11-16 (#1209). */
    ASSERT(strstr(json, "ffective") == NULL);

    cJSON_Delete(root);
    free(json);
}

static void test_setup_without_voice_leaves_speech_config_out(void)
{
    char *json = live_proto_build_setup(LIVE_PROTO_MODEL, "", "SYS");
    ASSERT(json != NULL);
    cJSON *root = cJSON_Parse(json);
    ASSERT(PATH(root, "setup", "generationConfig", "speechConfig") == NULL);
    cJSON_Delete(root);
    free(json);
    ASSERT(live_proto_build_setup(NULL, "x", "SYS") == NULL);
    ASSERT(live_proto_build_setup(LIVE_PROTO_MODEL, "x", NULL) == NULL);
}

static void test_setup_escapes_the_instruction(void)
{
    const char *sys = "Rivi \"lainaus\"\nja kenoviiva \\ äö";
    char *json = live_proto_build_setup(LIVE_PROTO_MODEL, "Kore", sys);
    cJSON *root = cJSON_Parse(json);
    ASSERT(root != NULL);
    cJSON *parts = PATH(root, "setup", "systemInstruction", "parts");
    ASSERT(strcmp(cJSON_GetStringValue(PATH(cJSON_GetArrayItem(parts, 0), "text")), sys) == 0);
    cJSON_Delete(root);
    free(json);
}

/* ---------------------------------------------------------- instruction */

static void test_finnish_instruction_carries_persona_and_wait(void)
{
    char *s = live_proto_build_instruction("Teuvo", "BRIEF.", "STYLE", "fi-FI");
    ASSERT(s != NULL);
    /* The probe's wording, which kept the register on 2026-11-07. */
    ASSERT(strstr(s, "Olet Teuvo, puhuva robottiauto. BRIEF.") == s);
    ASSERT(strstr(s, "Ääntämys ja esitystapa: STYLE.") != NULL);
    ASSERT(strstr(s, "Vastaa aina suomeksi, ääneen ja lyhyesti.") != NULL);
    ASSERT(strstr(s, "`wait`") != NULL);
    free(s);
}

static void test_english_instruction_and_missing_fields(void)
{
    char *s = live_proto_build_instruction("Robocar", NULL, "", "en-US");
    ASSERT(s != NULL);
    ASSERT(strstr(s, "You are Robocar, a talking robot car.") == s);
    ASSERT(strstr(s, "Delivery") == NULL); /* no empty style line */
    ASSERT(strstr(s, "`wait`") != NULL);
    ASSERT(strstr(s, "(null)") == NULL);
    free(s);
    ASSERT(live_proto_build_instruction(NULL, "b", "s", "fi-FI") == NULL);
}

/* ---------------------------------------------------------------- audio */

static void test_audio_message_round_trips(void)
{
    const int16_t pcm[2] = {0x0201, 0x0403}; /* bytes 01 02 03 04, little-endian */
    char buf[256];
    const size_t need = live_proto_audio_msg_size(sizeof(pcm));
    ASSERT(need <= sizeof(buf));
    const size_t n = live_proto_build_audio(pcm, 2, buf, sizeof(buf));
    ASSERT(n == need - 1);
    ASSERT(strlen(buf) == n);

    cJSON *root = cJSON_Parse(buf);
    ASSERT(root != NULL);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(root, "realtime_input", "audio", "mime_type")),
                  "audio/pcm;rate=16000") == 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(root, "realtime_input", "audio", "data")),
                  "AQIDBA==") == 0);
    cJSON_Delete(root);
}

static void test_audio_message_refuses_a_short_buffer(void)
{
    int16_t pcm[1600] = {0};
    const size_t need = live_proto_audio_msg_size(sizeof(pcm));
    char *buf = malloc(need);
    ASSERT(live_proto_build_audio(pcm, 1600, buf, need - 1) == 0);
    ASSERT(live_proto_build_audio(pcm, 1600, buf, need) == need - 1);
    ASSERT(live_proto_build_audio(NULL, 1600, buf, need) == 0);
    ASSERT(live_proto_build_audio(pcm, 0, buf, need) == 0);
    /* 100 ms: 3.2 kB of PCM is about 4.3 kB on the wire (ADR-032 §1). */
    ASSERT(need > 4266 && need < 4400);
    free(buf);
}

static void test_stream_end_and_tool_response(void)
{
    cJSON *end = cJSON_Parse(live_proto_audio_stream_end());
    ASSERT(cJSON_IsTrue(PATH(end, "realtime_input", "audioStreamEnd")));
    cJSON_Delete(end);

    char *json = live_proto_build_tool_response("fc-1", "wait");
    cJSON *root = cJSON_Parse(json);
    cJSON *fr = cJSON_GetArrayItem(PATH(root, "tool_response", "functionResponses"), 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(fr, "id")), "fc-1") == 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(fr, "name")), "wait") == 0);
    ASSERT(strcmp(cJSON_GetStringValue(PATH(fr, "scheduling")), "SILENT") == 0);
    ASSERT(cJSON_IsObject(PATH(fr, "response")));
    cJSON_Delete(root);
    free(json);
    ASSERT(live_proto_build_tool_response(NULL, "wait") == NULL);
}

/* --------------------------------------------------------------- parser */

typedef struct {
    uint8_t buf[64];
    size_t len;
    bool fail;
} sink_acc_t;

static bool sink(const uint8_t *data, size_t len, void *ctx)
{
    sink_acc_t *a = ctx;
    if (a->fail) {
        return false;
    }
    for (size_t i = 0; i < len && a->len < sizeof(a->buf); i++) {
        a->buf[a->len++] = data[i];
    }
    return true;
}

static int parse(const char *json, live_server_msg_t *m, sink_acc_t *a)
{
    return live_proto_parse_server(json, strlen(json), m, a ? sink : NULL, a);
}

static void test_parse_setup_complete(void)
{
    live_server_msg_t m;
    ASSERT(parse("{\"setupComplete\":{}}", &m, NULL) == 0);
    ASSERT(m.setup_complete);
    ASSERT(!m.turn_complete && m.audio_parts == 0);
}

static void test_parse_audio_parts_in_order(void)
{
    const char *json =
        "{\"serverContent\":{\"modelTurn\":{\"parts\":["
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=24000\",\"data\":\"AQI=\"}},"
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=24000\",\"data\":\"AwQ=\"}}]}}}";
    live_server_msg_t m;
    sink_acc_t a = {0};
    ASSERT(parse(json, &m, &a) == 0);
    ASSERT(m.audio_parts == 2);
    ASSERT(m.audio_bytes == 4);
    ASSERT(a.len == 4 && a.buf[0] == 1 && a.buf[1] == 2 && a.buf[2] == 3 && a.buf[3] == 4);
    ASSERT(!m.turn_complete);
}

static void test_parse_skips_audio_at_another_rate(void)
{
    const char *json =
        "{\"serverContent\":{\"modelTurn\":{\"parts\":["
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=16000\",\"data\":\"AQI=\"}},"
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=24000\",\"data\":\"AwQ=\"}}]}}}";
    live_server_msg_t m;
    sink_acc_t a = {0};
    ASSERT(parse(json, &m, &a) == 0);
    ASSERT(m.audio_parts == 1 && m.audio_parts_skipped == 1);
    ASSERT(a.len == 2 && a.buf[0] == 3 && a.buf[1] == 4);
}

static void test_parse_counts_without_a_sink(void)
{
    const char *json =
        "{\"serverContent\":{\"modelTurn\":{\"parts\":["
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=24000\",\"data\":\"AQIDBA==\"}}]}}}";
    live_server_msg_t m;
    ASSERT(parse(json, &m, NULL) == 0);
    ASSERT(m.audio_parts == 1 && m.audio_bytes == 4);
}

static void test_parse_reports_a_stalled_sink(void)
{
    const char *json =
        "{\"serverContent\":{\"modelTurn\":{\"parts\":["
        "{\"inlineData\":{\"mimeType\":\"audio/pcm;rate=24000\",\"data\":\"AQIDBA==\"}}]}}}";
    live_server_msg_t m;
    sink_acc_t a = {.fail = true};
    ASSERT(parse(json, &m, &a) == 0);
    ASSERT(m.sink_aborted);
}

static void test_parse_turn_flags_and_transcripts(void)
{
    live_server_msg_t m;
    ASSERT(parse("{\"serverContent\":{\"turnComplete\":true,\"generationComplete\":true,"
                 "\"outputTranscription\":{\"text\":\"Hyvää päivää\"}}}",
                 &m, NULL) == 0);
    ASSERT(m.turn_complete && m.generation_complete && !m.interrupted);
    ASSERT(strcmp(m.output_text, "Hyvää päivää") == 0);
    ASSERT(m.input_text[0] == '\0');

    ASSERT(parse("{\"serverContent\":{\"interrupted\":true,"
                 "\"inputTranscription\":{\"text\":\"Hei\"}}}",
                 &m, NULL) == 0);
    ASSERT(m.interrupted && !m.turn_complete);
    ASSERT(strcmp(m.input_text, "Hei") == 0);
}

static void test_parse_truncates_long_text_on_a_utf8_boundary(void)
{
    /* 200 two-byte characters: 400 bytes into a 256-byte field. */
    char json[600];
    size_t n = (size_t)snprintf(json, sizeof(json),
                                "{\"serverContent\":{\"outputTranscription\""
                                ":{\"text\":\"");
    for (int i = 0; i < 200; i++) {
        json[n++] = (char)0xC3;
        json[n++] = (char)0xA4; /* ä */
    }
    n += (size_t)snprintf(json + n, sizeof(json) - n, "\"}}}");
    live_server_msg_t m;
    ASSERT(live_proto_parse_server(json, n, &m, NULL, NULL) == 0);
    const size_t len = strlen(m.output_text);
    ASSERT(len > 0 && len < LIVE_PROTO_TEXT_MAX);
    ASSERT(len % 2 == 0); /* never half a character */
}

static void test_parse_tool_call(void)
{
    live_server_msg_t m;
    ASSERT(parse("{\"toolCall\":{\"functionCalls\":[{\"id\":\"fc-9\",\"name\":\"wait\","
                 "\"args\":{}}]}}",
                 &m, NULL) == 0);
    ASSERT(m.tool_call);
    ASSERT(strcmp(m.tool_id, "fc-9") == 0 && strcmp(m.tool_name, "wait") == 0);
}

static void test_parse_go_away_and_usage(void)
{
    live_server_msg_t m;
    ASSERT(parse("{\"goAway\":{\"timeLeft\":\"9.5s\"},\"usageMetadata\":{\"promptTokenCount\":885,"
                 "\"responseTokenCount\":238,\"totalTokenCount\":1123}}",
                 &m, NULL) == 0);
    ASSERT(m.go_away && m.go_away_ms == 9500);
    ASSERT(m.has_usage);
    ASSERT(m.prompt_tokens == 885 && m.response_tokens == 238 && m.total_tokens == 1123);
}

static void test_parse_rejects_non_objects(void)
{
    live_server_msg_t m;
    ASSERT(parse("not json", &m, NULL) == -1);
    ASSERT(parse("[1,2]", &m, NULL) == -1);
    ASSERT(live_proto_parse_server(NULL, 0, &m, NULL, NULL) == -1);
    /* An unknown message type is not an error: the server adds fields. */
    ASSERT(parse("{\"sessionResumptionUpdate\":{}}", &m, NULL) == 0);
}

static void test_parse_length_bounds_the_input(void)
{
    /* The receive buffer is not terminated where the message ends. */
    const char *buf = "{\"setupComplete\":{}}GARBAGE";
    live_server_msg_t m;
    ASSERT(live_proto_parse_server(buf, 20, &m, NULL, NULL) == 0);
    ASSERT(m.setup_complete);
}

static void test_duration(void)
{
    uint32_t ms = 1;
    ASSERT(live_proto_parse_duration_ms("10s", &ms) && ms == 10000);
    ASSERT(live_proto_parse_duration_ms("0.250s", &ms) && ms == 250);
    ASSERT(live_proto_parse_duration_ms("9.5s", &ms) && ms == 9500);
    ASSERT(!live_proto_parse_duration_ms("10", &ms) && ms == 0);
    ASSERT(!live_proto_parse_duration_ms("", &ms));
    ASSERT(!live_proto_parse_duration_ms("x1s", &ms));
    ASSERT(!live_proto_parse_duration_ms(NULL, &ms));
}

int main(void)
{
    test_run("setup has the SDK shape", test_setup_has_the_sdk_shape);
    test_run("setup without voice", test_setup_without_voice_leaves_speech_config_out);
    test_run("setup escapes the instruction", test_setup_escapes_the_instruction);
    test_run("Finnish instruction", test_finnish_instruction_carries_persona_and_wait);
    test_run("English instruction", test_english_instruction_and_missing_fields);
    test_run("audio message round-trips", test_audio_message_round_trips);
    test_run("audio message buffer bound", test_audio_message_refuses_a_short_buffer);
    test_run("stream end and tool response", test_stream_end_and_tool_response);
    test_run("parse setupComplete", test_parse_setup_complete);
    test_run("parse audio parts in order", test_parse_audio_parts_in_order);
    test_run("parse skips other rates", test_parse_skips_audio_at_another_rate);
    test_run("parse counts without sink", test_parse_counts_without_a_sink);
    test_run("parse reports stalled sink", test_parse_reports_a_stalled_sink);
    test_run("parse turn flags + transcripts", test_parse_turn_flags_and_transcripts);
    test_run("parse truncates on UTF-8 boundary",
             test_parse_truncates_long_text_on_a_utf8_boundary);
    test_run("parse tool call", test_parse_tool_call);
    test_run("parse goAway + usage", test_parse_go_away_and_usage);
    test_run("parse rejects non-objects", test_parse_rejects_non_objects);
    test_run("parse honours length", test_parse_length_bounds_the_input);
    test_run("duration", test_duration);
    printf("\nAll %d live_proto tests passed\n", test_count);
    return 0;
}
