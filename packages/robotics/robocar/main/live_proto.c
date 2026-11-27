/**
 * @file live_proto.c
 * @brief Gemini Live API messages. See live_proto.h for the shapes and why.
 */

#include "live_proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

static const char k_wait_description[] =
    "Call this instead of speaking when no reaction is needed. Not every remark needs an "
    "answer: there is no need to fill a silence or to chime in on everything you hear. Say "
    "nothing after it.";

/* The persona lines are the probe's (live_probe.py extract_persona()). */
static const char k_fi_head[] = "Olet %s, puhuva robottiauto.";
static const char k_fi_style[] = "Ääntämys ja esitystapa: %s.";
static const char k_fi_tail[] =
    "Vastaa aina suomeksi, ääneen ja lyhyesti.\n"
    "Kaikkeen ei tarvitse reagoida. Älä täytä hiljaisuutta äläkä puutu jokaiseen "
    "lausahdukseen; kun vastausta ei tarvita, kutsu `wait` äläkä sano mitään.";
static const char k_en_head[] = "You are %s, a talking robot car.";
static const char k_en_style[] = "Delivery: %s.";
static const char k_en_tail[] =
    "Always answer aloud and briefly.\n"
    "Not everything needs a reaction. Don't fill silences or chime in on every remark; when "
    "no answer is needed, call `wait` and say nothing.";

static const char k_stream_end[] = "{\"realtime_input\":{\"audioStreamEnd\":true}}";

/* The fixed parts of an audio message around the base64 payload. */
static const char k_audio_head[] =
    "{\"realtime_input\":{\"audio\":{\"mime_type\":\"audio/pcm;rate=16000\",\"data\":\"";
static const char k_audio_tail[] = "\"}}}";

static bool nonempty(const char *s)
{
    return s && s[0] != '\0';
}

char *live_proto_build_instruction(const char *name, const char *text_brief, const char *tts_style,
                                   const char *language_code)
{
    if (!name) {
        return NULL;
    }
    const bool fi = language_code && strncmp(language_code, "fi", 2) == 0;
    const char *head = fi ? k_fi_head : k_en_head;
    const char *style = fi ? k_fi_style : k_en_style;
    const char *tail = fi ? k_fi_tail : k_en_tail;

    const size_t cap =
        strlen(head) + strlen(name) + 1 + (nonempty(text_brief) ? strlen(text_brief) + 1 : 0) +
        (nonempty(tts_style) ? strlen(style) + strlen(tts_style) + 1 : 0) + strlen(tail) + 1;
    char *out = malloc(cap);
    if (!out) {
        return NULL;
    }
    int n = snprintf(out, cap, head, name);
    if (nonempty(text_brief)) {
        n += snprintf(out + n, cap - (size_t)n, " %s", text_brief);
    }
    n += snprintf(out + n, cap - (size_t)n, "\n");
    if (nonempty(tts_style)) {
        n += snprintf(out + n, cap - (size_t)n, style, tts_style);
        n += snprintf(out + n, cap - (size_t)n, "\n");
    }
    snprintf(out + n, cap - (size_t)n, "%s", tail);
    return out;
}

char *live_proto_build_setup(const char *model, const char *voice, const char *instruction)
{
    if (!model || !instruction) {
        return NULL;
    }
    char model_path[96];
    if (snprintf(model_path, sizeof(model_path), "models/%s", model) >= (int)sizeof(model_path)) {
        return NULL;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON *setup = cJSON_AddObjectToObject(root, "setup");
    cJSON_AddStringToObject(setup, "model", model_path);

    cJSON *gen = cJSON_AddObjectToObject(setup, "generationConfig");
    cJSON *mods = cJSON_AddArrayToObject(gen, "responseModalities");
    cJSON_AddItemToArray(mods, cJSON_CreateString("AUDIO"));
    if (nonempty(voice)) {
        cJSON *speech = cJSON_AddObjectToObject(gen, "speechConfig");
        cJSON *vc = cJSON_AddObjectToObject(speech, "voice_config");
        cJSON *pvc = cJSON_AddObjectToObject(vc, "prebuilt_voice_config");
        cJSON_AddStringToObject(pvc, "voice_name", voice);
    }

    cJSON *sys = cJSON_AddObjectToObject(setup, "systemInstruction");
    cJSON *parts = cJSON_AddArrayToObject(sys, "parts");
    cJSON *part = cJSON_CreateObject();
    cJSON_AddStringToObject(part, "text", instruction);
    cJSON_AddItemToArray(parts, part);
    cJSON_AddStringToObject(sys, "role", "user");

    cJSON *tools = cJSON_AddArrayToObject(setup, "tools");
    cJSON *tool = cJSON_CreateObject();
    cJSON *decls = cJSON_AddArrayToObject(tool, "functionDeclarations");
    cJSON *wait = cJSON_CreateObject();
    cJSON_AddStringToObject(wait, "description", k_wait_description);
    cJSON_AddStringToObject(wait, "name", LIVE_PROTO_WAIT_TOOL);
    cJSON_AddStringToObject(wait, "behavior", "NON_BLOCKING");
    cJSON_AddItemToArray(decls, wait);
    cJSON_AddItemToArray(tools, tool);

    cJSON_AddObjectToObject(setup, "inputAudioTranscription");
    cJSON_AddObjectToObject(setup, "outputAudioTranscription");
    cJSON *cwc = cJSON_AddObjectToObject(setup, "contextWindowCompression");
    cJSON_AddObjectToObject(cwc, "sliding_window");

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

size_t live_proto_audio_msg_size(size_t pcm_bytes)
{
    /* base64_encode_length() counts the terminator, which this one shares. */
    return (sizeof(k_audio_head) - 1) + base64_encode_length(pcm_bytes) +
           (sizeof(k_audio_tail) - 1);
}

size_t live_proto_build_audio(const int16_t *pcm, size_t samples, char *out, size_t cap)
{
    if (!pcm || !out || samples == 0) {
        return 0;
    }
    const size_t pcm_bytes = samples * sizeof(int16_t);
    if (cap < live_proto_audio_msg_size(pcm_bytes)) {
        return 0;
    }
    const size_t head = sizeof(k_audio_head) - 1;
    memcpy(out, k_audio_head, head);
    size_t b64_len = cap - head;
    if (base64_encode((const uint8_t *)pcm, pcm_bytes, out + head, &b64_len) != 0) {
        return 0;
    }
    memcpy(out + head + b64_len, k_audio_tail, sizeof(k_audio_tail)); /* with terminator */
    return head + b64_len + sizeof(k_audio_tail) - 1;
}

const char *live_proto_audio_stream_end(void)
{
    return k_stream_end;
}

char *live_proto_build_tool_response(const char *id, const char *name)
{
    if (!id || !name) {
        return NULL;
    }
    cJSON *root = cJSON_CreateObject();
    cJSON *tr = cJSON_AddObjectToObject(root, "tool_response");
    cJSON *list = cJSON_AddArrayToObject(tr, "functionResponses");
    cJSON *fr = cJSON_CreateObject();
    cJSON_AddStringToObject(fr, "scheduling", "SILENT");
    cJSON_AddStringToObject(fr, "id", id);
    cJSON_AddStringToObject(fr, "name", name);
    cJSON *resp = cJSON_AddObjectToObject(fr, "response");
    cJSON_AddStringToObject(resp, "result", "ok");
    cJSON_AddItemToArray(list, fr);
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

/* ------------------------------------------------------------------ parse */

/** Copy @p src into @p dst, cutting at a UTF-8 character boundary. */
static void copy_text(char *dst, size_t cap, const char *src)
{
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t n = strlen(src);
    if (n >= cap) {
        n = cap - 1;
        /* Back off continuation bytes (10xxxxxx) so no character is split. */
        while (n > 0 && ((unsigned char)src[n] & 0xC0) == 0x80) {
            n--;
        }
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static uint32_t uint_field(const cJSON *obj, const char *key)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
    return (cJSON_IsNumber(v) && v->valuedouble > 0) ? (uint32_t)v->valuedouble : 0;
}

static bool true_field(const cJSON *obj, const char *key)
{
    return cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(obj, key));
}

/** Sink adapter that counts bytes and notices an abort. */
typedef struct {
    base64_sink_fn sink;
    void *ctx;
    size_t bytes;
} count_ctx_t;

static bool count_sink(const uint8_t *data, size_t len, void *ctx)
{
    count_ctx_t *c = ctx;
    c->bytes += len;
    return c->sink ? c->sink(data, len, c->ctx) : true;
}

/* Decode one part's base64 by feeding the stream decoder a `"data":"…"`
 * framing of it: the decoder the TTS path already trusts, unchanged. */
static int decode_part(const char *b64, count_ctx_t *cc)
{
    static const char k_open[] = "\"data\":\"";
    base64_stream_t st;
    base64_stream_init(&st);
    if (base64_stream_feed(&st, k_open, sizeof(k_open) - 1, count_sink, cc) != 0) {
        return -1;
    }
    if (base64_stream_feed(&st, b64, strlen(b64), count_sink, cc) != 0) {
        return -1;
    }
    return base64_stream_feed(&st, "\"", 1, count_sink, cc);
}

static void parse_server_content(const cJSON *sc, live_server_msg_t *out, base64_sink_fn sink,
                                 void *ctx)
{
    out->turn_complete = true_field(sc, "turnComplete");
    out->interrupted = true_field(sc, "interrupted");
    out->generation_complete = true_field(sc, "generationComplete");

    const cJSON *it = cJSON_GetObjectItemCaseSensitive(sc, "inputTranscription");
    copy_text(out->input_text, sizeof(out->input_text),
              cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(it, "text")));
    const cJSON *ot = cJSON_GetObjectItemCaseSensitive(sc, "outputTranscription");
    copy_text(out->output_text, sizeof(out->output_text),
              cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(ot, "text")));

    const cJSON *turn = cJSON_GetObjectItemCaseSensitive(sc, "modelTurn");
    const cJSON *parts = cJSON_GetObjectItemCaseSensitive(turn, "parts");
    const cJSON *part = NULL;
    cJSON_ArrayForEach(part, parts)
    {
        const cJSON *inl = cJSON_GetObjectItemCaseSensitive(part, "inlineData");
        if (!inl) {
            continue;
        }
        const char *mime = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(inl, "mimeType"));
        const char *data = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(inl, "data"));
        if (!mime || !data || strcmp(mime, LIVE_PROTO_OUTPUT_MIME) != 0) {
            out->audio_parts_skipped++;
            continue;
        }
        out->audio_parts++;
        if (out->sink_aborted) {
            continue; /* the ring stalled: count, decode nothing more */
        }
        count_ctx_t cc = {.sink = sink, .ctx = ctx, .bytes = 0};
        if (decode_part(data, &cc) != 0) {
            out->sink_aborted = true;
        }
        out->audio_bytes += cc.bytes;
    }
}

int live_proto_parse_server(const char *json, size_t len, live_server_msg_t *out,
                            base64_sink_fn sink, void *ctx)
{
    if (!out) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    if (!json || len == 0) {
        return -1;
    }
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return -1;
    }

    out->setup_complete = cJSON_IsObject(cJSON_GetObjectItemCaseSensitive(root, "setupComplete"));

    const cJSON *sc = cJSON_GetObjectItemCaseSensitive(root, "serverContent");
    if (cJSON_IsObject(sc)) {
        parse_server_content(sc, out, sink, ctx);
    }

    const cJSON *tc = cJSON_GetObjectItemCaseSensitive(root, "toolCall");
    const cJSON *fc = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(tc, "functionCalls"), 0);
    if (cJSON_IsObject(fc)) {
        out->tool_call = true;
        copy_text(out->tool_id, sizeof(out->tool_id),
                  cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(fc, "id")));
        copy_text(out->tool_name, sizeof(out->tool_name),
                  cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(fc, "name")));
    }

    const cJSON *ga = cJSON_GetObjectItemCaseSensitive(root, "goAway");
    if (cJSON_IsObject(ga)) {
        out->go_away = true;
        live_proto_parse_duration_ms(
            cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(ga, "timeLeft")),
            &out->go_away_ms);
    }

    const cJSON *um = cJSON_GetObjectItemCaseSensitive(root, "usageMetadata");
    if (cJSON_IsObject(um)) {
        out->has_usage = true;
        out->prompt_tokens = uint_field(um, "promptTokenCount");
        out->response_tokens = uint_field(um, "responseTokenCount");
        out->total_tokens = uint_field(um, "totalTokenCount");
    }

    cJSON_Delete(root);
    return 0;
}

bool live_proto_parse_duration_ms(const char *s, uint32_t *ms)
{
    if (ms) {
        *ms = 0;
    }
    if (!s || !ms || s[0] < '0' || s[0] > '9') {
        return false;
    }
    uint32_t whole = 0;
    while (*s >= '0' && *s <= '9') {
        whole = whole * 10u + (uint32_t)(*s - '0');
        s++;
    }
    uint32_t frac = 0;
    uint32_t scale = 100; /* first fractional digit is hundreds of ms */
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            frac += (uint32_t)(*s - '0') * scale;
            scale /= 10u;
            s++;
        }
    }
    if (s[0] != 's' || s[1] != '\0') {
        return false;
    }
    *ms = whole * 1000u + frac;
    return true;
}
