/**
 * @file gemini_backend.c
 * @brief Google Gemini Robotics-ER 1.6 planner backend.
 *
 * Uploads a JPEG frame to the Gemini generateContent endpoint with a set of
 * tool declarations.  The model responds with a functionCall object which is
 * parsed into a goal_t for the reactive executor.
 *
 * Design notes:
 * - thinking_budget = 0  for minimum latency (~500–1500 ms target).
 * - API key sent via "x-goog-api-key" header (preferred over query param).
 * - crt_bundle_attach used for TLS — requires CONFIG_MBEDTLS_CERTIFICATE_BUNDLE.
 * - Single-task assumption: do NOT call gemini_backend_plan() concurrently.
 * - On any failure the caller receives GOAL_KIND_STOP so the executor holds.
 */

#include "gemini_backend.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "base64.h"
#include "cJSON.h"
#include "credentials_loader.h"
#include "dialogue_style.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gemini_http.h"
#include "gemini_parse.h"
#include "goal_state.h"
#include "planner_task.h" /* PLANNER_LOOP_PERIOD_MS — keeps the stated cadence honest */
#include "speech_budget.h"
#include "voice_persona.h"

static const char *TAG = "gemini_backend";

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

/* Model id must carry the "-preview" suffix — the bare "gemini-robotics-er-1.6"
 * 404s on v1beta ("not found ... or is not supported for generateContent").
 * Verified against ListModels (2026-07): the API advertises
 * gemini-robotics-er-1.6-preview and gemini-robotics-er-1.5-preview. */
#define GEMINI_MODEL "gemini-robotics-er-1.6-preview"
#define GEMINI_BASE_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" GEMINI_MODEL ":generateContent"

/** Text model that phrases self-report facts into a spoken line. A cheap flash
 *  text model — this is a short one-sentence completion, not vision. */
#define NARRATE_MODEL "gemini-flash-latest"
#define NARRATE_BASE_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" NARRATE_MODEL ":generateContent"

/** Narrate uses its own small buffer (not s_response_buf) so it can run
 *  concurrently with the planner on another task without racing the shared
 *  buffer. A one-sentence reply is tiny; 4 kB also absorbs an error body. */
#define NARRATE_RESPONSE_BUF_SIZE (4 * 1024)
#define NARRATE_TIMEOUT_MS 15000
/** Must cover thinking tokens *plus* the sentence: maxOutputTokens is the
 *  combined budget, and thinking is spent first. gemini-flash-latest resolves to
 *  a Gemini 3-era model that always thinks, so too small a value spends the whole
 *  budget on thinking and returns finishReason=MAX_TOKENS with a sentence cut off
 *  mid-word — or no text at all.
 *
 *  Sized from measurement, not guesswork, because thinking is *variable*: the
 *  same prompt used 487–745 thought tokens across runs (a persona brief costs
 *  more than plain English — that same prompt cost 395 in English). 512
 *  truncated every time; 1024 happened to pass but leaves too little margin
 *  above an observed 745. Raising the ceiling is close to free — thinking tokens
 *  are billed whether or not the cap truncates the reply, so the cap only
 *  decides if the sentence survives. */
#define NARRATE_MAX_OUTPUT_TOKENS 2048

/** HTTP response buffer.  16 kB matches the reference client; function-call
 *  responses are much smaller but the buffer is also used to absorb error
 *  bodies from the API. */
#define GEMINI_RESPONSE_BUF_SIZE (16 * 1024)

/** Request timeout.  Gemini ER with thinking_budget=0 is typically <2 s on a
 *  good WiFi link; 15 s gives headroom without holding up the planner loop for
 *  too long when the network is degraded. */
#define GEMINI_TIMEOUT_MS 15000

#define GEMINI_THINKING_BUDGET 0

/** Planner system prompt buffer.
 *
 *  Lives on the planner task stack, so it is sized deliberately rather than
 *  generously: ~800 bytes of fixed instruction, plus the persona's text_brief
 *  (~450) and tag_brief (~350), plus a variation directive (up to
 *  DIALOGUE_STYLE_MAX) and the recalled-lines clause (GEMINI_RECALL_LIST_MAX).
 *  PLANNER_TASK_STACK_SIZE is set with this frame in mind — grow them together.
 */
#define GEMINI_SYSTEM_PROMPT_MAX 3072

/** Cap on the rendered "you already said these" list.
 *
 *  Bounds prompt cost, not correctness: dialogue_style_recent_lines() drops
 *  whole entries (newest first is kept) rather than truncating one, so a
 *  smaller cap simply means a shorter memory in the prompt. The on-device
 *  repetition check still sees the full ring. */
#define GEMINI_RECALL_LIST_MAX 768

/* The two prompt clauses whose length is not fixed at compile time are the
 * recall list and the persona briefs, and they are assembled *before* the
 * closing "function calls only" instruction. Left to saturate, a long persona
 * would silently truncate that instruction away and the model would start
 * answering in prose — a failure with no obvious cause. So the recall list is
 * sized against the room actually left, minus exactly what the closing sentence
 * and this clause's own wrapper need. Both are measured with sizeof from the
 * literals themselves so editing the wording cannot put the arithmetic out of
 * date. */
static const char k_prompt_closing[] = "Respond ONLY with function calls — no prose, no markdown.";
static const char k_prompt_recall_fmt[] =
    "You have already said these recently: %s. Do not repeat any of them, and do not restate "
    "the same observation in different words — if all you would do is say one of them again, "
    "do not call 'speak' at all. ";
#define GEMINI_RECALL_CLAUSE_OVERHEAD (sizeof(k_prompt_recall_fmt) - 3u) /* less "%s", less NUL */
#define GEMINI_PROMPT_TAIL_RESERVE (sizeof(k_prompt_closing) + GEMINI_RECALL_CLAUSE_OVERHEAD)

/* -------------------------------------------------------------------------- */
/* Module state                                                                 */
/* -------------------------------------------------------------------------- */

static char *s_response_buf = NULL; /* allocated in gemini_backend_init() */

/* -------------------------------------------------------------------------- */
/* HTTP accumulator                                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} response_acc_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id != HTTP_EVENT_ON_DATA) {
        return ESP_OK;
    }
    response_acc_t *acc = (response_acc_t *)evt->user_data;
    if (!acc || !acc->buf || !evt->data || evt->data_len <= 0) {
        return ESP_OK;
    }
    size_t avail = acc->cap - 1 - acc->len;
    size_t to_copy = ((size_t)evt->data_len < avail) ? (size_t)evt->data_len : avail;
    if (to_copy > 0) {
        memcpy(acc->buf + acc->len, evt->data, to_copy);
        acc->len += to_copy;
        acc->buf[acc->len] = '\0';
    }
    // cppcheck-suppress knownConditionTrueFalse // true when buffer fills (to_copy==avail<data_len)
    if (to_copy < (size_t)evt->data_len) {
        ESP_LOGW(TAG, "response truncated — discarded %d bytes",
                 (int)((size_t)evt->data_len - to_copy));
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Request JSON construction                                                    */
/* -------------------------------------------------------------------------- */

/**
 * Build the tool declaration array that instructs Gemini to respond with one
 * of drive / track / rotate / stop function calls, optionally accompanied by a
 * speak call (which is not a motion goal — see the note at its declaration).
 *
 * @param allow_speak When false the `speak` declaration is omitted entirely, so
 *        the model *cannot* produce an utterance on this cycle. Withholding the
 *        tool rather than asking for restraint is the point: a stateless model
 *        has no way to know how recently it last spoke, so "use it sparingly"
 *        is advice it cannot act on. See speech_budget.h.
 *
 * Schema follows the Gemini API "functionDeclarations" format:
 *   https://ai.google.dev/api/generate-content#v1beta.Tool
 *
 * Box coordinates use the ER 1.6 convention: [ymin, xmin, ymax, xmax]
 * integers normalised 0..1000.
 */
static cJSON *build_tools(bool allow_speak)
{
    cJSON *tools_arr = cJSON_CreateArray();

    /* ---- drive ---- */
    {
        cJSON *fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "name", "drive");
        cJSON_AddStringToObject(fn, "description",
                                "Drive the robot on a given heading for a set distance.");
        cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "OBJECT");
        cJSON *props = cJSON_AddObjectToObject(params, "properties");

        cJSON *heading = cJSON_CreateObject();
        cJSON_AddStringToObject(heading, "type", "INTEGER");
        cJSON_AddStringToObject(
            heading, "description",
            "Heading in degrees (0=forward, 90=right, -90=left, 180=backward).");
        cJSON_AddItemToObject(props, "heading_deg", heading);

        cJSON *distance = cJSON_CreateObject();
        cJSON_AddStringToObject(distance, "type", "INTEGER");
        cJSON_AddStringToObject(distance, "description", "Distance to travel in centimetres (>0).");
        cJSON_AddItemToObject(props, "distance_cm", distance);

        cJSON *speed = cJSON_CreateObject();
        cJSON_AddStringToObject(speed, "type", "INTEGER");
        cJSON_AddStringToObject(speed, "description", "Motor speed as a percentage 0..100.");
        cJSON_AddItemToObject(props, "speed_pct", speed);

        cJSON *required = cJSON_CreateArray();
        cJSON_AddItemToArray(required, cJSON_CreateString("heading_deg"));
        cJSON_AddItemToArray(required, cJSON_CreateString("distance_cm"));
        cJSON_AddItemToArray(required, cJSON_CreateString("speed_pct"));
        cJSON_AddItemToObject(params, "required", required);

        cJSON_AddItemToArray(tools_arr, fn);
    }

    /* ---- track ---- */
    {
        cJSON *fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "name", "track");
        cJSON_AddStringToObject(fn, "description",
                                "Visually servo toward a bounding box detected in the frame. "
                                "box_2d uses ER 1.6 format: [ymin, xmin, ymax, xmax] normalised "
                                "0..1000.");
        cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "OBJECT");
        cJSON *props = cJSON_AddObjectToObject(params, "properties");

        cJSON *box = cJSON_CreateObject();
        cJSON_AddStringToObject(box, "type", "ARRAY");
        cJSON_AddStringToObject(box, "description",
                                "Bounding box [ymin, xmin, ymax, xmax] integers 0..1000.");
        cJSON *items = cJSON_AddObjectToObject(box, "items");
        cJSON_AddStringToObject(items, "type", "INTEGER");
        cJSON_AddItemToObject(props, "box_2d", box);

        cJSON *max_spd = cJSON_CreateObject();
        cJSON_AddStringToObject(max_spd, "type", "INTEGER");
        cJSON_AddStringToObject(max_spd, "description",
                                "Maximum motor speed as a percentage 0..100.");
        cJSON_AddItemToObject(props, "max_speed_pct", max_spd);

        cJSON *required = cJSON_CreateArray();
        cJSON_AddItemToArray(required, cJSON_CreateString("box_2d"));
        cJSON_AddItemToArray(required, cJSON_CreateString("max_speed_pct"));
        cJSON_AddItemToObject(params, "required", required);

        cJSON_AddItemToArray(tools_arr, fn);
    }

    /* ---- rotate ---- */
    {
        cJSON *fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "name", "rotate");
        cJSON_AddStringToObject(fn, "description", "Rotate in place by the given angle.");
        cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "OBJECT");
        cJSON *props = cJSON_AddObjectToObject(params, "properties");

        cJSON *angle = cJSON_CreateObject();
        cJSON_AddStringToObject(angle, "type", "INTEGER");
        cJSON_AddStringToObject(angle, "description",
                                "Angle in degrees. Positive = clockwise, negative = "
                                "counter-clockwise.");
        cJSON_AddItemToObject(props, "angle_deg", angle);

        cJSON *required = cJSON_CreateArray();
        cJSON_AddItemToArray(required, cJSON_CreateString("angle_deg"));
        cJSON_AddItemToObject(params, "required", required);

        cJSON_AddItemToArray(tools_arr, fn);
    }

    /* ---- stop ---- */
    {
        cJSON *fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "name", "stop");
        cJSON_AddStringToObject(fn, "description", "Immediately halt all movement.");
        cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "OBJECT");
        cJSON *props = cJSON_AddObjectToObject(params, "properties");
        /* stop takes no parameters; empty properties object is valid */
        (void)props;
        cJSON_AddItemToObject(params, "required", cJSON_CreateArray());

        cJSON_AddItemToArray(tools_arr, fn);
    }

    /* ---- speak ----
     * Unlike the four above, `speak` is NOT a motion goal — it is emitted
     * *alongside* one, and travels to the TTS task via speech_queue rather
     * than goal_state. See speech_queue.h for why the two cannot share a
     * lifetime. The parser recovers both from the same response. */
    if (allow_speak) {
        cJSON *fn = cJSON_CreateObject();
        cJSON_AddStringToObject(fn, "name", "speak");
        cJSON_AddStringToObject(fn, "description",
                                "Say something out loud through the robot's speaker. Call this "
                                "IN ADDITION to a movement function, not instead of one. Omit it "
                                "entirely unless this frame shows something you have not already "
                                "remarked on — saying nothing is the normal case.");
        cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
        cJSON_AddStringToObject(params, "type", "OBJECT");
        cJSON *props = cJSON_AddObjectToObject(params, "properties");

        cJSON *text = cJSON_CreateObject();
        cJSON_AddStringToObject(text, "type", "STRING");
        cJSON_AddStringToObject(text, "description",
                                "One short spoken sentence, at most 20 words. Plain text only — "
                                "no markdown, no emoji.");
        cJSON_AddItemToObject(props, "text", text);

        cJSON *required = cJSON_CreateArray();
        cJSON_AddItemToArray(required, cJSON_CreateString("text"));
        cJSON_AddItemToObject(params, "required", required);

        cJSON_AddItemToArray(tools_arr, fn);
    }

    return tools_arr;
}

/**
 * Build the full generateContent request body.
 *
 * Structure:
 * {
 *   "contents": [{ "role": "user", "parts": [
 *       { "inlineData": { "mimeType": "image/jpeg", "data": "<b64>" } },
 *       { "text": "<system prompt>" }
 *   ]}],
 *   "tools": [{ "functionDeclarations": [ ...five functions... ] }],
 *   "generationConfig": {
 *     "thinkingConfig": { "thinkingBudget": 0 }
 *   }
 * }
 */
/** snprintf-append into @p buf, saturating rather than overrunning.
 *  @return the new length, always < @p cap. */
__attribute__((format(printf, 4, 5))) static size_t append_prompt(char *buf, size_t cap, size_t pos,
                                                                  const char *fmt, ...)
{
    if (pos + 1u >= cap) {
        return cap - 1u;
    }
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf + pos, cap - pos, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return pos;
    }
    return (pos + (size_t)n >= cap) ? (cap - 1u) : (pos + (size_t)n);
}

static char *build_request_json(const char *b64_image)
{
    /* Built per call rather than static: the spoken register follows the active
     * persona, which is switchable at runtime. The cadence is stated from
     * PLANNER_LOOP_PERIOD_MS so the model's sense of how often it is consulted
     * cannot drift from the loop that actually calls it. */
    const voice_persona_t *persona = voice_persona_get();

    /* The speech half of the prompt — and the `speak` declaration itself — is
     * assembled only on a cycle where the budget permits an utterance. Every
     * request is stateless, so this is the only place the robot's own recent
     * history exists; see speech_budget.h. */
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const bool may_speak = speech_budget_allows(now_ms);

    char system_prompt[GEMINI_SYSTEM_PROMPT_MAX];
    size_t pos = 0;

    pos = append_prompt(system_prompt, sizeof(system_prompt), pos,
                        "You are the planning brain of a small wheeled robot. "
                        "Examine the image and choose exactly one movement for the robot to take "
                        "next by calling one of: drive, track, rotate, or stop. "
                        "Prefer 'track' when a target object is visible and centred in the frame. "
                        "Call 'stop' when the path is blocked or the scene is ambiguous. ");

    if (may_speak) {
        /* Drawn fresh per request — this is the whole anti-repetition mechanism.
         * The persona brief is constant, so without a directive that differs call
         * to call the model converges on one favourite opening and keeps it. */
        char variation[DIALOGUE_STYLE_MAX];
        const size_t vlen = dialogue_style_directive(
            &persona->openers, &persona->shapes, persona->avoid_lead, variation, sizeof(variation));

        pos = append_prompt(system_prompt, sizeof(system_prompt), pos,
                            "You may ALSO call 'speak' in the same response to say one short "
                            "sentence out loud — do so only if this frame shows something worth "
                            "remarking on, and stay silent otherwise. You are consulted about "
                            "every %u seconds, so narrating every time would be tiresome. "
                            "When you do call 'speak', the spoken text MUST follow this voice: %s ",
                            PLANNER_LOOP_PERIOD_MS / 1000U, persona->text_brief);

        if (vlen) {
            pos = append_prompt(system_prompt, sizeof(system_prompt), pos,
                                "For this one line only: %s ", variation);
        }
        if (persona->tag_brief) {
            pos =
                append_prompt(system_prompt, sizeof(system_prompt), pos, "%s ", persona->tag_brief);
        }

        /* What the robot has actually said, last because it is the instruction
         * most worth having close to the answer — and the one clause that can be
         * shortened without losing anything the others carry. The avoid-list
         * inside `variation` only constrains the first few *words*; this is what
         * stops the model from re-serving the same observation rearranged. */
        size_t room = sizeof(system_prompt) - GEMINI_PROMPT_TAIL_RESERVE;
        room = (pos < room) ? (room - pos) : 0u;
        if (room > GEMINI_RECALL_LIST_MAX) {
            room = GEMINI_RECALL_LIST_MAX;
        }

        char recall[GEMINI_RECALL_LIST_MAX];
        if (room > 0u && dialogue_style_recent_lines(recall, room) > 0u) {
            pos = append_prompt(system_prompt, sizeof(system_prompt), pos, k_prompt_recall_fmt,
                                recall);
        }
    }

    pos = append_prompt(system_prompt, sizeof(system_prompt), pos, "%s", k_prompt_closing);
    (void)pos;

    const char *SYSTEM_PROMPT = system_prompt;

    cJSON *root = cJSON_CreateObject();

    /* contents */
    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON_AddStringToObject(content, "role", "user");
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");

    cJSON *img_part = cJSON_CreateObject();
    cJSON *inline_data = cJSON_AddObjectToObject(img_part, "inlineData");
    cJSON_AddStringToObject(inline_data, "mimeType", "image/jpeg");
    cJSON_AddStringToObject(inline_data, "data", b64_image);
    cJSON_AddItemToArray(parts, img_part);

    cJSON *text_part = cJSON_CreateObject();
    cJSON_AddStringToObject(text_part, "text", SYSTEM_PROMPT);
    cJSON_AddItemToArray(parts, text_part);

    cJSON_AddItemToArray(contents, content);

    /* tools — one tool object containing all five function declarations */
    cJSON *tools_arr = cJSON_AddArrayToObject(root, "tools");
    cJSON *tool_obj = cJSON_CreateObject();
    cJSON *fn_decls = build_tools(may_speak); /* array of function objects */
    cJSON_AddItemToObject(tool_obj, "functionDeclarations", fn_decls);
    cJSON_AddItemToArray(tools_arr, tool_obj);

    /* tool_config: force a function call (never a text reply) */
    cJSON *tool_config = cJSON_AddObjectToObject(root, "toolConfig");
    cJSON *fn_call_config = cJSON_AddObjectToObject(tool_config, "functionCallingConfig");
    cJSON_AddStringToObject(fn_call_config, "mode", "ANY");

    /* generationConfig */
    cJSON *gen_config = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON *thinking = cJSON_AddObjectToObject(gen_config, "thinkingConfig");
    cJSON_AddNumberToObject(thinking, "thinkingBudget", GEMINI_THINKING_BUDGET);

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body; /* caller must free() */
}

/* -------------------------------------------------------------------------- */
/* Response parsing delegated to gemini_parse.c so it can be unit-tested on   */
/* the host without pulling in esp_http_client / WiFi / mbedtls.              */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Public API                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t gemini_backend_init(void)
{
    const char *api_key = get_gemini_api_key();
    if (!api_key || api_key[0] == '\0') {
        ESP_LOGE(TAG, "GEMINI_API_KEY is not set — cannot initialise Gemini backend");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_response_buf) {
        /* Already initialised — idempotent. */
        return ESP_OK;
    }

    s_response_buf = malloc(GEMINI_RESPONSE_BUF_SIZE);
    if (!s_response_buf) {
        ESP_LOGE(TAG, "failed to allocate %d-byte response buffer", GEMINI_RESPONSE_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "initialised — model=%s thinking_budget=%d", GEMINI_MODEL,
             GEMINI_THINKING_BUDGET);
    return ESP_OK;
}

esp_err_t gemini_backend_plan(const uint8_t *jpeg, size_t jpeg_len, goal_t *out_goal,
                              uint32_t *latency_ms_out, char *out_speech, size_t speech_cap)
{
    if (!jpeg || jpeg_len == 0 || !out_goal) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_speech && speech_cap > 0) {
        out_speech[0] = '\0';
    }
    if (!s_response_buf) {
        ESP_LOGE(TAG, "gemini_backend_plan() called before gemini_backend_init()");
        out_goal->kind = GOAL_KIND_STOP;
        return ESP_FAIL;
    }

    const char *api_key = get_gemini_api_key();
    if (!api_key || api_key[0] == '\0') {
        ESP_LOGE(TAG, "API key unavailable at plan time");
        out_goal->kind = GOAL_KIND_STOP;
        return ESP_FAIL;
    }

    const int64_t t_start = esp_timer_get_time();
    esp_err_t err = ESP_FAIL;

    /* ---- Base64-encode the JPEG ---- */
    char *b64 = base64_encode_alloc(jpeg, jpeg_len);
    if (!b64) {
        ESP_LOGE(TAG, "base64 allocation failed");
        out_goal->kind = GOAL_KIND_STOP;
        return ESP_ERR_NO_MEM;
    }

    /* ---- Build request body ---- */
    char *request_body = build_request_json(b64);
    free(b64);
    b64 = NULL;

    if (!request_body) {
        ESP_LOGE(TAG, "request JSON build failed");
        out_goal->kind = GOAL_KIND_STOP;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "POST %s (body=%zu bytes, jpeg=%zu bytes)", GEMINI_BASE_URL, strlen(request_body),
             jpeg_len);

    /* ---- HTTP client ---- */
    response_acc_t acc = {
        .buf = s_response_buf,
        .len = 0,
        .cap = GEMINI_RESPONSE_BUF_SIZE,
    };
    acc.buf[0] = '\0';

    int status = 0;
    err = gemini_http_post(GEMINI_BASE_URL, api_key, request_body, GEMINI_TIMEOUT_MS,
                           http_event_handler, &acc, &status);

    const int64_t t_end = esp_timer_get_time();
    const uint32_t latency_ms = (uint32_t)((t_end - t_start) / 1000);
    if (latency_ms_out) {
        *latency_ms_out = latency_ms;
    }
    ESP_LOGI(TAG, "HTTP %d in %u ms (response body=%zu bytes)", status, (unsigned)latency_ms,
             acc.len);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "request failed: %s status=%d", esp_err_to_name(err), status);
        if (acc.len > 0) {
            ESP_LOGE(TAG, "error body: %.*s", (int)acc.len, acc.buf);
        }
        out_goal->kind = GOAL_KIND_STOP;
        free(request_body);
        return ESP_FAIL;
    }

    /* ---- Parse function calls from response ---- */
    err = gemini_parse_response(acc.buf, out_goal, out_speech, speech_cap);
    if (err != ESP_OK) {
        /* gemini_parse_response already set out_goal->kind = GOAL_KIND_STOP.
         * out_speech may still be populated — a speech-only response is not a
         * reason to stay silent. */
        ESP_LOGW(TAG, "falling back to STOP due to parse failure");
    }

    free(request_body);
    return err;
}

/* -------------------------------------------------------------------------- */
/* Narration (text-only self-report phrasing)                                  */
/* -------------------------------------------------------------------------- */

/** Build the text-only generateContent body for a spoken status line. */
static char *build_narrate_json(const char *facts, bool is_update)
{
    const voice_persona_t *persona = voice_persona_get();

    /* Deliberately no tag_brief on this path — unlike the planner's `speak`,
     * which is an idle observation, this line is a status report. Half of them
     * name a dead subsystem, and a robot that sighs or giggles while announcing
     * its own camera failure is the one delivery nobody wants. The filter in
     * speech_queue_post() still guards the path; this just declines to invite
     * tags in the first place. */

    /* Same per-call draw as the planner path. This one matters more, not less:
     * the self-introduction is spoken on every boot from a prompt whose facts
     * barely change, so it is the line a listener hears repeat. */
    char variation[DIALOGUE_STYLE_MAX];
    const size_t vlen = dialogue_style_directive(&persona->openers, &persona->shapes,
                                                 persona->avoid_lead, variation, sizeof(variation));

    char prompt[2560];
    snprintf(prompt, sizeof(prompt),
             "You are a small wheeled robot named Robocar, speaking aloud. "
             "Voice and language to use: %s\n\n"
             "Here are your live on-device subsystem facts:\n%s\n\n"
             "%s"
             "Write ONE short, friendly spoken sentence (at most 25 words, plain text only — "
             "no markdown, no emoji, no quotes) %s and stating your status, naming anything that "
             "is not responding. Report only the facts above; do not invent hardware."
             "%s%s",
             persona->text_brief, facts,
             is_update ? "This is a status UPDATE: a subsystem's health just changed — keep to "
                         "what changed. "
                       : "",
             is_update ? "noting the change" : "introducing yourself",
             vlen ? "\n\nFor this one line only: " : "", variation);

    cJSON *root = cJSON_CreateObject();

    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON_AddStringToObject(content, "role", "user");
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");
    cJSON *text_part = cJSON_CreateObject();
    cJSON_AddStringToObject(text_part, "text", prompt);
    cJSON_AddItemToArray(parts, text_part);
    cJSON_AddItemToArray(contents, content);

    cJSON *gen_config = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON_AddNumberToObject(gen_config, "maxOutputTokens", NARRATE_MAX_OUTPUT_TOKENS);
    /* 0.9 rather than the original 0.7: the per-call variation directive does
     * the real work of keeping lines distinct, but a warmer sampler stops the
     * model from settling on one phrasing *within* a given directive. Not
     * warmer than this — the prompt asks for a factual status report, and the
     * "do not invent hardware" constraint is the thing temperature erodes. */
    cJSON_AddNumberToObject(gen_config, "temperature", 0.9);
    /* Hold thinking down so most of the token budget reaches the reply.
     * NOTE: the field is "thinkingLevel", not "thinkingBudget" — the numeric
     * thinkingBudget is rejected outright by the Gemini 3-era model behind
     * gemini-flash-latest, which is what made every narrate call fail with a
     * bare HTTP 400 "Request contains an invalid argument". These models
     * always think, so thinking cannot be switched off entirely; budget for it
     * via NARRATE_MAX_OUTPUT_TOKENS instead. */
    cJSON *thinking = cJSON_AddObjectToObject(gen_config, "thinkingConfig");
    cJSON_AddStringToObject(thinking, "thinkingLevel", "low");

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body; /* caller frees */
}

/** Collapse newlines to spaces and strip surrounding whitespace, in place. */
static void sanitize_spoken_line(char *s)
{
    for (char *p = s; *p != '\0'; ++p) {
        if (*p == '\n' || *p == '\r' || *p == '\t') {
            *p = ' ';
        }
    }
    /* trim leading */
    char *start = s;
    while (*start == ' ') {
        ++start;
    }
    if (start != s) {
        memmove(s, start, strlen(start) + 1);
    }
    /* trim trailing */
    size_t len = strlen(s);
    while (len > 0 && s[len - 1] == ' ') {
        s[--len] = '\0';
    }
}

/** Extract and concatenate candidates[0].content.parts[*].text into @p out. */
static esp_err_t parse_narrate_response(const char *json, char *out, size_t out_len)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    cJSON *candidates = cJSON_GetObjectItem(root, "candidates");
    cJSON *c0 = cJSON_IsArray(candidates) ? cJSON_GetArrayItem(candidates, 0) : NULL;
    cJSON *content = c0 ? cJSON_GetObjectItem(c0, "content") : NULL;
    cJSON *parts = content ? cJSON_GetObjectItem(content, "parts") : NULL;

    if (cJSON_IsArray(parts)) {
        out[0] = '\0';
        const int n = cJSON_GetArraySize(parts);
        for (int i = 0; i < n; ++i) {
            cJSON *t = cJSON_GetObjectItem(cJSON_GetArrayItem(parts, i), "text");
            if (cJSON_IsString(t) && t->valuestring) {
                strlcat(out, t->valuestring, out_len);
            }
        }
        if (out[0] != '\0') {
            sanitize_spoken_line(out);
            if (out[0] != '\0') {
                ret = ESP_OK;
            }
        }
    }

    cJSON_Delete(root);
    return ret;
}

esp_err_t gemini_backend_narrate(const char *facts, bool is_update, char *out, size_t out_len)
{
    if (!facts || !out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out[0] = '\0';

    const char *api_key = get_gemini_api_key();
    if (!api_key || api_key[0] == '\0') {
        ESP_LOGW(TAG, "narrate: API key unavailable");
        return ESP_FAIL;
    }

    char *request_body = build_narrate_json(facts, is_update);
    if (!request_body) {
        return ESP_ERR_NO_MEM;
    }

    /* Own buffer — do not touch s_response_buf, which the planner task uses. */
    char *resp = malloc(NARRATE_RESPONSE_BUF_SIZE);
    if (!resp) {
        free(request_body);
        return ESP_ERR_NO_MEM;
    }
    resp[0] = '\0';

    response_acc_t acc = {.buf = resp, .len = 0, .cap = NARRATE_RESPONSE_BUF_SIZE};

    int status = 0;
    esp_err_t err = gemini_http_post(NARRATE_BASE_URL, api_key, request_body, NARRATE_TIMEOUT_MS,
                                     http_event_handler, &acc, &status);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "narrate request failed: %s status=%d", esp_err_to_name(err), status);
        if (acc.len > 0) {
            /* Error at E, matching the planner path: at D this body is hidden
             * at the default log level, which left a 400 showing only its
             * status code and no indication of which field the API rejected. */
            ESP_LOGE(TAG, "narrate error body: %.*s", (int)acc.len, acc.buf);
        }
        free(request_body);
        free(resp);
        return ESP_FAIL;
    }

    err = parse_narrate_response(acc.buf, out, out_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "narrate: no usable text in response");
    }

    free(request_body);
    free(resp);
    return err;
}

void gemini_backend_deinit(void)
{
    if (s_response_buf) {
        free(s_response_buf);
        s_response_buf = NULL;
    }
    ESP_LOGI(TAG, "deinitialized");
}
