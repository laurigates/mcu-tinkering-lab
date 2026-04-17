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

#include <stdlib.h>
#include <string.h>

#include "base64.h"
#include "cJSON.h"
#include "credentials_loader.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "goal_state.h"

static const char *TAG = "gemini_backend";

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

#define GEMINI_MODEL "gemini-robotics-er-1.6"
#define GEMINI_BASE_URL \
    "https://generativelanguage.googleapis.com/v1beta/models/" GEMINI_MODEL ":generateContent"

/** HTTP response buffer.  16 kB matches the reference client; function-call
 *  responses are much smaller but the buffer is also used to absorb error
 *  bodies from the API. */
#define GEMINI_RESPONSE_BUF_SIZE (16 * 1024)

/** Request timeout.  Gemini ER with thinking_budget=0 is typically <2 s on a
 *  good WiFi link; 15 s gives headroom without holding up the planner loop for
 *  too long when the network is degraded. */
#define GEMINI_TIMEOUT_MS 15000

#define GEMINI_THINKING_BUDGET 0

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
 * of drive / track / rotate / stop function calls.
 *
 * Schema follows the Gemini API "functionDeclarations" format:
 *   https://ai.google.dev/api/generate-content#v1beta.Tool
 *
 * Box coordinates use the ER 1.6 convention: [ymin, xmin, ymax, xmax]
 * integers normalised 0..1000.
 */
static cJSON *build_tools(void)
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

        cJSON *decl = cJSON_CreateObject();
        cJSON_AddItemToObject(decl, "functionDeclarations",
                              cJSON_CreateArray()); /* replaced below */
        /* Simpler: add fn directly into a declarations array inside the tool */
        cJSON_Delete(decl);

        /* The Gemini API wraps declarations in a "tools" array where each
         * element has a "functionDeclarations" key.  We collect all four
         * functions into a single tool object; build that object outside this
         * helper and just return the individual function objects here.
         * Restructure: return fn objects via the outer array directly. */
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
 *   "tools": [{ "functionDeclarations": [ ...four functions... ] }],
 *   "generationConfig": {
 *     "thinkingConfig": { "thinkingBudget": 0 }
 *   }
 * }
 */
static char *build_request_json(const char *b64_image)
{
    static const char *SYSTEM_PROMPT =
        "You are the planning brain of a small wheeled robot. "
        "Examine the image and choose exactly one action for the robot to take next "
        "by calling one of the provided functions: drive, track, rotate, or stop. "
        "Prefer 'track' when a target object is visible and centred in the frame. "
        "Call 'stop' when the path is blocked or the scene is ambiguous. "
        "Respond ONLY with a single function call — no prose, no markdown.";

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

    /* tools — one tool object containing all four function declarations */
    cJSON *tools_arr = cJSON_AddArrayToObject(root, "tools");
    cJSON *tool_obj = cJSON_CreateObject();
    cJSON *fn_decls = build_tools(); /* array of function objects */
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
/* Response parsing                                                             */
/* -------------------------------------------------------------------------- */

/**
 * Navigate candidates[0].content.parts[0].functionCall from the raw API
 * response and populate *out_goal.
 *
 * Gemini function-call response shape:
 * {
 *   "candidates": [{
 *     "content": {
 *       "parts": [{
 *         "functionCall": {
 *           "name": "drive",
 *           "args": { "heading_deg": 0, "distance_cm": 50, "speed_pct": 60 }
 *         }
 *       }]
 *     }
 *   }]
 * }
 */
static esp_err_t parse_function_call(const char *json_text, goal_t *out_goal)
{
    out_goal->kind = GOAL_KIND_STOP; /* safe default */

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        ESP_LOGE(TAG, "failed to parse API response JSON");
        return ESP_FAIL;
    }

    esp_err_t result = ESP_FAIL;

    cJSON *candidates = cJSON_GetObjectItem(root, "candidates");
    cJSON *cand0 = candidates ? cJSON_GetArrayItem(candidates, 0) : NULL;
    cJSON *cand_content = cand0 ? cJSON_GetObjectItem(cand0, "content") : NULL;
    cJSON *cand_parts = cand_content ? cJSON_GetObjectItem(cand_content, "parts") : NULL;
    cJSON *part0 = cand_parts ? cJSON_GetArrayItem(cand_parts, 0) : NULL;
    cJSON *fn_call = part0 ? cJSON_GetObjectItem(part0, "functionCall") : NULL;

    if (!fn_call) {
        ESP_LOGE(TAG, "no functionCall in response — candidates[0].content.parts[0].functionCall "
                      "missing");
        /* Log finish reason if present for diagnostics */
        cJSON *finish_reason = cand0 ? cJSON_GetObjectItem(cand0, "finishReason") : NULL;
        if (cJSON_IsString(finish_reason)) {
            ESP_LOGE(TAG, "finishReason: %s", finish_reason->valuestring);
        }
        goto done;
    }

    cJSON *name_item = cJSON_GetObjectItem(fn_call, "name");
    cJSON *args = cJSON_GetObjectItem(fn_call, "args");

    if (!cJSON_IsString(name_item)) {
        ESP_LOGE(TAG, "functionCall.name missing or not a string");
        goto done;
    }

    const char *name = name_item->valuestring;
    ESP_LOGI(TAG, "model called function: %s", name);

    if (strcmp(name, "stop") == 0) {
        out_goal->kind = GOAL_KIND_STOP;
        result = ESP_OK;

    } else if (strcmp(name, "drive") == 0) {
        cJSON *h = args ? cJSON_GetObjectItem(args, "heading_deg") : NULL;
        cJSON *d = args ? cJSON_GetObjectItem(args, "distance_cm") : NULL;
        cJSON *s = args ? cJSON_GetObjectItem(args, "speed_pct") : NULL;
        if (!cJSON_IsNumber(h) || !cJSON_IsNumber(d) || !cJSON_IsNumber(s)) {
            ESP_LOGE(TAG, "drive: missing or invalid args");
            goto done;
        }
        out_goal->kind = GOAL_KIND_DRIVE;
        out_goal->params.drive.heading_deg = (int16_t)h->valueint;
        out_goal->params.drive.distance_cm = (uint16_t)d->valueint;
        out_goal->params.drive.speed_pct = (uint8_t)s->valueint;
        result = ESP_OK;

    } else if (strcmp(name, "track") == 0) {
        cJSON *box = args ? cJSON_GetObjectItem(args, "box_2d") : NULL;
        cJSON *ms = args ? cJSON_GetObjectItem(args, "max_speed_pct") : NULL;
        if (!cJSON_IsArray(box) || cJSON_GetArraySize(box) != 4 || !cJSON_IsNumber(ms)) {
            ESP_LOGE(TAG, "track: invalid box_2d (need 4-element array) or missing max_speed_pct");
            goto done;
        }
        out_goal->kind = GOAL_KIND_TRACK;
        /* ER 1.6 box_2d order: [ymin, xmin, ymax, xmax] */
        out_goal->params.track.ymin = (uint16_t)cJSON_GetArrayItem(box, 0)->valueint;
        out_goal->params.track.xmin = (uint16_t)cJSON_GetArrayItem(box, 1)->valueint;
        out_goal->params.track.ymax = (uint16_t)cJSON_GetArrayItem(box, 2)->valueint;
        out_goal->params.track.xmax = (uint16_t)cJSON_GetArrayItem(box, 3)->valueint;
        out_goal->params.track.max_speed_pct = (uint8_t)ms->valueint;
        result = ESP_OK;

    } else if (strcmp(name, "rotate") == 0) {
        cJSON *a = args ? cJSON_GetObjectItem(args, "angle_deg") : NULL;
        if (!cJSON_IsNumber(a)) {
            ESP_LOGE(TAG, "rotate: missing angle_deg");
            goto done;
        }
        out_goal->kind = GOAL_KIND_ROTATE;
        out_goal->params.rotate.angle_deg = (int16_t)a->valueint;
        result = ESP_OK;

    } else {
        ESP_LOGW(TAG, "unrecognised function name: %s — defaulting to stop", name);
        out_goal->kind = GOAL_KIND_STOP;
        result = ESP_OK; /* not a fatal error; safe default applied */
    }

    /* Log usage metadata for cost observability */
    cJSON *usage = cJSON_GetObjectItem(root, "usageMetadata");
    if (usage) {
        cJSON *pt = cJSON_GetObjectItem(usage, "promptTokenCount");
        cJSON *ot = cJSON_GetObjectItem(usage, "candidatesTokenCount");
        cJSON *tt = cJSON_GetObjectItem(usage, "totalTokenCount");
        ESP_LOGI(TAG, "tokens: prompt=%d output=%d total=%d",
                 cJSON_IsNumber(pt) ? pt->valueint : -1, cJSON_IsNumber(ot) ? ot->valueint : -1,
                 cJSON_IsNumber(tt) ? tt->valueint : -1);
    }

done:
    cJSON_Delete(root);
    return result;
}

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
                              uint32_t *latency_ms_out)
{
    if (!jpeg || jpeg_len == 0 || !out_goal) {
        return ESP_ERR_INVALID_ARG;
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

    esp_http_client_config_t cfg = {
        .url = GEMINI_BASE_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = GEMINI_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .user_data = &acc,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "esp_http_client_init() failed");
        out_goal->kind = GOAL_KIND_STOP;
        free(request_body);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "x-goog-api-key", api_key);
    esp_http_client_set_post_field(client, request_body, strlen(request_body));

    err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    const int64_t t_end = esp_timer_get_time();
    const uint32_t latency_ms = (uint32_t)((t_end - t_start) / 1000);
    if (latency_ms_out) {
        *latency_ms_out = latency_ms;
    }
    ESP_LOGI(TAG, "HTTP %d in %u ms (response body=%zu bytes)", status, (unsigned)latency_ms,
             acc.len);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "request failed: %s status=%d", esp_err_to_name(err), status);
        if (acc.len > 0) {
            ESP_LOGE(TAG, "error body: %.*s", (int)acc.len, acc.buf);
        }
        out_goal->kind = GOAL_KIND_STOP;
        err = ESP_FAIL;
        goto cleanup;
    }

    /* ---- Parse function call from response ---- */
    err = parse_function_call(acc.buf, out_goal);
    if (err != ESP_OK) {
        /* parse_function_call already set out_goal->kind = GOAL_KIND_STOP */
        ESP_LOGW(TAG, "falling back to STOP due to parse failure");
    }

cleanup:
    esp_http_client_cleanup(client);
    free(request_body);
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
