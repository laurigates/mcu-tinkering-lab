/**
 * @file gemini_parse.c
 * @brief Parser for Gemini Robotics-ER 2 function-call responses.
 *
 * Extracted from gemini_backend.c for host-based unit testing. See
 * gemini_parse.h for the public contract.
 *
 * Response shape this file accepts::
 *
 *   {
 *     "candidates": [{
 *       "content": {
 *         "parts": [{
 *           "functionCall": {
 *             "name": "drive",
 *             "args": { "heading_deg": 0, "distance_cm": 50, "speed_pct": 60 }
 *           }
 *         }]
 *       },
 *       "finishReason": "STOP"
 *     }],
 *     "usageMetadata": { "promptTokenCount": 123, ... }
 *   }
 *
 * Box coordinates for ``track`` follow Gemini's convention: ``[ymin, xmin, ymax, xmax]``,
 * integers normalised 0..1000.
 */

#include "gemini_parse.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "sound_clips_core.h"

/* Copy of the `speak` text while a sound tag is put in front of it. The
 * caller's buffer is SPEECH_TEXT_MAX (320 B); this is the same size, so a
 * full-length line loses only what no longer fits after the tag. */
#define SPEECH_LINE_SCRATCH 320

/* On the host-test build ESP_LOGx macros are no-ops, leaving TAG unused.
 * Suppress with a compiler attribute rather than an #ifdef — keeps the source
 * identical across targets. */
#if defined(__GNUC__) || defined(__clang__)
#define GP_MAYBE_UNUSED __attribute__((unused))
#else
#define GP_MAYBE_UNUSED
#endif

static const char *TAG GP_MAYBE_UNUSED = "gemini_parse";

/* -------------------------------------------------------------------------- */
/* Token usage from the most recent planner response                           */
/* -------------------------------------------------------------------------- */

/** Static rather than an out-parameter, which is safe only because
 *  gemini_parse_response() has exactly one caller on the target
 *  (gemini_backend_plan(), on the planner task). The narrate and voice-turn
 *  paths go through gemini_parse_text(), which deliberately does NOT touch this
 *  — self_report runs on its own task, and a shared slot would let its parse
 *  land between the planner's parse and the planner's read. */
static gemini_usage_t s_last_usage;

void gemini_parse_last_usage(gemini_usage_t *out)
{
    if (out) {
        *out = s_last_usage;
    }
}

esp_err_t gemini_parse_function_call(const char *json_text, goal_t *out_goal)
{
    return gemini_parse_response(json_text, out_goal, NULL, 0);
}

esp_err_t gemini_parse_response(const char *json_text, goal_t *out_goal, char *out_speech,
                                size_t speech_cap)
{
    if (json_text == NULL || out_goal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    out_goal->kind = GOAL_KIND_STOP; /* safe default */
    if (out_speech && speech_cap > 0) {
        out_speech[0] = '\0';
    }

    /* Cleared before the parse, not after it: a caller reading this back must
     * see "no usage from THIS response", never last response's figures dressed
     * up as this one's. plan_budget.c charges an assumed cost when `present` is
     * false, so a stale-but-plausible number here would silently under-charge
     * the fuse — the one failure mode a spend ceiling cannot have. */
    s_last_usage = (gemini_usage_t){.prompt = -1, .output = -1, .total = -1, .present = false};

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        ESP_LOGE(TAG, "failed to parse API response JSON");
        return ESP_FAIL;
    }

    /* Read before any of the branching below, because several paths reach the
     * exit through `goto done`. Extracting this at the bottom (where it used to
     * live) meant a response that parsed as JSON but carried no recognisable
     * functionCall logged no token count at all — exactly the response you most
     * want the cost of. */
    cJSON *usage = cJSON_GetObjectItem(root, "usageMetadata");
    if (usage) {
        cJSON *pt = cJSON_GetObjectItem(usage, "promptTokenCount");
        cJSON *ot = cJSON_GetObjectItem(usage, "candidatesTokenCount");
        cJSON *tt = cJSON_GetObjectItem(usage, "totalTokenCount");
        s_last_usage.prompt = cJSON_IsNumber(pt) ? pt->valueint : -1;
        s_last_usage.output = cJSON_IsNumber(ot) ? ot->valueint : -1;
        s_last_usage.total = cJSON_IsNumber(tt) ? tt->valueint : -1;
        /* `present` tracks the TOTAL specifically, not the object: a
         * usageMetadata block that has lost the one field the budget charges
         * against is no more useful than no block at all, and treating it as
         * present would charge zero. */
        s_last_usage.present = (s_last_usage.total >= 0);
        ESP_LOGI(TAG, "tokens: prompt=%d output=%d total=%d", (int)s_last_usage.prompt,
                 (int)s_last_usage.output, (int)s_last_usage.total);
    }

    esp_err_t result = ESP_FAIL;

    cJSON *candidates = cJSON_GetObjectItem(root, "candidates");
    cJSON *cand0 = candidates ? cJSON_GetArrayItem(candidates, 0) : NULL;
    cJSON *cand_content = cand0 ? cJSON_GetObjectItem(cand0, "content") : NULL;
    cJSON *cand_parts = cand_content ? cJSON_GetObjectItem(cand_content, "parts") : NULL;

    /* The model may emit several parallel function calls — typically one
     * motion call plus a `speak`. Scanning only parts[0] (as this parser did
     * before speech existed) would silently drop whichever came second. */
    cJSON *fn_call = NULL;
    sound_clip_t sound = SOUND_CLIP_NONE; /* first `sound` call wins */
    const int part_count = cand_parts ? cJSON_GetArraySize(cand_parts) : 0;

    for (int i = 0; i < part_count; i++) {
        cJSON *part = cJSON_GetArrayItem(cand_parts, i);
        cJSON *fc = part ? cJSON_GetObjectItem(part, "functionCall") : NULL;
        cJSON *fc_name = fc ? cJSON_GetObjectItem(fc, "name") : NULL;
        if (!cJSON_IsString(fc_name)) {
            continue;
        }

        if (strcmp(fc_name->valuestring, "speak") == 0) {
            cJSON *sp_args = cJSON_GetObjectItem(fc, "args");
            cJSON *text = sp_args ? cJSON_GetObjectItem(sp_args, "text") : NULL;
            if (cJSON_IsString(text) && out_speech && speech_cap > 0) {
                /* snprintf rather than strlcpy: this file also builds on the
                 * host for the unit tests, where strlcpy is not portable. */
                snprintf(out_speech, speech_cap, "%s", text->valuestring);
            }
            continue;
        }

        if (strcmp(fc_name->valuestring, "sound") == 0) {
            cJSON *so_args = cJSON_GetObjectItem(fc, "args");
            cJSON *so_name = so_args ? cJSON_GetObjectItem(so_args, "name") : NULL;
            if (cJSON_IsString(so_name) && sound == SOUND_CLIP_NONE) {
                sound = sound_clip_from_name(so_name->valuestring);
                if (sound == SOUND_CLIP_NONE) {
                    ESP_LOGW(TAG, "sound: unknown clip \"%s\" ignored", so_name->valuestring);
                }
            }
            continue;
        }

        if (!fn_call) {
            fn_call = fc; /* first motion call wins */
        }
    }

    /* The clip leads the line whichever order the calls came in; the TTS task
     * takes the tag out again (sound_clips_split()). A sound with no `speak`
     * becomes a line of just the tag, which plays without a TTS request. */
    if (sound != SOUND_CLIP_NONE && out_speech && speech_cap > 0) {
        char words[SPEECH_LINE_SCRATCH];
        snprintf(words, sizeof(words), "%s", out_speech);
        snprintf(out_speech, speech_cap, "[%s]%s%s", sound_clip_name(sound),
                 words[0] != '\0' ? " " : "", words);
    }

    if (!fn_call) {
        ESP_LOGE(TAG, "no motion functionCall in response (%d parts scanned)", part_count);
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
        result = ESP_OK;
    }

done:
    cJSON_Delete(root);
    return result;
}

/* -------------------------------------------------------------------------- */
/* Plain-text replies                                                          */
/* -------------------------------------------------------------------------- */

/** Collapse newlines to spaces and strip surrounding whitespace, in place.
 *
 *  A spoken line must be one line: the TTS renderer reads the layout otherwise,
 *  and a leading newline becomes an audible pause before the robot says
 *  anything. */
static void sanitize_spoken_line(char *s)
{
    for (char *p = s; *p != '\0'; ++p) {
        if (*p == '\n' || *p == '\r' || *p == '\t') {
            *p = ' ';
        }
    }
    char *start = s;
    while (*start == ' ') {
        ++start;
    }
    if (start != s) {
        memmove(s, start, strlen(start) + 1);
    }
    size_t len = strlen(s);
    while (len > 0 && s[len - 1] == ' ') {
        s[--len] = '\0';
    }
}

/** Copy @p src onto the end of @p out at *pos, never past @p out_len. */
static void append_text(char *out, size_t out_len, size_t *pos, const char *src)
{
    if (*pos + 1 >= out_len) {
        return;
    }
    /* snprintf rather than strlcat: this file also builds on the host for the
     * unit tests, where strlcat is not portable. It always NUL-terminates and
     * returns the length it WOULD have written, so clamp before advancing or a
     * long first part makes pos run past the buffer. */
    const int w = snprintf(out + *pos, out_len - *pos, "%s", src);
    if (w < 0) {
        return;
    }
    *pos += ((size_t)w < out_len - *pos) ? (size_t)w : (out_len - *pos - 1);
}

/** Shared body of gemini_parse_text_meta() and gemini_parse_voice_reply().
 *  @p last_only keeps only the last non-thought text part instead of joining
 *  them all. */
static esp_err_t parse_text_parts(const char *json_text, char *out, size_t out_len,
                                  gemini_text_meta_t *meta, bool last_only)
{
    if (meta) {
        memset(meta, 0, sizeof(*meta));
    }
    if (!json_text || !out || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    cJSON *candidates = cJSON_GetObjectItem(root, "candidates");
    cJSON *c0 = cJSON_IsArray(candidates) ? cJSON_GetArrayItem(candidates, 0) : NULL;
    cJSON *content = c0 ? cJSON_GetObjectItem(c0, "content") : NULL;
    cJSON *parts = content ? cJSON_GetObjectItem(content, "parts") : NULL;

    if (meta && c0) {
        cJSON *finish = cJSON_GetObjectItem(c0, "finishReason");
        if (cJSON_IsString(finish) && finish->valuestring) {
            snprintf(meta->finish_reason, sizeof(meta->finish_reason), "%s", finish->valuestring);
        }
    }

    if (cJSON_IsArray(parts)) {
        out[0] = '\0';
        size_t pos = 0;
        const char *last = NULL;
        const int n = cJSON_GetArraySize(parts);
        if (meta) {
            meta->parts = n;
        }
        for (int i = 0; i < n; ++i) {
            cJSON *part = cJSON_GetArrayItem(parts, i);
            /* Reasoning is never speech (#950): on 2026-11-05 a voice turn
             * spoke 34.8 s of "The user asks ..." in English. */
            if (cJSON_IsTrue(cJSON_GetObjectItem(part, "thought"))) {
                if (meta) {
                    meta->thought_parts++;
                }
                continue;
            }
            cJSON *t = cJSON_GetObjectItem(part, "text");
            if (!cJSON_IsString(t) || !t->valuestring) {
                continue;
            }
            if (meta) {
                meta->text_parts++;
            }
            if (last_only) {
                last = t->valuestring;
            } else {
                /* Once full this does nothing; the loop keeps counting parts
                 * for the log. */
                append_text(out, out_len, &pos, t->valuestring);
            }
        }
        if (last) {
            append_text(out, out_len, &pos, last);
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

esp_err_t gemini_parse_text_meta(const char *json_text, char *out, size_t out_len,
                                 gemini_text_meta_t *meta)
{
    return parse_text_parts(json_text, out, out_len, meta, false);
}

esp_err_t gemini_parse_voice_reply(const char *json_text, char *out, size_t out_len,
                                   gemini_text_meta_t *meta)
{
    /* gemini-flash-latest can write its reasoning as an ordinary text part
     * with no thought flag, then the reply as the next part (#950, logged
     * 2026-11-08 as parts=2 thought=0 text=2). The reply is the last part. */
    return parse_text_parts(json_text, out, out_len, meta, true);
}

esp_err_t gemini_parse_text(const char *json_text, char *out, size_t out_len)
{
    return gemini_parse_text_meta(json_text, out, out_len, NULL);
}
