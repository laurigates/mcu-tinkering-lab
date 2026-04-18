/**
 * @file gemini_parse.c
 * @brief Parser for Gemini Robotics-ER 1.6 function-call responses.
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
 * Box coordinates for ``track`` follow ER 1.6: ``[ymin, xmin, ymax, xmax]``,
 * integers normalised 0..1000.
 */

#include "gemini_parse.h"

#include <stddef.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"

/* On the host-test build ESP_LOGx macros are no-ops, leaving TAG and the
 * usageMetadata locals unused. Suppress with a compiler attribute rather
 * than an #ifdef — keeps the source identical across targets. */
#if defined(__GNUC__) || defined(__clang__)
#define GP_MAYBE_UNUSED __attribute__((unused))
#else
#define GP_MAYBE_UNUSED
#endif

static const char *TAG GP_MAYBE_UNUSED = "gemini_parse";

esp_err_t gemini_parse_function_call(const char *json_text, goal_t *out_goal)
{
    if (json_text == NULL || out_goal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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

    cJSON *usage = cJSON_GetObjectItem(root, "usageMetadata");
    if (usage) {
        cJSON *pt GP_MAYBE_UNUSED = cJSON_GetObjectItem(usage, "promptTokenCount");
        cJSON *ot GP_MAYBE_UNUSED = cJSON_GetObjectItem(usage, "candidatesTokenCount");
        cJSON *tt GP_MAYBE_UNUSED = cJSON_GetObjectItem(usage, "totalTokenCount");
        ESP_LOGI(TAG, "tokens: prompt=%d output=%d total=%d",
                 cJSON_IsNumber(pt) ? pt->valueint : -1, cJSON_IsNumber(ot) ? ot->valueint : -1,
                 cJSON_IsNumber(tt) ? tt->valueint : -1);
    }

done:
    cJSON_Delete(root);
    return result;
}
