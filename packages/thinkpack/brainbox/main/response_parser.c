/**
 * @file response_parser.c
 * @brief Parse LLM-generated JSON into ThinkPack command payloads.
 *
 * Uses cJSON (bundled with ESP-IDF via the `json` component) to walk the
 * "commands" array and emit packed payload structs via the caller-supplied
 * callback. All safety clamping mandated by the spec is applied here so that
 * neither the dispatcher nor the target box needs to re-validate.
 */

#include "response_parser.h"

#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "thinkpack_commands.h"
#include "thinkpack_protocol.h"

static const char *TAG = "response_parser";

/* ------------------------------------------------------------------ */
/* Safety limits                                                       */
/* ------------------------------------------------------------------ */

/** Maximum brightness value per channel (60 % of 255 = 153). */
#define MAX_BRIGHTNESS 153u

/** Maximum pattern index for thinkpack_led_pattern_t. */
#define LED_PATTERN_MAX LED_PATTERN_FLASH

/* ------------------------------------------------------------------ */
/* Target-name → box_type mapping                                      */
/* ------------------------------------------------------------------ */

static uint8_t target_name_to_box_type(const char *name)
{
    if (!name || strcmp(name, "all") == 0) {
        return (uint8_t)BOX_UNKNOWN; /* broadcast */
    }
    if (strcmp(name, "glowbug") == 0) {
        return (uint8_t)BOX_GLOWBUG;
    }
    if (strcmp(name, "boombox") == 0) {
        return (uint8_t)BOX_BOOMBOX;
    }
    if (strcmp(name, "chatterbox") == 0) {
        return (uint8_t)BOX_CHATTERBOX;
    }
    if (strcmp(name, "finderbox") == 0) {
        return (uint8_t)BOX_FINDERBOX;
    }
    ESP_LOGW(TAG, "Unknown target '%s' — treating as broadcast", name);
    return (uint8_t)BOX_UNKNOWN;
}

/* ------------------------------------------------------------------ */
/* Pattern-name → enum                                                 */
/* ------------------------------------------------------------------ */

static uint8_t pattern_name_to_enum(const char *name)
{
    if (!name) {
        return (uint8_t)LED_PATTERN_SOLID;
    }
    if (strcmp(name, "breathe") == 0) {
        return (uint8_t)LED_PATTERN_BREATHE;
    }
    if (strcmp(name, "rainbow") == 0) {
        return (uint8_t)LED_PATTERN_RAINBOW;
    }
    if (strcmp(name, "sparkle") == 0) {
        return (uint8_t)LED_PATTERN_SPARKLE;
    }
    if (strcmp(name, "nightlight") == 0) {
        return (uint8_t)LED_PATTERN_NIGHTLIGHT;
    }
    if (strcmp(name, "flash") == 0) {
        return (uint8_t)LED_PATTERN_FLASH;
    }
    /* default: solid */
    return (uint8_t)LED_PATTERN_SOLID;
}

/* ------------------------------------------------------------------ */
/* Per-type command parsers                                            */
/* ------------------------------------------------------------------ */

static void parse_led_pattern(const cJSON *obj, uint8_t box_type, response_command_cb_t cb,
                              void *user_ctx)
{
    cmd_led_pattern_payload_t payload = {0};

    cJSON *r_item = cJSON_GetObjectItemCaseSensitive(obj, "r");
    cJSON *g_item = cJSON_GetObjectItemCaseSensitive(obj, "g");
    cJSON *b_item = cJSON_GetObjectItemCaseSensitive(obj, "b");
    cJSON *pat_item = cJSON_GetObjectItemCaseSensitive(obj, "pattern");

    uint32_t r = cJSON_IsNumber(r_item) ? (uint32_t)r_item->valueint : 128u;
    uint32_t g = cJSON_IsNumber(g_item) ? (uint32_t)g_item->valueint : 128u;
    uint32_t bv = cJSON_IsNumber(b_item) ? (uint32_t)b_item->valueint : 128u;

    /* Safety clamp: brightness ≤ 153 (60 %) */
    payload.r = (uint8_t)(r > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : r);
    payload.g = (uint8_t)(g > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : g);
    payload.b = (uint8_t)(bv > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : bv);

    /* Pattern can be a string name or an integer index */
    if (cJSON_IsString(pat_item) && pat_item->valuestring) {
        payload.pattern = pattern_name_to_enum(pat_item->valuestring);
    } else if (cJSON_IsNumber(pat_item)) {
        uint32_t idx = (uint32_t)pat_item->valueint;
        payload.pattern =
            (uint8_t)(idx > (uint32_t)LED_PATTERN_MAX ? (uint32_t)LED_PATTERN_SOLID : idx);
    } else {
        payload.pattern = (uint8_t)LED_PATTERN_SOLID;
    }

    cb(box_type, (uint8_t)CMD_LED_PATTERN, (const uint8_t *)&payload, sizeof(payload), user_ctx);
}

static void parse_set_mood(const cJSON *obj, uint8_t box_type, response_command_cb_t cb,
                           void *user_ctx)
{
    cmd_set_mood_payload_t payload = {0};

    cJSON *hue_item = cJSON_GetObjectItemCaseSensitive(obj, "hue");
    cJSON *int_item = cJSON_GetObjectItemCaseSensitive(obj, "intensity");

    payload.hue = cJSON_IsNumber(hue_item) ? (uint8_t)hue_item->valueint : 128u;
    payload.intensity = cJSON_IsNumber(int_item) ? (uint8_t)int_item->valueint : 50u;

    cb(box_type, (uint8_t)CMD_SET_MOOD, (const uint8_t *)&payload, sizeof(payload), user_ctx);
}

static void parse_play_melody(const cJSON *obj, uint8_t box_type, response_command_cb_t cb,
                              void *user_ctx)
{
    cmd_play_melody_payload_t payload = {0};

    cJSON *pid_item = cJSON_GetObjectItemCaseSensitive(obj, "pattern_id");
    cJSON *rep_item = cJSON_GetObjectItemCaseSensitive(obj, "repeat_count");

    payload.pattern_id = cJSON_IsNumber(pid_item) ? (uint8_t)pid_item->valueint : 0u;
    payload.repeat_count = cJSON_IsNumber(rep_item) ? (uint8_t)rep_item->valueint : 8u;

    cb(box_type, (uint8_t)CMD_PLAY_MELODY, (const uint8_t *)&payload, sizeof(payload), user_ctx);
}

static void parse_buzz(const cJSON *obj, uint8_t box_type, response_command_cb_t cb, void *user_ctx)
{
    cmd_buzz_payload_t payload = {0};

    cJSON *note_item = cJSON_GetObjectItemCaseSensitive(obj, "note");
    cJSON *dur_item = cJSON_GetObjectItemCaseSensitive(obj, "duration_ms");
    cJSON *semi_item = cJSON_GetObjectItemCaseSensitive(obj, "semitone_shift");

    payload.note = cJSON_IsNumber(note_item) ? (uint8_t)note_item->valueint : 0u;
    payload.duration_ms = cJSON_IsNumber(dur_item) ? (uint16_t)dur_item->valueint : 200u;
    payload.semitone_shift = cJSON_IsNumber(semi_item) ? (int8_t)semi_item->valueint : 0;

    cb(box_type, (uint8_t)CMD_BUZZ, (const uint8_t *)&payload, sizeof(payload), user_ctx);
}

static void parse_play_sequence(const cJSON *obj, uint8_t box_type, response_command_cb_t cb,
                                void *user_ctx)
{
    cmd_play_sequence_payload_t payload = {0};

    cJSON *dur_item = cJSON_GetObjectItemCaseSensitive(obj, "note_duration_ms");
    cJSON *semi_item = cJSON_GetObjectItemCaseSensitive(obj, "semitone_shift");
    cJSON *notes_item = cJSON_GetObjectItemCaseSensitive(obj, "notes");

    payload.note_duration_ms = cJSON_IsNumber(dur_item) ? (uint16_t)dur_item->valueint : 250u;
    payload.semitone_shift = cJSON_IsNumber(semi_item) ? (int8_t)semi_item->valueint : 0;

    if (cJSON_IsArray(notes_item)) {
        int count = cJSON_GetArraySize(notes_item);
        /* Safety clamp: note_count ≤ CMD_SEQUENCE_MAX_NOTES (20) */
        if (count > CMD_SEQUENCE_MAX_NOTES) {
            ESP_LOGW(TAG, "Note count %d exceeds maximum %d — truncating", count,
                     CMD_SEQUENCE_MAX_NOTES);
            count = CMD_SEQUENCE_MAX_NOTES;
        }
        payload.note_count = (uint8_t)count;
        for (int i = 0; i < count; i++) {
            cJSON *note = cJSON_GetArrayItem(notes_item, i);
            payload.notes[i] = cJSON_IsNumber(note) ? (uint8_t)note->valueint : 0u;
        }
    }

    cb(box_type, (uint8_t)CMD_PLAY_SEQUENCE, (const uint8_t *)&payload, sizeof(payload), user_ctx);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t response_parser_parse(const char *json_text, size_t json_length, response_command_cb_t cb,
                                void *user_ctx)
{
    if (!json_text || !cb) {
        ESP_LOGE(TAG, "NULL argument");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_ParseWithLength(json_text, json_length);
    if (!root) {
        const char *err_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON parse failed near: %s", err_ptr ? err_ptr : "(unknown)");
        return ESP_FAIL;
    }

    cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    if (!cJSON_IsArray(commands)) {
        ESP_LOGW(TAG, "JSON root has no 'commands' array — nothing to dispatch");
        cJSON_Delete(root);
        return ESP_OK;
    }

    int dispatched = 0;
    cJSON *cmd_obj = NULL;
    cJSON_ArrayForEach(cmd_obj, commands)
    {
        if (!cJSON_IsObject(cmd_obj)) {
            ESP_LOGW(TAG, "Non-object entry in commands array — skipping");
            continue;
        }

        cJSON *target_item = cJSON_GetObjectItemCaseSensitive(cmd_obj, "target");
        cJSON *type_item = cJSON_GetObjectItemCaseSensitive(cmd_obj, "type");

        const char *target_str = (cJSON_IsString(target_item) && target_item->valuestring)
                                     ? target_item->valuestring
                                     : NULL;
        const char *type_str =
            (cJSON_IsString(type_item) && type_item->valuestring) ? type_item->valuestring : NULL;

        if (!type_str) {
            ESP_LOGW(TAG, "Command entry missing 'type' — skipping");
            continue;
        }

        uint8_t box_type = target_name_to_box_type(target_str);

        if (strcmp(type_str, "led_pattern") == 0) {
            parse_led_pattern(cmd_obj, box_type, cb, user_ctx);
            dispatched++;
        } else if (strcmp(type_str, "set_mood") == 0) {
            parse_set_mood(cmd_obj, box_type, cb, user_ctx);
            dispatched++;
        } else if (strcmp(type_str, "play_melody") == 0) {
            parse_play_melody(cmd_obj, box_type, cb, user_ctx);
            dispatched++;
        } else if (strcmp(type_str, "buzz") == 0) {
            parse_buzz(cmd_obj, box_type, cb, user_ctx);
            dispatched++;
        } else if (strcmp(type_str, "play_sequence") == 0) {
            parse_play_sequence(cmd_obj, box_type, cb, user_ctx);
            dispatched++;
        } else {
            ESP_LOGW(TAG, "Unknown command type '%s' — skipping", type_str);
        }
    }

    ESP_LOGI(TAG, "Parsed %d command(s) from LLM response", dispatched);
    cJSON_Delete(root);
    return ESP_OK;
}
