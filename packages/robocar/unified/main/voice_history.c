/**
 * @file voice_history.c
 * @brief Rolling multi-turn conversational memory and prompt formatting.
 */

#include "voice_history.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static voice_history_entry_t s_history[VOICE_HISTORY_MAX_TURNS];
static size_t s_count = 0;
static size_t s_head = 0;
static uint32_t s_last_turn_ms = 0;
static bool s_has_history = false;
static uint32_t s_conversation_active_until_ms = 0;

void voice_history_reset(void)
{
    s_count = 0;
    s_head = 0;
    s_last_turn_ms = 0;
    s_has_history = false;
    s_conversation_active_until_ms = 0;
    memset(s_history, 0, sizeof(s_history));
}

bool voice_history_is_ignore(const char *reply)
{
    if (!reply) {
        return true;
    }
    while (*reply == ' ' || *reply == '\t' || *reply == '\r' || *reply == '\n') {
        reply++;
    }
    if (*reply == '\0') {
        return true;
    }
    return (strncmp(reply, VOICE_HISTORY_IGNORE_TAG, strlen(VOICE_HISTORY_IGNORE_TAG)) == 0);
}

void voice_history_mark_conversation_active(uint32_t now_ms)
{
    s_conversation_active_until_ms = now_ms + VOICE_HISTORY_CONVERSATION_WINDOW_MS;
}

bool voice_history_in_conversation(uint32_t now_ms)
{
    if (s_conversation_active_until_ms == 0) {
        return false;
    }
    const int32_t diff = (int32_t)(s_conversation_active_until_ms - now_ms);
    return diff > 0;
}

void voice_history_expire(uint32_t now_ms)
{
    if (s_has_history) {
        const uint32_t elapsed = now_ms - s_last_turn_ms;
        if (elapsed > VOICE_HISTORY_IDLE_TIMEOUT_MS) {
            voice_history_reset();
        }
    }
}

void voice_history_record(const char *reply, uint32_t now_ms)
{
    if (!reply || reply[0] == '\0' || voice_history_is_ignore(reply)) {
        return;
    }

    voice_history_expire(now_ms);

    voice_history_entry_t *entry = &s_history[s_head];
    snprintf(entry->reply, sizeof(entry->reply), "%s", reply);

    s_head = (s_head + 1) % VOICE_HISTORY_MAX_TURNS;
    if (s_count < VOICE_HISTORY_MAX_TURNS) {
        s_count++;
    }
    s_last_turn_ms = now_ms;
    s_has_history = true;
}

size_t voice_history_count(uint32_t now_ms)
{
    voice_history_expire(now_ms);
    return s_count;
}

const char *voice_history_get_reply(size_t index, uint32_t now_ms)
{
    voice_history_expire(now_ms);
    if (index >= s_count) {
        return NULL;
    }
    const size_t slot =
        (s_head + VOICE_HISTORY_MAX_TURNS - s_count + index) % VOICE_HISTORY_MAX_TURNS;
    return s_history[slot].reply;
}

bool voice_history_format_telemetry(const reactive_telemetry_t *tele, char *out, size_t out_size)
{
    if (!out || out_size == 0) {
        return false;
    }
    out[0] = '\0';
    if (!tele) {
        return false;
    }
    if (tele->sensor_failed) {
        snprintf(out, out_size, "Physical state: ultrasonic sensor unavailable.");
    } else {
        snprintf(out, out_size, "Physical state: distance to obstacle ahead is %u cm%s.",
                 (unsigned)tele->distance_cm,
                 tele->reflex_active ? " (obstacle reflex active)" : "");
    }
    return true;
}

void voice_history_build_prompt(const char *name, bool has_image, const char *tele_summary,
                                char *out, size_t out_size)
{
    if (!out || out_size == 0) {
        return;
    }
    const char *robot_name = (name && name[0] != '\0') ? name : "Robocar";
    const char *media = has_image ? "the camera view and audio follow." : "the audio follows.";

    if (tele_summary && tele_summary[0] != '\0') {
        snprintf(out, out_size,
                 "You are a small wheeled robot named %s. %s "
                 "%s Reply with ONE short spoken sentence — "
                 "no preamble, no quotation marks, no markdown.",
                 robot_name, media, tele_summary);
    } else {
        snprintf(out, out_size,
                 "You are a small wheeled robot named %s. %s "
                 "Reply with ONE short spoken sentence — "
                 "no preamble, no quotation marks, no markdown.",
                 robot_name, media);
    }
}

bool voice_history_build_contents(cJSON *contents, const char *prompt_text, const char *b64_jpeg,
                                  const char *b64_wav, uint32_t now_ms)
{
    if (!contents || !prompt_text || !b64_wav) {
        return false;
    }

    voice_history_expire(now_ms);
    const size_t count = s_count;

    for (size_t i = 0; i < count; i++) {
        const char *reply = voice_history_get_reply(i, now_ms);

        cJSON *user_turn = cJSON_CreateObject();
        if (!user_turn) {
            return false;
        }
        cJSON_AddItemToArray(contents, user_turn);
        cJSON_AddStringToObject(user_turn, "role", "user");
        cJSON *user_parts = cJSON_AddArrayToObject(user_turn, "parts");
        if (!user_parts) {
            return false;
        }
        cJSON *user_part = cJSON_CreateObject();
        if (!user_part) {
            return false;
        }
        cJSON_AddItemToArray(user_parts, user_part);
        cJSON_AddStringToObject(user_part, "text", "(User spoke to you)");

        cJSON *model_turn = cJSON_CreateObject();
        if (!model_turn) {
            return false;
        }
        cJSON_AddItemToArray(contents, model_turn);
        cJSON_AddStringToObject(model_turn, "role", "model");
        cJSON *model_parts = cJSON_AddArrayToObject(model_turn, "parts");
        if (!model_parts) {
            return false;
        }
        cJSON *model_part = cJSON_CreateObject();
        if (!model_part) {
            return false;
        }
        cJSON_AddItemToArray(model_parts, model_part);
        cJSON_AddStringToObject(model_part, "text", reply ? reply : "");
    }

    /* Current turn (user) */
    cJSON *curr_turn = cJSON_CreateObject();
    if (!curr_turn) {
        return false;
    }
    cJSON_AddItemToArray(contents, curr_turn);
    cJSON_AddStringToObject(curr_turn, "role", "user");
    cJSON *curr_parts = cJSON_AddArrayToObject(curr_turn, "parts");
    if (!curr_parts) {
        return false;
    }

    cJSON *text_part = cJSON_CreateObject();
    if (!text_part) {
        return false;
    }
    cJSON_AddItemToArray(curr_parts, text_part);
    cJSON_AddStringToObject(text_part, "text", prompt_text);

    if (b64_jpeg && b64_jpeg[0] != '\0') {
        cJSON *img_part = cJSON_CreateObject();
        if (!img_part) {
            return false;
        }
        cJSON_AddItemToArray(curr_parts, img_part);
        cJSON *inline_img = cJSON_AddObjectToObject(img_part, "inlineData");
        if (!inline_img) {
            return false;
        }
        cJSON_AddStringToObject(inline_img, "mimeType", "image/jpeg");
        cJSON_AddStringToObject(inline_img, "data", b64_jpeg);
    }

    cJSON *audio_part = cJSON_CreateObject();
    if (!audio_part) {
        return false;
    }
    cJSON_AddItemToArray(curr_parts, audio_part);
    cJSON *inline_audio = cJSON_AddObjectToObject(audio_part, "inlineData");
    if (!inline_audio) {
        return false;
    }
    cJSON_AddStringToObject(inline_audio, "mimeType", "audio/wav");
    cJSON_AddStringToObject(inline_audio, "data", b64_wav);

    return true;
}

char *voice_history_build_request_body(const char *name, const char *sys_prompt,
                                       const reactive_telemetry_t *tele, bool has_telemetry,
                                       const char *b64_jpeg, const char *b64_wav, uint32_t now_ms)
{
    char tele_summary[128] = {0};
    if (has_telemetry && tele) {
        voice_history_format_telemetry(tele, tele_summary, sizeof(tele_summary));
    }

    char prompt_text[512] = {0};
    voice_history_build_prompt(name, (b64_jpeg != NULL && b64_jpeg[0] != '\0'), tele_summary,
                               prompt_text, sizeof(prompt_text));

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return NULL;
    }

    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    if (!contents ||
        !voice_history_build_contents(contents, prompt_text, b64_jpeg, b64_wav, now_ms)) {
        cJSON_Delete(root);
        return NULL;
    }

    cJSON *sys = cJSON_AddObjectToObject(root, "systemInstruction");
    cJSON *sys_parts = cJSON_AddArrayToObject(sys, "parts");
    cJSON *sys_text = cJSON_CreateObject();
    cJSON_AddItemToArray(sys_parts, sys_text);
    cJSON_AddStringToObject(sys_text, "text",
                            (sys_prompt && sys_prompt[0] != '\0') ? sys_prompt : "Be brief.");

    cJSON *gen = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON_AddNumberToObject(gen, "maxOutputTokens", VOICE_HISTORY_MAX_OUTPUT_TOKENS);
    cJSON *thinking = cJSON_AddObjectToObject(gen, "thinkingConfig");
    cJSON_AddStringToObject(thinking, "thinkingLevel", "low");

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body;
}
