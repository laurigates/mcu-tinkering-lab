/**
 * @file tts_request.c
 * @brief Gemini TTS request body. See tts_request.h.
 */

#include "tts_request.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

typedef struct {
    const char *square; /**< As speech_tags.h allows it. */
    const char *angle;  /**< As the 3.8 vocal-tag table lists it. */
} tag_map_t;

/* [excited] and [bored] are delivery moods, not point-in-time vocal events,
 * and the 3.8 table has no entry for either; they are no longer allowed. */
static const tag_map_t k_tag_map[] = {
    {"[sighs]", "<sighs>"},
    {"[laughs]", "<laugh>"},
    {"[whispers]", "<whispers>"},
    {"[gasp]", "<gasp>"},
};

#define TAG_MAP_COUNT (sizeof(k_tag_map) / sizeof(k_tag_map[0]))

/** Length of the bracketed run at @p p (which points at '['), or 0 if open. */
static size_t bracket_run_len(const char *p)
{
    const char *close = strchr(p, ']');
    return close ? (size_t)(close - p) + 1 : 0;
}

static const char *angle_for(const char *run, size_t len)
{
    for (size_t i = 0; i < TAG_MAP_COUNT; i++) {
        if (strlen(k_tag_map[i].square) == len && memcmp(k_tag_map[i].square, run, len) == 0) {
            return k_tag_map[i].angle;
        }
    }
    return NULL;
}

/** Bytes in the UTF-8 sequence starting with lead byte @p c (1 for invalid). */
static size_t utf8_seq_len(unsigned char c)
{
    if (c >= 0xF0) {
        return 4;
    }
    if (c >= 0xE0) {
        return 3;
    }
    if (c >= 0xC0) {
        return 2;
    }
    return 1;
}

/** Append @p n bytes if they fit whole; returns false once out is full. */
static bool append(char *out, size_t out_len, size_t *w, const char *s, size_t n)
{
    if (*w + n + 1 > out_len) {
        return false;
    }
    memcpy(out + *w, s, n);
    *w += n;
    return true;
}

size_t tts_request_translate_tags(const char *in, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return 0;
    }
    size_t w = 0;
    out[0] = '\0';
    if (in == NULL) {
        return 0;
    }

    const char *p = in;
    while (*p != '\0') {
        if (*p == '[') {
            const size_t run = bracket_run_len(p);
            if (run == 0) {
                break; /* an unclosed tag would be spoken; drop the rest */
            }
            const char *angle = angle_for(p, run);
            p += run;
            if (angle != NULL) {
                if (!append(out, out_len, &w, angle, strlen(angle))) {
                    break;
                }
            } else if (*p == ' ' && (w == 0 || out[w - 1] == ' ')) {
                p++; /* removed run: do not leave a double or leading space */
            }
            continue;
        }
        const size_t n = utf8_seq_len((unsigned char)*p);
        size_t avail = 0;
        while (avail < n && p[avail] != '\0') {
            avail++;
        }
        /* A sequence cut short (an upstream buffer truncated mid-character)
         * would make the JSON string invalid UTF-8; drop it. */
        if (avail < n || !append(out, out_len, &w, p, avail)) {
            break;
        }
        p += avail;
    }
    while (w > 0 && out[w - 1] == ' ') {
        w--;
    }
    out[w] = '\0';
    return w;
}

char *tts_request_build(const char *style, const char *language_code, const char *voice,
                        const char *text)
{
    if (language_code == NULL || voice == NULL || text == NULL) {
        return NULL;
    }

    /* A translated tag is at most one byte shorter or longer than its source
     * ("[laughs]" -> "<laugh>"), so twice the input bounds the output. */
    const size_t cap = 2 * strlen(text) + 1;
    char *spoken = malloc(cap);
    if (spoken == NULL) {
        return NULL;
    }
    tts_request_translate_tags(text, spoken, cap);

    char *json = NULL;
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        goto done;
    }

    cJSON *contents = cJSON_AddArrayToObject(root, "contents");
    cJSON *content = cJSON_CreateObject();
    cJSON_AddItemToArray(contents, content);
    cJSON *parts = cJSON_AddArrayToObject(content, "parts");
    cJSON *part = cJSON_CreateObject();
    cJSON_AddItemToArray(parts, part);
    cJSON_AddStringToObject(part, "text", spoken);
    if (style != NULL && style[0] != '\0') {
        cJSON *meta = cJSON_AddObjectToObject(part, "speechMetadata");
        cJSON_AddStringToObject(meta, "style", style);
    }

    cJSON *gen_cfg = cJSON_AddObjectToObject(root, "generationConfig");
    cJSON *modalities = cJSON_AddArrayToObject(gen_cfg, "responseModalities");
    cJSON_AddItemToArray(modalities, cJSON_CreateString("AUDIO"));
    cJSON *speech_cfg = cJSON_AddObjectToObject(gen_cfg, "speechConfig");
    cJSON_AddStringToObject(speech_cfg, "languageCode", language_code);
    cJSON *voice_cfg = cJSON_AddObjectToObject(speech_cfg, "voiceConfig");
    cJSON *prebuilt = cJSON_AddObjectToObject(voice_cfg, "prebuiltVoiceConfig");
    cJSON_AddStringToObject(prebuilt, "voiceName", voice);

    json = cJSON_PrintUnformatted(root);

done:
    cJSON_Delete(root);
    free(spoken);
    return json;
}
