/**
 * @file test_voice_prompts.c
 * @brief The TTS request has the gemini-3.8-flash-tts shape (#844) and
 *        carries a whole SPEECH_TEXT_MAX line for every persona (#847).
 *
 * Built against the real voice_persona.c table, so a persona added or a brief
 * lengthened later is covered without editing this file. The failures pinned
 * here are silent on a device: a cut brief or a cut spoken line produces a
 * valid request and a fluent reply, just not the one the persona asked for.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "speech_queue.h"
#include "speech_tags.h"
#include "tts_request.h"
#include "voice_persona.h"

uint32_t esp_random(void)
{
    return 0x12345678u;
}

static int test_count = 0;

#define ASSERT(cond)                                                         \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            abort();                                                         \
        }                                                                    \
    } while (0)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    printf("     PASS\n");
}

static const char *json_text_at(const cJSON *root, const char *path0, const char *key)
{
    const cJSON *contents = cJSON_GetObjectItem(root, path0);
    const cJSON *parts = cJSON_GetObjectItem(cJSON_GetArrayItem(contents, 0), "parts");
    const cJSON *item = cJSON_GetObjectItem(cJSON_GetArrayItem(parts, 0), key);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}

/* ---- #844 / #847: the TTS request -------------------------------------- */

/** Fill @p out with @p bytes of Finnish text (2-byte 'ä' throughout). */
static void long_finnish_line(char *out, size_t bytes)
{
    size_t w = 0;
    static const char unit[] = "Hyvää päivää ä ";
    while (w + sizeof(unit) - 1 < bytes) {
        memcpy(out + w, unit, sizeof(unit) - 1);
        w += sizeof(unit) - 1;
    }
    out[w - 1] = '.'; /* a sentence ends on its stop, not a space */
    out[w] = '\0';
}

static void test_tts_request_carries_whole_line_and_style(void)
{
    char line[SPEECH_TEXT_MAX];
    long_finnish_line(line, sizeof(line) - 1);
    ASSERT(strlen(line) > 300);

    for (size_t i = 0; voice_persona_at(i) != NULL; i++) {
        const voice_persona_t *p = voice_persona_at(i);
        char *body = tts_request_build(p->tts_style, p->language_code, p->voice, line);
        ASSERT(body != NULL);
        cJSON *root = cJSON_Parse(body);
        ASSERT(root != NULL);

        /* The spoken text is the line alone: no "<style>: " prefix, which
         * 3.8 would read aloud, and nothing cut. */
        const char *text = json_text_at(root, "contents", "text");
        ASSERT(text != NULL);
        ASSERT(strcmp(text, line) == 0);

        const cJSON *parts = cJSON_GetObjectItem(
            cJSON_GetArrayItem(cJSON_GetObjectItem(root, "contents"), 0), "parts");
        const cJSON *meta = cJSON_GetObjectItem(cJSON_GetArrayItem(parts, 0), "speechMetadata");
        const cJSON *style = cJSON_GetObjectItem(meta, "style");
        ASSERT(cJSON_IsString(style) && strcmp(style->valuestring, p->tts_style) == 0);

        const cJSON *speech =
            cJSON_GetObjectItem(cJSON_GetObjectItem(root, "generationConfig"), "speechConfig");
        const cJSON *lang = cJSON_GetObjectItem(speech, "languageCode");
        ASSERT(cJSON_IsString(lang) && strcmp(lang->valuestring, p->language_code) == 0);
        const cJSON *voice = cJSON_GetObjectItem(
            cJSON_GetObjectItem(cJSON_GetObjectItem(speech, "voiceConfig"), "prebuiltVoiceConfig"),
            "voiceName");
        ASSERT(cJSON_IsString(voice) && strcmp(voice->valuestring, p->voice) == 0);

        cJSON_Delete(root);
        free(body);
    }
}

static void test_tts_request_without_style_omits_metadata(void)
{
    char *body = tts_request_build(NULL, "en-US", "Kore", "Hello.");
    ASSERT(body != NULL);
    ASSERT(strstr(body, "speechMetadata") == NULL);
    free(body);
    ASSERT(tts_request_build("x", NULL, "Kore", "Hello.") == NULL);
    ASSERT(tts_request_build("x", "en-US", "Kore", NULL) == NULL);
}

static void check_translate(const char *in, const char *want)
{
    char out[256];
    const size_t n = tts_request_translate_tags(in, out, sizeof(out));
    if (strcmp(out, want) != 0) {
        printf("     in=\"%s\"\n     got=\"%s\"\n     want=\"%s\"\n", in, out, want);
    }
    ASSERT(strcmp(out, want) == 0);
    ASSERT(n == strlen(want));
}

static void test_every_allowed_tag_has_a_3_8_form(void)
{
    for (size_t i = 0; i < SPEECH_TAG_ALLOWED_COUNT; i++) {
        char out[64];
        tts_request_translate_tags(SPEECH_TAG_ALLOWED[i], out, sizeof(out));
        ASSERT(out[0] == '<'); /* translated, not dropped */
        ASSERT(strchr(out, '[') == NULL);
    }
}

static void test_translates_square_tags(void)
{
    check_translate("[sighs] Jaahas, taas este.", "<sighs> Jaahas, taas este.");
    check_translate("Kas vain. [laughs]", "Kas vain. <laugh>");
    check_translate("[whispers] Hiljaa nyt. [gasp] Oho!", "<whispers> Hiljaa nyt. <gasp> Oho!");
    check_translate("Ei tageja.", "Ei tageja.");
    check_translate("", "");
}

static void test_drops_tags_with_no_3_8_form(void)
{
    check_translate("[excited] Hienoa!", "Hienoa!");
    check_translate("No niin. [bored] Taas.", "No niin. Taas.");
    check_translate("Loppu [bored]", "Loppu");
    check_translate("Auki [whis", "Auki"); /* unclosed: never spoken */
}

static void test_translation_never_splits_a_character(void)
{
    /* "ää" is 4 bytes; a 4-byte buffer holds 3 plus the NUL, so the second
     * 'ä' must be dropped whole, not half-written. */
    char out[4];
    ASSERT(tts_request_translate_tags("ää", out, sizeof(out)) == 2);
    ASSERT(strcmp(out, "ä") == 0);

    /* A line an upstream strlcpy cut mid-character loses the stray byte. */
    const char cut[] = {'O', 'h', 'o', ' ', (char)0xC3, '\0'};
    char out2[16];
    ASSERT(tts_request_translate_tags(cut, out2, sizeof(out2)) == 3);
    ASSERT(strcmp(out2, "Oho") == 0);
}

int main(void)
{
    test_run("tts_request_carries_whole_line_and_style",
             test_tts_request_carries_whole_line_and_style);
    test_run("tts_request_without_style_omits_metadata",
             test_tts_request_without_style_omits_metadata);
    test_run("every_allowed_tag_has_a_3_8_form", test_every_allowed_tag_has_a_3_8_form);
    test_run("translates_square_tags", test_translates_square_tags);
    test_run("drops_tags_with_no_3_8_form", test_drops_tags_with_no_3_8_form);
    test_run("translation_never_splits_a_character", test_translation_never_splits_a_character);
    printf("\n%d tests passed\n", test_count);
    return 0;
}
