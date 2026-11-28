/**
 * @file test_gemini_tools_core.c
 * @brief Host tests for the planner's tool table (gemini_tools_core.c).
 *
 * The gate is the enforcement half of stateless-model-gating.md §1: a tool the
 * firmware did not offer must be absent from the request, because an
 * instruction not to call it is advisory. These tests pin which tools each
 * gate mask declares, in which order, and the schema shape each row produces.
 */

#include "gemini_tools_core.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "note_gate.h"
#include "sound_clips_core.h"

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

/** The declared names for @p mask, comma-joined into @p out. */
static void names_for(uint32_t mask, char *out, size_t cap)
{
    cJSON *decls = cJSON_CreateArray();
    const size_t n = gemini_tools_build(decls, mask);
    ASSERT(n == (size_t)cJSON_GetArraySize(decls));
    out[0] = '\0';
    const cJSON *fn = NULL;
    cJSON_ArrayForEach(fn, decls)
    {
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(fn, "name");
        ASSERT(cJSON_IsString(name));
        if (out[0] != '\0') {
            strncat(out, ",", cap - strlen(out) - 1);
        }
        strncat(out, name->valuestring, cap - strlen(out) - 1);
    }
    cJSON_Delete(decls);
}

/** The declaration named @p tool under the mask that offers everything. */
static cJSON *declared(cJSON *decls, const char *tool)
{
    const cJSON *fn = NULL;
    cJSON_ArrayForEach(fn, decls)
    {
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(fn, "name");
        if (cJSON_IsString(name) && strcmp(name->valuestring, tool) == 0) {
            return (cJSON *)fn;
        }
    }
    return NULL;
}

#define ALL_GATES                                                                        \
    (GEMINI_TOOL_GATE_SPEAK | GEMINI_TOOL_GATE_NOTE | GEMINI_TOOL_GATE_QUEST_DISCOVERY | \
     GEMINI_TOOL_GATE_QUEST_DONE)

static void test_no_gate_declares_only_the_motion_tools(void)
{
    char names[256];
    names_for(0, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop") == 0);
}

static void test_speak_gate_adds_speak_and_sound(void)
{
    char names[256];
    names_for(GEMINI_TOOL_GATE_SPEAK, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop,speak,sound") == 0);
}

static void test_note_gate_adds_only_note(void)
{
    char names[256];
    names_for(GEMINI_TOOL_GATE_NOTE, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop,note") == 0);
}

static void test_quest_gates_are_independent(void)
{
    char names[256];
    names_for(GEMINI_TOOL_GATE_QUEST_DISCOVERY, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop,log_discovery") == 0);
    names_for(GEMINI_TOOL_GATE_QUEST_DONE, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop,quest_done") == 0);
}

static void test_all_gates_declare_every_row_in_table_order(void)
{
    char names[256];
    names_for(ALL_GATES, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop,speak,sound,note,log_discovery,quest_done") == 0);
    ASSERT(gemini_tools_count() == 9);
}

static void test_unknown_gate_bits_add_nothing(void)
{
    char names[256];
    names_for(0xFFFFFF00u, names, sizeof(names));
    ASSERT(strcmp(names, "drive,track,rotate,stop") == 0);
}

/* Every parameter is required: a reply missing one would be a goal the parser
 * has to guess at, and the API enforces `required` for us. */
static void test_required_lists_every_property_in_order(void)
{
    cJSON *decls = cJSON_CreateArray();
    gemini_tools_build(decls, ALL_GATES);
    const cJSON *fn = NULL;
    cJSON_ArrayForEach(fn, decls)
    {
        const cJSON *params = cJSON_GetObjectItemCaseSensitive(fn, "parameters");
        const cJSON *type = cJSON_GetObjectItemCaseSensitive(params, "type");
        ASSERT(cJSON_IsString(type) && strcmp(type->valuestring, "OBJECT") == 0);
        const cJSON *props = cJSON_GetObjectItemCaseSensitive(params, "properties");
        const cJSON *required = cJSON_GetObjectItemCaseSensitive(params, "required");
        ASSERT(cJSON_IsObject(props) && cJSON_IsArray(required));
        ASSERT(cJSON_GetArraySize(props) == cJSON_GetArraySize(required));
        int i = 0;
        const cJSON *prop = NULL;
        cJSON_ArrayForEach(prop, props)
        {
            const cJSON *req = cJSON_GetArrayItem(required, i++);
            ASSERT(cJSON_IsString(req) && strcmp(req->valuestring, prop->string) == 0);
        }
    }
    cJSON_Delete(decls);
}

static void test_stop_takes_no_parameters(void)
{
    cJSON *decls = cJSON_CreateArray();
    gemini_tools_build(decls, 0);
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(declared(decls, "stop"), "parameters");
    ASSERT(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(params, "properties")) == 0);
    ASSERT(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(params, "required")) == 0);
    cJSON_Delete(decls);
}

static void test_box_is_an_integer_array(void)
{
    cJSON *decls = cJSON_CreateArray();
    gemini_tools_build(decls, 0);
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(declared(decls, "track"), "parameters");
    const cJSON *box = cJSON_GetObjectItemCaseSensitive(
        cJSON_GetObjectItemCaseSensitive(params, "properties"), "box_2d");
    ASSERT(strcmp(cJSON_GetObjectItemCaseSensitive(box, "type")->valuestring, "ARRAY") == 0);
    const cJSON *items = cJSON_GetObjectItemCaseSensitive(box, "items");
    ASSERT(strcmp(cJSON_GetObjectItemCaseSensitive(items, "type")->valuestring, "INTEGER") == 0);
    cJSON_Delete(decls);
}

/** The enum values of @p tool's @p param, as declared. */
static const cJSON *enum_of(cJSON *decls, const char *tool, const char *param)
{
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(declared(decls, tool), "parameters");
    const cJSON *p = cJSON_GetObjectItemCaseSensitive(
        cJSON_GetObjectItemCaseSensitive(params, "properties"), param);
    ASSERT(strcmp(cJSON_GetObjectItemCaseSensitive(p, "format")->valuestring, "enum") == 0);
    return cJSON_GetObjectItemCaseSensitive(p, "enum");
}

/* The enums come from the tables the parser checks against, so a new clip or
 * note kind is offered and accepted from the one edit. */
static void test_sound_enum_is_the_clip_table(void)
{
    cJSON *decls = cJSON_CreateArray();
    gemini_tools_build(decls, GEMINI_TOOL_GATE_SPEAK);
    const cJSON *values = enum_of(decls, "sound", "name");
    ASSERT(cJSON_GetArraySize(values) == SOUND_CLIP_COUNT);
    for (int i = 0; i < SOUND_CLIP_COUNT; i++) {
        ASSERT(strcmp(cJSON_GetArrayItem(values, i)->valuestring, SOUND_CLIP_NAMES[i]) == 0);
    }
    cJSON_Delete(decls);
}

static void test_note_kind_enum_skips_none(void)
{
    cJSON *decls = cJSON_CreateArray();
    gemini_tools_build(decls, GEMINI_TOOL_GATE_NOTE);
    const cJSON *values = enum_of(decls, "note", "kind");
    ASSERT(cJSON_GetArraySize(values) == NOTE_KIND_COUNT - 1);
    for (int k = NOTE_KIND_NONE + 1; k < NOTE_KIND_COUNT; k++) {
        const cJSON *v = cJSON_GetArrayItem(values, k - (NOTE_KIND_NONE + 1));
        ASSERT(strcmp(v->valuestring, note_kind_name((note_kind_t)k)) == 0);
        ASSERT(note_kind_from_name(v->valuestring) == (note_kind_t)k);
    }
    cJSON_Delete(decls);
}

static void test_find_and_bounds(void)
{
    ASSERT(gemini_tools_find("quest_done") == gemini_tools_at(8));
    ASSERT(gemini_tools_find("quest_done")->gate == GEMINI_TOOL_GATE_QUEST_DONE);
    ASSERT(gemini_tools_find("wait") == NULL);
    ASSERT(gemini_tools_find(NULL) == NULL);
    ASSERT(gemini_tools_at(gemini_tools_count()) == NULL);
    ASSERT(gemini_tools_build(NULL, ALL_GATES) == 0);
}

static void test_names_are_unique(void)
{
    for (size_t i = 0; i < gemini_tools_count(); i++) {
        ASSERT(gemini_tools_find(gemini_tools_at(i)->name) == gemini_tools_at(i));
    }
}

int main(void)
{
    printf("=== gemini_tools_core Host Tests ===\n\n");
    test_run("no gate declares only the motion tools", test_no_gate_declares_only_the_motion_tools);
    test_run("speak gate adds speak and sound", test_speak_gate_adds_speak_and_sound);
    test_run("note gate adds only note", test_note_gate_adds_only_note);
    test_run("quest gates are independent", test_quest_gates_are_independent);
    test_run("all gates declare every row in table order",
             test_all_gates_declare_every_row_in_table_order);
    test_run("unknown gate bits add nothing", test_unknown_gate_bits_add_nothing);
    test_run("required lists every property in order", test_required_lists_every_property_in_order);
    test_run("stop takes no parameters", test_stop_takes_no_parameters);
    test_run("box is an integer array", test_box_is_an_integer_array);
    test_run("sound enum is the clip table", test_sound_enum_is_the_clip_table);
    test_run("note kind enum skips none", test_note_kind_enum_skips_none);
    test_run("find and bounds", test_find_and_bounds);
    test_run("names are unique", test_names_are_unique);
    printf("\n=== %d tests passed ===\n", test_count);
    return 0;
}
