/**
 * @file gemini_tools_core.c
 * @brief The planner's function declarations (see gemini_tools_core.h).
 *
 * Schema reference:
 *   https://ai.google.dev/api/generate-content#v1beta.Tool
 *
 * Box coordinates use Gemini's convention: [ymin, xmin, ymax, xmax]
 * integers normalised 0..1000.
 */

#include "gemini_tools_core.h"

#include <string.h>

#include "note_gate.h"
#include "sound_clips_core.h"

#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))

static const char *sound_name_at(int i)
{
    return (i >= 0 && i < SOUND_CLIP_COUNT) ? SOUND_CLIP_NAMES[i] : NULL;
}

static const char *note_kind_at(int i)
{
    const int k = NOTE_KIND_NONE + 1 + i;
    return (i >= 0 && k < NOTE_KIND_COUNT) ? note_kind_name((note_kind_t)k) : NULL;
}

/* ---- motion: exactly one per reply (toolConfig mode ANY) ---- */

static const gemini_tool_param_t k_drive_params[] = {
    {"heading_deg", GEMINI_PARAM_INTEGER,
     "Heading in degrees (0=forward, 90=right, -90=left, 180=backward).", NULL},
    {"distance_cm", GEMINI_PARAM_INTEGER, "Distance to travel in centimetres (>0).", NULL},
    {"speed_pct", GEMINI_PARAM_INTEGER, "Motor speed as a percentage 0..100.", NULL},
};

static const gemini_tool_param_t k_track_params[] = {
    {"box_2d", GEMINI_PARAM_INTEGER_ARRAY,
     "Bounding box [ymin, xmin, ymax, xmax] integers 0..1000.", NULL},
    {"max_speed_pct", GEMINI_PARAM_INTEGER, "Maximum motor speed as a percentage 0..100.", NULL},
};

static const gemini_tool_param_t k_rotate_params[] = {
    {"angle_deg", GEMINI_PARAM_INTEGER,
     "Angle in degrees. Positive = clockwise, negative = counter-clockwise.", NULL},
};

/* ---- speak / sound ----
 * Not motion goals: emitted alongside one. Speech travels to the TTS task via
 * speech_queue, not goal_state (speech_queue.h). `sound` is offered only beside
 * `speak`, so the same on-device gates decide whether the robot may make a
 * noise this cycle; the parser turns it into a `[name]` tag on the line. */

static const gemini_tool_param_t k_speak_params[] = {
    {"text", GEMINI_PARAM_STRING,
     "One short spoken sentence, at most 20 words. Plain text only — no markdown, no emoji.", NULL},
};

static const gemini_tool_param_t k_sound_params[] = {
    {"name", GEMINI_PARAM_STRING_ENUM, NULL, sound_name_at},
};

/* ---- note ----
 * A note to the developers, sent to GlitchTip and never spoken (ADR-033).
 * Offered only when note_gate offered a trigger: the model cannot know what
 * happened since its last note, so the firmware decides and the prompt says
 * why. The parser keeps a `note` call out of the motion goal. */

static const gemini_tool_param_t k_note_params[] = {
    {"kind", GEMINI_PARAM_STRING_ENUM,
     "difficulty = something stopped you doing what you meant to; behaviour = something you "
     "notice about how you act; suggestion = a change that would help you.",
     note_kind_at},
    {"text", GEMINI_PARAM_STRING,
     "At most 280 characters of plain English: what you saw, what you tried and what happened. "
     "No markdown.",
     NULL},
};

/* ---- log_discovery / quest_done ----
 * Offered only on a request whose quest offer allows them (quest_prompt.h,
 * ADR-039 §3); the clause says why. The parser keeps both out of the motion
 * goal, and quest_prompt_apply() drops a call the offer did not allow. */

static const gemini_tool_param_t k_discovery_params[] = {
    {"text", GEMINI_PARAM_STRING,
     "At most 160 characters of plain English about what is in view. No markdown.", NULL},
};

static const gemini_tool_param_t k_quest_done_params[] = {
    {"achieved", GEMINI_PARAM_BOOLEAN,
     "true when the quest is done; false when it cannot be done from here.", NULL},
    {"outcome", GEMINI_PARAM_STRING, "At most 120 characters: what you found, or why not.", NULL},
};

#define TOOL(name_, gate_, desc_, params_) {name_, gate_, desc_, params_, COUNT_OF(params_)}

static const gemini_tool_t k_tools[] = {
    TOOL("drive", GEMINI_TOOL_GATE_ALWAYS, "Drive the robot on a given heading for a set distance.",
         k_drive_params),
    TOOL("track", GEMINI_TOOL_GATE_ALWAYS,
         "Visually servo toward a bounding box detected in the frame. box_2d is [ymin, xmin, "
         "ymax, xmax] normalised 0..1000.",
         k_track_params),
    TOOL("rotate", GEMINI_TOOL_GATE_ALWAYS, "Rotate in place by the given angle.", k_rotate_params),
    {"stop", GEMINI_TOOL_GATE_ALWAYS, "Immediately halt all movement.", NULL, 0},
    TOOL("speak", GEMINI_TOOL_GATE_SPEAK,
         "Say something out loud through the robot's speaker. Call this IN ADDITION to a "
         "movement function, not instead of one. Omit it entirely unless this frame shows "
         "something you have not already remarked on — saying nothing is the normal case.",
         k_speak_params),
    TOOL("sound", GEMINI_TOOL_GATE_SPEAK,
         "Play a short sound effect through the speaker, in addition to a movement function: "
         "horn = a polite two-toot car horn, aoogah = a comic vintage klaxon, bell = a bicycle "
         "bell, boing = a cartoon spring. Use one when it fits what the frame shows, such as a "
         "person in your way; usually play none.",
         k_sound_params),
    TOOL("note", GEMINI_TOOL_GATE_NOTE,
         "Write one short note to the robot's developers about your own behaviour or a "
         "difficulty you ran into. It is logged, not spoken. Call this IN ADDITION to a "
         "movement function, and only if you have something specific to report.",
         k_note_params),
    TOOL("log_discovery", GEMINI_TOOL_GATE_QUEST_DISCOVERY,
         "Add one line to your quest log: what you see that bears on the active quest and what "
         "is near it. Call this IN ADDITION to a movement function.",
         k_discovery_params),
    TOOL("quest_done", GEMINI_TOOL_GATE_QUEST_DONE,
         "Close the active quest. Call this IN ADDITION to a movement function.",
         k_quest_done_params),
};

size_t gemini_tools_count(void)
{
    return COUNT_OF(k_tools);
}

const gemini_tool_t *gemini_tools_at(size_t i)
{
    return (i < COUNT_OF(k_tools)) ? &k_tools[i] : NULL;
}

const gemini_tool_t *gemini_tools_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < COUNT_OF(k_tools); i++) {
        if (strcmp(k_tools[i].name, name) == 0) {
            return &k_tools[i];
        }
    }
    return NULL;
}

static const char *kind_type(gemini_param_kind_t kind)
{
    switch (kind) {
        case GEMINI_PARAM_INTEGER:
            return "INTEGER";
        case GEMINI_PARAM_STRING:
        case GEMINI_PARAM_STRING_ENUM:
            return "STRING";
        case GEMINI_PARAM_BOOLEAN:
            return "BOOLEAN";
        case GEMINI_PARAM_INTEGER_ARRAY:
            return "ARRAY";
    }
    return "STRING";
}

static cJSON *param_schema(const gemini_tool_param_t *p)
{
    cJSON *schema = cJSON_CreateObject();
    cJSON_AddStringToObject(schema, "type", kind_type(p->kind));
    if (p->kind == GEMINI_PARAM_STRING_ENUM) {
        cJSON_AddStringToObject(schema, "format", "enum");
        cJSON *values = cJSON_AddArrayToObject(schema, "enum");
        for (int i = 0; p->enum_at != NULL && p->enum_at(i) != NULL; i++) {
            cJSON_AddItemToArray(values, cJSON_CreateString(p->enum_at(i)));
        }
    }
    if (p->description != NULL) {
        cJSON_AddStringToObject(schema, "description", p->description);
    }
    if (p->kind == GEMINI_PARAM_INTEGER_ARRAY) {
        cJSON *items = cJSON_AddObjectToObject(schema, "items");
        cJSON_AddStringToObject(items, "type", "INTEGER");
    }
    return schema;
}

static cJSON *declaration(const gemini_tool_t *t)
{
    cJSON *fn = cJSON_CreateObject();
    cJSON_AddStringToObject(fn, "name", t->name);
    cJSON_AddStringToObject(fn, "description", t->description);
    cJSON *params = cJSON_AddObjectToObject(fn, "parameters");
    cJSON_AddStringToObject(params, "type", "OBJECT");
    cJSON *props = cJSON_AddObjectToObject(params, "properties");
    cJSON *required = cJSON_CreateArray();
    for (size_t i = 0; i < t->param_count; i++) {
        cJSON_AddItemToObject(props, t->params[i].name, param_schema(&t->params[i]));
        cJSON_AddItemToArray(required, cJSON_CreateString(t->params[i].name));
    }
    cJSON_AddItemToObject(params, "required", required);
    return fn;
}

size_t gemini_tools_build(cJSON *decls, uint32_t gate_mask)
{
    if (decls == NULL) {
        return 0;
    }
    size_t n = 0;
    for (size_t i = 0; i < COUNT_OF(k_tools); i++) {
        const uint32_t gate = k_tools[i].gate;
        if (gate != GEMINI_TOOL_GATE_ALWAYS && (gate & gate_mask) == 0) {
            continue;
        }
        cJSON_AddItemToArray(decls, declaration(&k_tools[i]));
        n++;
    }
    return n;
}
