/**
 * @file gemini_tools_core.h
 * @brief The planner's function declarations as data, and the gate that picks them.
 *
 * Each planner tool is one row of a table: its name, the gate that offers it,
 * its description and its parameters. gemini_tools_build() turns the rows a
 * request's gate mask allows into the `functionDeclarations` array of a
 * generateContent request. A tool the mask does not allow is left out, never
 * declared with an instruction not to call it (stateless-model-gating.md §1).
 *
 * No ESP-IDF dependency: the host suite (test_gemini_tools_core.c) builds the
 * same code the firmware sends.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What a request offers beyond the motion tools. OR them into a mask. */
typedef enum {
    GEMINI_TOOL_GATE_ALWAYS = 0,                /**< drive, track, rotate, stop */
    GEMINI_TOOL_GATE_SPEAK = 1u << 0,           /**< speak and sound */
    GEMINI_TOOL_GATE_NOTE = 1u << 1,            /**< note (note_gate offered a trigger) */
    GEMINI_TOOL_GATE_QUEST_DISCOVERY = 1u << 2, /**< log_discovery (quest offer) */
    GEMINI_TOOL_GATE_QUEST_DONE = 1u << 3,      /**< quest_done (quest offer) */
} gemini_tool_gate_t;

/** A parameter's JSON schema shape. */
typedef enum {
    GEMINI_PARAM_INTEGER,
    GEMINI_PARAM_STRING,
    GEMINI_PARAM_BOOLEAN,
    GEMINI_PARAM_INTEGER_ARRAY, /**< ARRAY of INTEGER */
    GEMINI_PARAM_STRING_ENUM,   /**< STRING, format enum, values from enum_at */
} gemini_param_kind_t;

typedef struct {
    const char *name;
    gemini_param_kind_t kind;
    const char *description; /**< NULL: no description key */
    /** STRING_ENUM only: the i-th value, NULL past the last. */
    const char *(*enum_at)(int i);
} gemini_tool_param_t;

/** One function declaration. Every parameter is required. */
typedef struct {
    const char *name;
    uint32_t gate; /**< one gemini_tool_gate_t; ALWAYS for the motion tools */
    const char *description;
    const gemini_tool_param_t *params;
    size_t param_count;
} gemini_tool_t;

/** Number of rows in the table. */
size_t gemini_tools_count(void);

/** Row @p i in declaration order, NULL past the end. */
const gemini_tool_t *gemini_tools_at(size_t i);

/** The row named @p name, NULL for an unknown name or NULL. */
const gemini_tool_t *gemini_tools_find(const char *name);

/**
 * Append every tool @p gate_mask allows to @p decls (a cJSON array), in table
 * order. @return the number appended.
 */
size_t gemini_tools_build(cJSON *decls, uint32_t gate_mask);

#ifdef __cplusplus
}
#endif
