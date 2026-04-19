/**
 * @file prompt_builder.h
 * @brief Prompt construction helpers for ThinkPack Brainbox LLM queries.
 *
 * Builds natural-language prompts that describe the current peer group and
 * ask the LLM to generate a collective-play directive JSON object.
 */

#ifndef PROMPT_BUILDER_H
#define PROMPT_BUILDER_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Group manifest summary presented to the LLM.
 *
 * Filled in by the caller from thinkpack-mesh group_manager state before
 * calling prompt_builder_collective().
 */
typedef struct {
    size_t peer_count; /**< Number of valid entries in peers[] */
    struct {
        uint8_t box_type;      /**< thinkpack_box_type_t value */
        uint16_t capabilities; /**< CAP_* bitmask */
        char name[16];         /**< NUL-terminated human-readable name */
    } peers[8];                /**< Up to 8 peers (matches THINKPACK_MAX_PEERS practical limit) */
} group_manifest_t;

/**
 * @brief Build a prompt asking the LLM to generate a collective behaviour
 *        directive for the current peer group.
 *
 * The output is a self-contained prompt string that instructs the LLM to
 * respond with ONLY a JSON object describing per-box-type commands, e.g.:
 *
 * @code
 * {
 *   "commands": [
 *     {"target": "glowbug",  "type": "led_pattern",  "r": 80, "g": 40,
 *      "b": 200, "pattern": "sparkle"},
 *     {"target": "boombox",  "type": "play_melody",  "pattern_id": 2,
 *      "repeat_count": 16}
 *   ]
 * }
 * @endcode
 *
 * @param trigger   One-line description of what caused this call, e.g.
 *                  "new peer joined" or "periodic surprise".  Must not be NULL.
 * @param manifest  Current group description.  Must not be NULL.
 * @param out       Output buffer for the assembled prompt.
 * @param out_size  Size of @p out in bytes.
 * @return Number of bytes written (excluding NUL terminator), or a negative
 *         value on encoding error.
 */
int prompt_builder_collective(const char *trigger, const group_manifest_t *manifest, char *out,
                              size_t out_size);

#endif  // PROMPT_BUILDER_H
