#ifndef PROMPT_BUILDER_H
#define PROMPT_BUILDER_H

#include <stddef.h>

/**
 * Build a Gemini TTS prompt for a clue.
 *
 * @param buf       Output buffer for the prompt string.
 * @param buf_len   Size of the output buffer.
 * @param toy_name  Name of the toy that was just found.
 * @param found     Number of toys found so far (including this one).
 * @param total     Total number of toys in the hunt.
 * @param hint      Hint text for the next toy, or NULL if this was the last.
 */
void prompt_builder_clue(char *buf, size_t buf_len, const char *toy_name, int found, int total,
                         const char *hint);

/**
 * Build a Gemini TTS prompt for the celebration (all toys found).
 *
 * @param buf       Output buffer for the prompt string.
 * @param buf_len   Size of the output buffer.
 */
void prompt_builder_celebration(char *buf, size_t buf_len);

#endif  // PROMPT_BUILDER_H
