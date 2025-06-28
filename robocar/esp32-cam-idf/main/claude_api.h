/**
 * @file claude_backend.h
 * @brief Claude backend for the AI interface
 */

#ifndef CLAUDE_BACKEND_H
#define CLAUDE_BACKEND_H

#include "ai_backend.h"

/**
 * @brief Get the AI backend interface for the Claude API.
 * @return A pointer to the AI backend interface for Claude.
 */
const ai_backend_t* claude_backend_get(void);

#endif // CLAUDE_BACKEND_H