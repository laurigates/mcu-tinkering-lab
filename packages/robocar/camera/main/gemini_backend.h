#ifndef GEMINI_BACKEND_H
#define GEMINI_BACKEND_H

#include "ai_backend.h"

/**
 * @brief Get the AI backend interface for Google Gemini Robotics-ER.
 * @return A pointer to the AI backend interface for Gemini.
 */
const ai_backend_t *gemini_backend_get(void);

#endif  // GEMINI_BACKEND_H
