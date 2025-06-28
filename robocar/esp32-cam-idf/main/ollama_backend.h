#ifndef OLLAMA_BACKEND_H
#define OLLAMA_BACKEND_H

#include "ai_backend.h"

/**
 * @brief Get the AI backend interface for the Ollama API.
 * @return A pointer to the AI backend interface for Ollama.
 */
const ai_backend_t* ollama_backend_get(void);

#endif // OLLAMA_BACKEND_H
