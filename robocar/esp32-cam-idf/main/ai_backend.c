/**
 * @file ai_backend.c
 * @brief AI backend selection logic.
 */

#include "ai_backend.h"
#include "sdkconfig.h"

// Include the backend that is selected in the menuconfig
#if defined(CONFIG_AI_BACKEND_CLAUDE)
#include "claude_backend.h"
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
#include "ollama_backend.h"
#endif

const ai_backend_t* ai_backend_get_current(void) {
#if defined(CONFIG_AI_BACKEND_CLAUDE)
    return claude_backend_get();
#elif defined(CONFIG_AI_BACKEND_OLLAMA)
    return ollama_backend_get();
#else
    // Default or error case
    return NULL;
#endif
}
