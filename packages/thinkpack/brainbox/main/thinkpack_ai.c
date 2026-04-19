/**
 * @file thinkpack_ai.c
 * @brief Kconfig-driven AI backend dispatcher for ThinkPack Brainbox.
 *
 * Returns a pointer to the backend struct selected at compile time. The three
 * extern declarations below resolve to the top-level `const thinkpack_ai_backend_t`
 * objects defined in claude_text_backend.c, gemini_text_backend.c, and
 * ollama_text_backend.c respectively.
 */

#include "thinkpack_ai.h"
#include "sdkconfig.h"

extern const thinkpack_ai_backend_t claude_text_backend;
extern const thinkpack_ai_backend_t gemini_text_backend;
extern const thinkpack_ai_backend_t ollama_text_backend;

const thinkpack_ai_backend_t *thinkpack_ai_get_current(void)
{
#if defined(CONFIG_AI_BACKEND_CLAUDE)
    return &claude_text_backend;
#elif defined(CONFIG_AI_BACKEND_GEMINI)
    return &gemini_text_backend;
#else
    return &ollama_text_backend;
#endif
}
