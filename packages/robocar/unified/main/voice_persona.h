/**
 * @file voice_persona.h
 * @brief Language + delivery style of everything the robot says.
 *
 * One table, three consumers — keep them in sync by going through here rather
 * than hardcoding a language or voice at any call site:
 *
 *   - gemini_backend.c  picks the *words*  (register/idiom brief injected into
 *                       the planner `speak` prompt and the narrate prompt)
 *   - gemini_tts.c      picks the *sound*  (speechConfig languageCode + voice,
 *                       plus a delivery directive prefixed to the spoken text)
 *   - self_report.c     picks the *fallback* (the canned line used when the
 *                       narrate call fails, so an API outage does not drop the
 *                       robot back into another language mid-character)
 *
 * How style actually reaches the TTS model: Gemini exposes no style/accent/era
 * parameter — delivery is steered by natural-language prompting, i.e. the
 * documented "Say cheerfully: <text>" form. `tts_style` is that prefix.
 * Verified against the live API (2026-07) that the prefix is *interpreted*, not
 * spoken: swapping a ~40-word directive (12+ s if read aloud) for a 3-word one
 * changed the rendered audio by 0.36 s, while the requested delivery did change
 * the duration. Language comes from speechConfig.languageCode (BCP-47).
 *
 * Voice names are the Gemini prebuilt set (Kore, Puck, Charon, Aoede, …) and are
 * language-agnostic. The locale-specific Cloud TTS names (fi-FI-Chirp3-HD-*)
 * are a *different product* and 404 on this endpoint — do not put one in the
 * table without testing it against :generateContent first.
 */

#ifndef VOICE_PERSONA_H
#define VOICE_PERSONA_H

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Longest accepted prebuilt voice name, including NUL. */
#define VOICE_PERSONA_VOICE_MAX 24

typedef enum {
    VOICE_PERSONA_EN_DEFAULT = 0, /**< Plain contemporary English. */
    VOICE_PERSONA_FI_1950,        /**< 1950s Finnish film register. */
    VOICE_PERSONA_COUNT
} voice_persona_id_t;

/** Immutable table entry. All strings are string literals with static storage,
 *  so a pointer handed out here stays valid for the life of the program and can
 *  be read from any task without locking. */
typedef struct {
    const char *slug;          /**< Stable id for console + NVS ("fi-1950"). */
    const char *label;         /**< Human description for `voice` listing. */
    const char *language_code; /**< BCP-47 for speechConfig.languageCode. */
    const char *voice;         /**< Default Gemini prebuilt voice name. */
    const char *tts_style;     /**< Delivery directive; prefixed as "<style>: <text>". */
    const char *text_brief;    /**< Register/idiom brief for text generation. */
    const char *fallback_ok;   /**< Canned line: everything healthy. */
    const char *fallback_fmt;  /**< Canned line with one %s for the fault list. */
    const char *fault_wifi;    /**< Localised subsystem names for the fault list. */
    const char *fault_camera;
    const char *fault_motors;
    const char *fault_audio;
} voice_persona_t;

/**
 * @brief Load the persisted persona/voice from NVS, falling back to
 *        VOICE_PERSONA_DEFAULT (config.h) when nothing is stored.
 *
 * Safe to call before NVS init fails — a read error just leaves the default.
 * Call once during startup, before the planner and TTS tasks are created.
 */
void voice_persona_init(void);

/**
 * @brief Currently selected persona. Never NULL.
 *
 * Lock-free: the returned pointer is a const table entry, and selection is a
 * single word write, so a concurrent switch yields either the old or the new
 * persona — never a torn mix.
 */
const voice_persona_t *voice_persona_get(void);

/**
 * @brief Effective TTS voice: the runtime override when set, else the current
 *        persona's default. Copied out under a lock because the override is
 *        mutable storage, unlike the table's literals.
 */
void voice_persona_effective_voice(char *out, size_t out_len);

/** @brief Select a persona by slug. @return ESP_ERR_NOT_FOUND on unknown slug. */
esp_err_t voice_persona_set(const char *slug, bool persist);

/**
 * @brief Override the prebuilt voice name, independent of persona.
 *
 * Exists so the voice can be A/B'd by ear at runtime: only a listener can judge
 * whether a voice suits a persona, and a reflash per candidate is too slow a
 * loop for that. Pass NULL or "" to clear and fall back to the persona default.
 * The name is not validated here — an unknown name surfaces as an HTTP 400/404
 * from the TTS call, which is logged.
 */
esp_err_t voice_persona_set_voice(const char *voice, bool persist);

/** @brief Table entry at @p index, or NULL when out of range (for listing). */
const voice_persona_t *voice_persona_at(size_t index);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_PERSONA_H */
