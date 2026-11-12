/**
 * @file speech.h
 * @brief Spoken check names for the bringup sweep.
 *
 * The piezo cue vocabulary (cues.h) carries the VERDICT and remains the primary
 * channel: it works on a board with nothing fitted but power and a piezo, which
 * is the state this tool exists for. Speech carries the NAME, and answers the
 * one question the beeps structurally cannot — *which* check is running right
 * now, and therefore which one the sweep is stuck on when it stops.
 *
 * Clips are raw 16-bit LE PCM at 24 kHz mono, rendered offline by the Gemini
 * TTS API (tools/voices.json, driven by gamepad-synth's tools/tts/generate.py)
 * and embedded via EMBED_FILES. Nothing here touches the network: the device
 * keeps its "no WiFi, no API key, no cost" property, and regenerating the
 * vocabulary is an explicit developer action.
 *
 * 24 kHz mono is audio_player's native format, so the embedded bytes go
 * straight to audio_player_write() with no resampling — the same I2S channel,
 * DMA descriptors, ring and amplifier the `amp` check exercises.
 *
 * Degrades silently. No PSRAM, no amplifier, or a clip missing for a check all
 * mean the sweep says nothing and beeps exactly as it did before. A bench tool
 * whose announcements failed must not look like a board fault.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring the audio path up so later checks can be announced.
 *
 * Wraps audio_player_init(), which is idempotent — check_amp() calls it again
 * later and gets the same instance. Call once, early: a name spoken before a
 * check is only useful if the audio path is up before the FIRST check runs.
 */
esp_err_t speech_init(void);

/** True when a clip would actually be played. */
bool speech_is_ready(void);

/**
 * @brief Speak the name of the check called @p check_name, and wait it out.
 *
 * Blocking: the clip must finish before the check it announces starts, or the
 * `mic` check measures this speech and the `amp` check plays over it. Bounded,
 * so a wedged audio path costs one timeout rather than the sweep.
 *
 * A name with no clip is a silent no-op — see speech_audit().
 */
void speech_say(const char *check_name);

/**
 * @brief Log which checks have a clip and which do not.
 *
 * The clip table is matched to g_checks by NAME, at run time, because a C
 * string cannot be checked against the check table at compile time. A check
 * added without a clip would otherwise just quietly stop being announced, so
 * the coverage is stated once at boot instead of being assumed.
 */
void speech_audit(void);

#ifdef __cplusplus
}
#endif
