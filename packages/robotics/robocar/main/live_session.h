/**
 * @file live_session.h
 * @brief Real-time conversation over the Gemini Live API (ADR-032 phase 1).
 *
 * With `voice live on`, a hands-free trigger opens one Live session instead of
 * a record → generateContent → TTS turn. The session streams the microphone up
 * as 16 kHz PCM, the pre-roll that holds the triggering words first, and plays
 * the reply's 24 kHz PCM through the same ring and player as TTS. One request
 * per conversation instead of two per turn, and about 10 s less wait per turn
 * (the probe measured 1.1–1.4 s from end of speech to first audio).
 *
 * Half-duplex: nothing goes upstream while Teuvo speaks and for
 * AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT after (live_session_core.h). No
 * barge-in in this phase.
 *
 * The session runs on the voice-turn task (voice_turn.c), so it shares that
 * task's stack, its 1-deep queue, the budget checks at enqueue, and
 * voice_turn_is_busy(), which already keeps the planner's line off the
 * speaker and the OTA install back. `listen` keeps the old path.
 *
 * Off at boot, not persisted, like every other voice switch.
 */

#ifndef LIVE_SESSION_H
#define LIVE_SESSION_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Consecutive sessions that fail before setupComplete that switch live mode
 *  back off, so a broken endpoint or an unsupported model falls back to the
 *  old voice turn instead of eating every trigger. */
#define LIVE_SESSION_MAX_CONNECT_FAILURES 3

/** @brief `voice live on|off`. Off clears the failure count. */
void live_session_set_enabled(bool enabled);

/** @brief Whether hands-free triggers open Live sessions. */
bool live_session_enabled(void);

/** @brief `voice live idle <s>` / `voice live cap <s>` (0 keeps the value). */
void live_session_configure(uint32_t idle_ms, uint32_t max_ms);
void live_session_get_config(uint32_t *idle_ms, uint32_t *max_ms);

/**
 * @brief Run one session to its end. Called on the voice-turn task only.
 *
 * Blocks for the session's length. Charges the voice budget once, when the
 * server confirms the setup.
 *
 * @return ESP_OK once a session opened (however it ended); an error when it
 *         never got past setup.
 */
esp_err_t live_session_run(const char *api_key);

/** @brief True while a session is connecting or open. */
bool live_session_is_open(void);

/** @brief Ask the open session to close at its next chunk (`voice live stop`). */
void live_session_request_stop(void);

/** @brief Totals since boot for `voice`. Every pointer is optional. */
void live_session_stats(uint32_t *sessions, uint32_t *failures, uint32_t *turns, uint32_t *waits);

#ifdef __cplusplus
}
#endif

#endif  // LIVE_SESSION_H
