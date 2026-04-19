/**
 * @file standalone_mode.h
 * @brief Finderbox main loop — poll RC522, dispatch behaviour on UID match.
 *
 * Runs a dedicated task that polls the RC522 roughly every 50 ms and, when
 * a new tag is detected, looks up its behaviour in the tag registry and
 * triggers the piezo + LED ring.  A button short-press replays the last
 * scanned tag's chime.
 *
 * Group-mode / mesh behaviour (Hot-Cold seek, NFC Story Sounds) is handled
 * by PR E on top of this foundation.
 */

#ifndef FINDERBOX_STANDALONE_MODE_H
#define FINDERBOX_STANDALONE_MODE_H

#include "esp_err.h"
#include "thinkpack_nfc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the standalone scan task.
 *
 * Takes ownership of @p registry; the registry pointer must outlive the
 * task (typically a static in main.c).
 */
esp_err_t standalone_mode_start(thinkpack_nfc_registry_t *registry);

/**
 * @brief Replay the last-detected tag's behaviour.
 *
 * Intended as the button short-press callback.  No-op if no tag has been
 * seen yet this session.
 */
void standalone_mode_replay_last(void);

#ifdef __cplusplus
}
#endif

#endif /* FINDERBOX_STANDALONE_MODE_H */
