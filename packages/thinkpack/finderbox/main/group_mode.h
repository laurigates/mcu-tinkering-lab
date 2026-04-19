/**
 * @file group_mode.h
 * @brief Mesh-aware group behaviour for Finderbox — SEEK Hot-Cold +
 *        NFC Story Sounds dispatch.
 *
 * Register @ref group_mode_on_event as the thinkpack_mesh event
 * callback.  The module watches for peer beacons (used to update RSSI
 * samples for the current SEEK target) and large-message events
 * carrying a Brainbox reply for an earlier STORY request.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/198
 */

#ifndef FINDERBOX_GROUP_MODE_H
#define FINDERBOX_GROUP_MODE_H

#include "espnow_mesh.h"
#include "thinkpack_nfc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the group-mode state machine.
 *
 * Must be called once after standalone_mode_start() and before
 * thinkpack_mesh_set_event_callback().  Safe to call with or without
 * a populated registry; behaviour lookups dereference the registry
 * each time a tag is scanned.
 *
 * @param registry  Non-NULL pointer to the live tag registry used by
 *                  standalone_mode; must outlive the mesh.
 */
void group_mode_init(thinkpack_nfc_registry_t *registry);

/**
 * @brief Mesh event handler — forward every event to this function.
 *
 * The handler is intentionally a thin router: PEER_DISCOVERED and
 * LARGE_MESSAGE_RECEIVED drive the visible behaviour; all other event
 * types are logged and otherwise ignored.
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

/**
 * @brief Notify group mode that a tag was just scanned locally.
 *
 * Called by standalone_mode after a successful UID lookup.  Behaviour:
 *   - SEEK      → capture target UID so subsequent beacons drive
 *                 Hot-Cold.
 *   - STORY     → emit MSG_LLM_REQUEST to the mesh leader (if known).
 *   - CHIME/COLOR/NONE → no additional group-mode action.
 */
void group_mode_on_local_scan(const thinkpack_nfc_entry_t *entry);

#ifdef __cplusplus
}
#endif

#endif /* FINDERBOX_GROUP_MODE_H */
