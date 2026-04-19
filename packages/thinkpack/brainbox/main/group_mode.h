/**
 * @file group_mode.h
 * @brief LLM-driven collective behaviour coordinator for the ThinkPack Brainbox.
 *
 * group_mode owns a FreeRTOS worker task (Core 1) that dequeues trigger strings,
 * builds a prompt from the current peer group, queries the active AI backend,
 * parses the JSON response, and dispatches commands to peers via thinkpack-mesh.
 *
 * Typical usage (from app_main):
 * @code
 *   ESP_ERROR_CHECK(group_mode_init());
 *   thinkpack_mesh_set_event_callback(group_mode_on_event, NULL);
 *   ESP_ERROR_CHECK(thinkpack_mesh_start());
 *   // Later, from any task or button handler:
 *   group_mode_trigger_llm("button pressed");
 * @endcode
 */

#ifndef GROUP_MODE_H
#define GROUP_MODE_H

#include "esp_err.h"
#include "espnow_mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the group-mode subsystem.
 *
 * Creates the trigger queue and pins the worker task to Core 1.
 * Must be called after thinkpack_mesh_init() and before thinkpack_mesh_start().
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue or task creation fails.
 */
esp_err_t group_mode_init(void);

/**
 * @brief Mesh event callback — register with thinkpack_mesh_set_event_callback().
 *
 * Handles:
 *   - THINKPACK_EVENT_PEER_DISCOVERED → triggers "new peer joined" LLM query
 *   - THINKPACK_EVENT_BECAME_LEADER   → logs the election result
 *   - THINKPACK_EVENT_LARGE_MESSAGE_RECEIVED with MSG_LLM_RESPONSE → logs length
 *
 * Called from the receive task on Core 0.  Keep processing minimal — heavy work
 * is deferred to the worker task via the trigger queue.
 *
 * @param event     Event descriptor; valid only during this call.
 * @param user_ctx  Unused (pass NULL to thinkpack_mesh_set_event_callback).
 */
void group_mode_on_event(const thinkpack_mesh_event_data_t *event, void *user_ctx);

/**
 * @brief Schedule an LLM-driven collective behaviour.
 *
 * Non-blocking: pushes @p trigger onto the worker queue.  If the queue is full
 * the trigger is silently dropped (the system self-regulates under load).
 *
 * @param trigger  Short description of the cause, e.g. "button pressed".
 *                 Must not be NULL; copied internally before return.
 */
void group_mode_trigger_llm(const char *trigger);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_MODE_H */
