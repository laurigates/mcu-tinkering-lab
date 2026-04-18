/**
 * @file planner_task.h
 * @brief 1 Hz planner task — calls Gemini Robotics-ER 1.6 and writes goals.
 *
 * Runs pinned to Core 1 (camera/network core).  Each iteration:
 *   1. Captures a JPEG frame via camera_capture().
 *   2. Calls gemini_backend_plan() with the frame.
 *   3. On success: writes the goal via goal_state_write().
 *   4. On failure: calls goal_state_force_stop() so the executor latches safe.
 *   5. Returns the framebuffer with camera_return_fb().
 *   6. Logs goal kind + latency at INFO level.
 *
 * The reactive executor (Core 0, 30 Hz) reads goals independently; this task
 * only writes.
 *
 * Loop period: PLANNER_LOOP_PERIOD_MS (default 1000 ms, configurable).
 * Stack: 8192 bytes (holds JPEG reference, TLS context, cJSON parse).
 * Priority: 3 (below reactive_controller=5, motor_task=6).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Planner loop period in milliseconds. Override at compile time if needed. */
#ifndef PLANNER_LOOP_PERIOD_MS
#define PLANNER_LOOP_PERIOD_MS 1000U
#endif

/**
 * @brief Initialise the Gemini backend and spawn the planner task.
 *
 * Calls gemini_backend_init() internally.  goal_state_init() must have been
 * called before this function.  WiFi must be up before calling this function
 * because the planner makes outbound HTTPS requests.
 *
 * Idempotent — a second call returns ESP_OK without creating a second task.
 *
 * @return ESP_OK on success.
 *         ESP_ERR_INVALID_STATE if the Gemini API key is absent.
 *         ESP_FAIL if task creation fails.
 */
esp_err_t planner_task_init(void);

/**
 * @brief Stop and delete the planner task; deinitialise the Gemini backend.
 *
 * Calls goal_state_force_stop() before deleting the task so the executor
 * immediately falls back to a safe hold rather than running on a stale goal.
 *
 * Safe to call even if planner_task_init() was not called or failed.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the task was not running.
 */
esp_err_t planner_task_stop(void);

#ifdef __cplusplus
}
#endif
