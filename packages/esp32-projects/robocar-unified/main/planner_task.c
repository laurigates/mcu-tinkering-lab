/**
 * @file planner_task.c
 * @brief 1 Hz planner task implementation.
 *
 * Captures a JPEG frame, calls Gemini Robotics-ER 1.6 via gemini_backend_plan(),
 * and writes the resulting goal into goal_state.  On any failure the planner
 * writes a STOP goal so the reactive executor immediately falls back to safe
 * hold rather than running stale.
 */

#include "planner_task.h"

#include "camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_backend.h"
#include "goal_state.h"

static const char *TAG = "planner";

/* =========================================================================
 * Task constants
 * ========================================================================= */

#define PLANNER_TASK_STACK_SIZE 8192U
#define PLANNER_TASK_PRIORITY 3U
#define PLANNER_TASK_CORE 1

/* =========================================================================
 * Static task state
 * ========================================================================= */

static TaskHandle_t s_planner_task_handle;

/* =========================================================================
 * Goal kind name — for human-readable logging
 * ========================================================================= */

static const char *goal_kind_name(goal_kind_t kind)
{
    switch (kind) {
        case GOAL_KIND_NONE:
            return "none";
        case GOAL_KIND_STOP:
            return "stop";
        case GOAL_KIND_DRIVE:
            return "drive";
        case GOAL_KIND_TRACK:
            return "track";
        case GOAL_KIND_ROTATE:
            return "rotate";
        default:
            return "unknown";
    }
}

/* =========================================================================
 * Planner task body
 * ========================================================================= */

static void planner_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Planner task started on core %d, period %u ms", xPortGetCoreID(),
             PLANNER_LOOP_PERIOD_MS);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        /* ---- 1. Capture frame ---- */
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed — forcing stop");
            goal_state_force_stop();
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
            continue;
        }

        /* ---- 2. Call Gemini planner ---- */
        goal_t goal = {0};
        uint32_t latency_ms = 0;
        esp_err_t ret = gemini_backend_plan(fb->buf, fb->len, &goal, &latency_ms);

        /* ---- 3 / 4. Write goal or force stop ---- */
        if (ret == ESP_OK) {
            esp_err_t write_ret = goal_state_write(&goal, 0 /* use default TTL */);
            if (write_ret != ESP_OK) {
                ESP_LOGW(TAG, "goal_state_write failed: %d", write_ret);
            }
            ESP_LOGI(TAG, "Goal: %s | latency: %" PRIu32 " ms", goal_kind_name(goal.kind),
                     latency_ms);
        } else {
            ESP_LOGW(TAG, "gemini_backend_plan failed (%s) — forcing stop", esp_err_to_name(ret));
            goal_state_force_stop();
        }

        /* ---- 5. Return framebuffer ---- */
        camera_return_fb(fb);

        /* ---- 6. Sleep until next period ---- */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t planner_task_init(void)
{
    if (s_planner_task_handle != NULL) {
        /* Idempotent — already running */
        return ESP_OK;
    }

    esp_err_t ret = gemini_backend_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gemini_backend_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t created =
        xTaskCreatePinnedToCore(planner_task, "planner", PLANNER_TASK_STACK_SIZE, NULL,
                                PLANNER_TASK_PRIORITY, &s_planner_task_handle, PLANNER_TASK_CORE);

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create planner task");
        gemini_backend_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Planner task created (stack %u, priority %u, core %d)", PLANNER_TASK_STACK_SIZE,
             PLANNER_TASK_PRIORITY, PLANNER_TASK_CORE);
    return ESP_OK;
}

esp_err_t planner_task_stop(void)
{
    if (s_planner_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Force stop first so the executor falls back to safe hold immediately */
    goal_state_force_stop();

    vTaskDelete(s_planner_task_handle);
    s_planner_task_handle = NULL;

    gemini_backend_deinit();

    ESP_LOGI(TAG, "Planner task stopped");
    return ESP_OK;
}
