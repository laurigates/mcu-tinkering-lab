/**
 * @file planner_task.c
 * @brief Gemini planner task implementation (PLANNER_LOOP_PERIOD_MS, 15 s default).
 *
 * Captures a JPEG frame, calls Gemini Robotics-ER 1.6 via gemini_backend_plan(),
 * and writes the resulting goal into goal_state.  On any failure the planner
 * writes a STOP goal so the reactive executor immediately falls back to safe
 * hold rather than running stale.
 */

#include "planner_task.h"

#include "camera.h"
#include "dialogue_style.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_backend.h"
#include "goal_state.h"
#include "speech_queue.h"

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

    /* Extra periods to wait after a failed plan. Gemini answers a quota
     * overrun with HTTP 429 and a retryDelay that *grows* while the client
     * keeps asking (observed climbing 19 s -> 31 s under a 1 Hz loop), so
     * retrying at full rate makes the outage longer. Capped so recovery stays
     * bounded; reset on the first success. */
    uint32_t backoff_periods = 0;

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
        char speech[SPEECH_TEXT_MAX] = {0};
        esp_err_t ret =
            gemini_backend_plan(fb->buf, fb->len, &goal, &latency_ms, speech, sizeof(speech));

        /* ---- 2b. Hand any utterance to the TTS task ----
         * Posted before the goal is written and regardless of whether the
         * motion goal parsed, so the robot can speak while holding position.
         * Non-blocking: a full queue drops the line rather than stalling the
         * planner loop. */
        if (speech[0] != '\0') {
            const esp_err_t sp_ret = speech_queue_post(speech);
            if (sp_ret == ESP_OK) {
                /* Feeds the next prompt's "do not begin with" list. Only on a
                 * successful post: a line dropped by a full queue is never
                 * heard, so it is not a repetition anyone can notice. */
                dialogue_style_note_spoken(speech);
                ESP_LOGI(TAG, "Speech: \"%s\"", speech);
            } else if (sp_ret == ESP_ERR_NO_MEM) {
                ESP_LOGD(TAG, "still speaking — dropped: \"%s\"", speech);
            }
        }

        /* ---- 3 / 4. Write goal or force stop ---- */
        if (ret == ESP_OK) {
            esp_err_t write_ret = goal_state_write(&goal, 0 /* use default TTL */);
            if (write_ret != ESP_OK) {
                ESP_LOGW(TAG, "goal_state_write failed: %d", write_ret);
            }
            ESP_LOGI(TAG, "Goal: %s | latency: %" PRIu32 " ms", goal_kind_name(goal.kind),
                     latency_ms);
            backoff_periods = 0;
        } else {
            backoff_periods = (backoff_periods == 0) ? 1
                              : (backoff_periods < PLANNER_MAX_BACKOFF_PERIODS)
                                  ? backoff_periods * 2
                                  : PLANNER_MAX_BACKOFF_PERIODS;
            ESP_LOGW(TAG,
                     "gemini_backend_plan failed (%s) — forcing stop, backing off %" PRIu32
                     " extra period(s)",
                     esp_err_to_name(ret), backoff_periods);
            goal_state_force_stop();
        }

        /* ---- 5. Return framebuffer ---- */
        camera_return_fb(fb);

        /* ---- 6. Sleep until next period ----
         * A request slower than the period (a 21 s DNS timeout beats a 15 s
         * period) leaves the wake deadline in the past, and xTaskDelayUntil
         * then returns immediately — so the loop would spin at full tilt
         * exactly when the backend is already struggling. Resync on overrun
         * so the pacing holds. */
        if (xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS *
                                                           (1 + backoff_periods))) == pdFALSE) {
            vTaskDelay(pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
            last_wake_time = xTaskGetTickCount();
        }
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
