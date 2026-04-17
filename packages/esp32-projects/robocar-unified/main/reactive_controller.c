/**
 * @file reactive_controller.c
 * @brief 30 Hz reactive motor executor.
 *
 * See reactive_controller.h for architecture overview.
 *
 * Skid-steer mix (GOAL_KIND_DRIVE)
 * ---------------------------------
 * heading_deg is treated as a turn bias, not an absolute compass heading (we
 * have no IMU or odometry).  Positive heading_deg turns right; negative turns
 * left.  The mix is:
 *
 *   base   = speed_pct * 255 / 100
 *   bias   = |heading_deg| * base / 180   (linear, max 100 % differential)
 *   right  = (heading_deg > 0) ? base - bias : base
 *   left   = (heading_deg < 0) ? base - bias : base
 *   dir    = 1 (forward)
 *
 * This is intentionally simple.  A full heading-hold loop requires odometry
 * or a compass — neither is available in this phase.  The planner is expected
 * to keep re-issuing updated heading_deg goals as the scene changes.
 *
 * Visual-servo P controller (GOAL_KIND_TRACK)
 * -------------------------------------------
 *   cx    = (xmin + xmax) / 2          centre of bounding box, 0..1000
 *   error = cx - 500                   signed error, -500..500
 *   turn  = TRACK_KP * error           turn bias, ±75 at max error with Kp=0.15
 *   base  = max_speed_pct * 255 / 100
 *   right = base - turn   (clamped 0..255)
 *   left  = base + turn   (clamped 0..255)
 *   dir   = 1 (always creep forward while tracking)
 *
 * Obstacle reflex
 * ---------------
 * If smoothed distance < STOP_THRESHOLD_CM, motors are stopped and
 * reflex_active is latched.  The latch clears automatically once distance
 * exceeds STOP_THRESHOLD_CM again; no planner intervention is required.
 * The reflex is logged once on entry and once on exit to avoid log spam.
 *
 * Stack high-water-mark
 * ---------------------
 * Printed every 5 s via uxTaskGetStackHighWaterMark for gate G2 verification.
 */

#include "reactive_controller.h"

#ifndef REACTIVE_CONTROLLER_HOST_TEST

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "goal_state.h"
#include "motor_controller.h"
#include "ultrasonic.h"

static const char *TAG = "reactive_ctrl";

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;
static bool s_running = false;

/* Telemetry — written by executor task, read by any task under spinlock. */
static portMUX_TYPE s_telem_mux = portMUX_INITIALIZER_UNLOCKED;
static reactive_telemetry_t s_telemetry = {
    .distance_cm = ULTRASONIC_DIST_ERROR,
    .reflex_active = false,
    .loop_hz = 0,
    .stack_high_water = 0,
};

/* -------------------------------------------------------------------------
 * Skid-steer helpers
 * ---------------------------------------------------------------------- */

/** Clamp an integer to [0, 255]. */
static inline uint8_t clamp_u8(int32_t v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return (uint8_t)v;
}

/**
 * @brief Apply skid-steer mix for a drive goal.
 *
 * NOTE: motor_set_individual() takes (left_speed, right_speed, left_dir,
 * right_dir).  Direction 1 = forward.
 *
 * Shortcut noted: we ignore drive.distance_cm — the robot drives until the
 * planner emits a new goal.  Closed-loop distance requires odometry, which is
 * not available in this phase.
 */
static void execute_drive(const goal_t *goal)
{
    int16_t heading = goal->params.drive.heading_deg;
    uint8_t speed_pct = goal->params.drive.speed_pct;

    int32_t base = (int32_t)speed_pct * 255 / 100;
    int32_t bias = (int32_t)abs(heading) * base / 180;

    int32_t left, right;
    if (heading > 0) {
        /* Turn right: slow down right wheel */
        right = base - bias;
        left = base;
    } else if (heading < 0) {
        /* Turn left: slow down left wheel */
        left = base - bias;
        right = base;
    } else {
        left = base;
        right = base;
    }

    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

/**
 * @brief Apply proportional visual servo for a track goal.
 *
 * Kp = TRACK_KP (default 0.15).  With a 500-unit half-range and base speed
 * of 128 (≈50 %), the maximum turn bias is ±75 counts — a 59 % differential
 * that produces a visible arc without spinning in place.
 *
 * To tune: increase TRACK_KP for faster centering; decrease if the robot
 * oscillates left-right.  A derivative term can be added later once a
 * velocity estimate is available from odometry.
 */
static void execute_track(const goal_t *goal)
{
    uint16_t cx = (goal->params.track.xmin + goal->params.track.xmax) / 2u;
    int32_t error = (int32_t)cx - 500;

    float turn = TRACK_KP * (float)error;
    int32_t base = (int32_t)goal->params.track.max_speed_pct * 255 / 100;

    /* Positive error (target right of centre) → right wheel slower */
    int32_t left = base + (int32_t)turn;
    int32_t right = base - (int32_t)turn;

    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

/**
 * @brief Apply in-place rotation.
 *
 * Direction is determined by the sign of angle_deg.  Magnitude is ignored —
 * we have no IMU to close the angle loop.  The planner must emit a stop/drive
 * goal when the desired heading is reached (e.g. by inspecting the next camera
 * frame).
 *
 * Rotation speed is fixed at ~60 % to give the planner time to react.
 */
static void execute_rotate(const goal_t *goal)
{
    const uint8_t rot_speed = 153; /* ~60 % of 255 */
    if (goal->params.rotate.angle_deg >= 0) {
        motor_rotate_cw(rot_speed);
    } else {
        motor_rotate_ccw(rot_speed);
    }
}

/* -------------------------------------------------------------------------
 * Running-mean distance filter
 * ---------------------------------------------------------------------- */

static uint16_t s_dist_buf[ULTRASONIC_SMOOTH_N];
static uint8_t s_dist_idx = 0;
static bool s_dist_buf_full = false;

static uint16_t smooth_distance(uint16_t raw)
{
    /* Treat error values as max range so a single bad reading does not
     * erroneously trigger the reflex. */
    uint16_t clamped = (raw == ULTRASONIC_DIST_ERROR) ? ULTRASONIC_MAX_RANGE_CM : raw;

    s_dist_buf[s_dist_idx] = clamped;
    s_dist_idx = (s_dist_idx + 1) % ULTRASONIC_SMOOTH_N;
    if (s_dist_idx == 0)
        s_dist_buf_full = true;

    uint8_t n = s_dist_buf_full ? ULTRASONIC_SMOOTH_N : s_dist_idx;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += s_dist_buf[i];
    }
    return (uint16_t)(sum / n);
}

/* -------------------------------------------------------------------------
 * Executor task
 * ---------------------------------------------------------------------- */

static void reactive_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Reactive executor started (Core %d, %u Hz target)", xPortGetCoreID(),
             1000U / REACTIVE_LOOP_PERIOD_MS);

    TickType_t last_wake = xTaskGetTickCount();

    bool reflex_was_active = false;

    /* Hz measurement state */
    uint32_t loop_count = 0;
    int64_t hz_window_start = esp_timer_get_time();

    /* Stack HWM report interval */
    int64_t hwm_last_report = esp_timer_get_time();

    while (s_running) {
        /* ---- 1. Measure distance ---- */
        uint16_t raw_dist = ULTRASONIC_DIST_ERROR;
        ultrasonic_measure(&raw_dist);
        uint16_t dist = smooth_distance(raw_dist);

        /* ---- 2. Obstacle reflex ---- */
        bool reflex = (dist < STOP_THRESHOLD_CM);

        if (reflex && !reflex_was_active) {
            ESP_LOGW(TAG, "Obstacle reflex ACTIVE — distance %u cm (threshold %u cm)", dist,
                     STOP_THRESHOLD_CM);
        } else if (!reflex && reflex_was_active) {
            ESP_LOGI(TAG, "Obstacle reflex CLEARED — distance %u cm", dist);
        }
        reflex_was_active = reflex;

        if (reflex) {
            motor_stop();
            goto update_telemetry;
        }

        /* ---- 3. Execute goal ---- */
        {
            goal_t goal;
            bool is_fresh;
            if (goal_state_read(&goal, &is_fresh) != ESP_OK) {
                motor_stop();
                goto update_telemetry;
            }

            if (!is_fresh || goal.kind == GOAL_KIND_STOP || goal.kind == GOAL_KIND_NONE) {
                motor_stop();
            } else {
                switch (goal.kind) {
                    case GOAL_KIND_DRIVE:
                        execute_drive(&goal);
                        break;
                    case GOAL_KIND_TRACK:
                        execute_track(&goal);
                        break;
                    case GOAL_KIND_ROTATE:
                        execute_rotate(&goal);
                        break;
                    default:
                        motor_stop();
                        break;
                }
            }
        }

    update_telemetry:
        /* ---- 4. Telemetry ---- */
        loop_count++;
        int64_t now = esp_timer_get_time();

        uint32_t measured_hz = 0;
        if (now - hz_window_start >= 1000000LL) {
            measured_hz = loop_count;
            loop_count = 0;
            hz_window_start = now;
        }

        uint32_t hwm = 0;
        if (now - hwm_last_report >= 5000000LL) {
            hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Stack high-water-mark: %lu bytes remaining", (unsigned long)hwm);
            hwm_last_report = now;
        } else {
            hwm = uxTaskGetStackHighWaterMark(NULL);
        }

        portENTER_CRITICAL(&s_telem_mux);
        s_telemetry.distance_cm = dist;
        s_telemetry.reflex_active = reflex;
        if (measured_hz > 0) {
            s_telemetry.loop_hz = measured_hz;
        }
        s_telemetry.stack_high_water = hwm;
        portEXIT_CRITICAL(&s_telem_mux);

        /* ---- 5. Wait for next period ---- */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(REACTIVE_LOOP_PERIOD_MS));
    }

    motor_stop();
    ESP_LOGI(TAG, "Reactive executor stopping");
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t reactive_controller_init(void)
{
    if (s_running) {
        return ESP_OK; /* idempotent */
    }

    esp_err_t ret = ultrasonic_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ultrasonic_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Reset smoothing buffer */
    memset(s_dist_buf, 0, sizeof(s_dist_buf));
    s_dist_idx = 0;
    s_dist_buf_full = false;

    s_running = true;

    BaseType_t created =
        xTaskCreatePinnedToCore(reactive_task, "reactive_ctrl", REACTIVE_TASK_STACK_SIZE, NULL,
                                REACTIVE_TASK_PRIORITY, &s_task_handle, REACTIVE_TASK_CORE);

    if (created != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "Failed to create reactive controller task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Reactive controller started (stack=%u, prio=%u, core=%u)",
             REACTIVE_TASK_STACK_SIZE, REACTIVE_TASK_PRIORITY, REACTIVE_TASK_CORE);
    return ESP_OK;
}

esp_err_t reactive_controller_stop(void)
{
    if (!s_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running = false;

    /* Give the task time to exit its loop and call vTaskDelete(NULL). */
    vTaskDelay(pdMS_TO_TICKS(REACTIVE_LOOP_PERIOD_MS * 3));

    /* If the task handle is still valid (task did not self-delete yet), force
     * delete it.  This is a last resort; normally the task exits cleanly. */
    if (s_task_handle != NULL) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }

    motor_stop();
    ESP_LOGI(TAG, "Reactive controller stopped");
    return ESP_OK;
}

esp_err_t reactive_controller_get_telemetry(reactive_telemetry_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_telem_mux);
    *out = s_telemetry;
    portEXIT_CRITICAL(&s_telem_mux);

    return ESP_OK;
}

/* =========================================================================
 * Host-test build — stub implementation
 * =========================================================================
 *
 * The host-test build links against stub versions of goal_state, ultrasonic,
 * and motor_controller (provided by Agent E's test harness).  This file
 * provides the reactive_controller API surface so the decision logic
 * (execute_drive, execute_track, execute_rotate, smooth_distance, reflex latch)
 * can be called directly from host tests without any FreeRTOS or ESP-IDF
 * symbols.
 *
 * The task creation functions become no-ops; the test harness calls the
 * internal helpers directly via thin wrappers exposed in the test build.
 * ========================================================================= */

#else /* REACTIVE_CONTROLLER_HOST_TEST */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "goal_state.h"
#include "motor_controller.h"
#include "ultrasonic.h"

/* Stub telemetry */
static reactive_telemetry_t s_telemetry = {0};
static bool s_running = false;

/* --- Smoothing buffer (same logic as target) --- */
static uint16_t s_dist_buf[ULTRASONIC_SMOOTH_N];
static uint8_t s_dist_idx = 0;
static bool s_dist_buf_full = false;

static uint16_t smooth_distance(uint16_t raw)
{
    uint16_t clamped = (raw == ULTRASONIC_DIST_ERROR) ? ULTRASONIC_MAX_RANGE_CM : raw;
    s_dist_buf[s_dist_idx] = clamped;
    s_dist_idx = (s_dist_idx + 1) % ULTRASONIC_SMOOTH_N;
    if (s_dist_idx == 0)
        s_dist_buf_full = true;
    uint8_t n = s_dist_buf_full ? ULTRASONIC_SMOOTH_N : s_dist_idx;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++)
        sum += s_dist_buf[i];
    return (uint16_t)(sum / n);
}

static inline uint8_t clamp_u8(int32_t v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return (uint8_t)v;
}

static void execute_drive(const goal_t *goal)
{
    int16_t heading = goal->params.drive.heading_deg;
    uint8_t speed_pct = goal->params.drive.speed_pct;
    int32_t base = (int32_t)speed_pct * 255 / 100;
    int32_t bias = (int32_t)abs(heading) * base / 180;
    int32_t left, right;
    if (heading > 0) {
        right = base - bias;
        left = base;
    } else if (heading < 0) {
        left = base - bias;
        right = base;
    } else {
        left = base;
        right = base;
    }
    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

static void execute_track(const goal_t *goal)
{
    uint16_t cx = (goal->params.track.xmin + goal->params.track.xmax) / 2u;
    int32_t error = (int32_t)cx - 500;
    float turn = TRACK_KP * (float)error;
    int32_t base = (int32_t)goal->params.track.max_speed_pct * 255 / 100;
    int32_t left = base + (int32_t)turn;
    int32_t right = base - (int32_t)turn;
    motor_set_individual(clamp_u8(left), clamp_u8(right), 1, 1);
}

static void execute_rotate(const goal_t *goal)
{
    const uint8_t rot_speed = 153;
    if (goal->params.rotate.angle_deg >= 0) {
        motor_rotate_cw(rot_speed);
    } else {
        motor_rotate_ccw(rot_speed);
    }
}

/**
 * @brief Single executor tick — exposed for host tests.
 *
 * Mirrors the loop body of reactive_task() without FreeRTOS timing.  Tests
 * call this directly to exercise one decision cycle with controlled inputs
 * injected via ultrasonic_test_set_distance() and goal_state_write().
 */
void reactive_controller_tick_for_test(void)
{
    uint16_t raw_dist = ULTRASONIC_DIST_ERROR;
    ultrasonic_measure(&raw_dist);
    uint16_t dist = smooth_distance(raw_dist);

    bool reflex = (dist < STOP_THRESHOLD_CM);

    if (reflex) {
        motor_stop();
        s_telemetry.distance_cm = dist;
        s_telemetry.reflex_active = true;
        return;
    }

    goal_t goal;
    bool is_fresh;
    if (goal_state_read(&goal, &is_fresh) != ESP_OK) {
        motor_stop();
        s_telemetry.distance_cm = dist;
        s_telemetry.reflex_active = false;
        return;
    }

    if (!is_fresh || goal.kind == GOAL_KIND_STOP || goal.kind == GOAL_KIND_NONE) {
        motor_stop();
    } else {
        switch (goal.kind) {
            case GOAL_KIND_DRIVE:
                execute_drive(&goal);
                break;
            case GOAL_KIND_TRACK:
                execute_track(&goal);
                break;
            case GOAL_KIND_ROTATE:
                execute_rotate(&goal);
                break;
            default:
                motor_stop();
                break;
        }
    }

    s_telemetry.distance_cm = dist;
    s_telemetry.reflex_active = false;
}

esp_err_t reactive_controller_init(void)
{
    if (s_running)
        return ESP_OK;
    memset(s_dist_buf, 0, sizeof(s_dist_buf));
    s_dist_idx = 0;
    s_dist_buf_full = false;
    s_running = true;
    ultrasonic_init();
    return ESP_OK;
}

esp_err_t reactive_controller_stop(void)
{
    if (!s_running)
        return ESP_ERR_INVALID_STATE;
    s_running = false;
    motor_stop();
    return ESP_OK;
}

esp_err_t reactive_controller_get_telemetry(reactive_telemetry_t *out)
{
    if (out == NULL)
        return ESP_ERR_INVALID_ARG;
    *out = s_telemetry;
    return ESP_OK;
}

#endif /* REACTIVE_CONTROLLER_HOST_TEST */
