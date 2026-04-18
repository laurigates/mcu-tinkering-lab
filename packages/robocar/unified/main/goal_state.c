/**
 * @file goal_state.c
 * @brief Mutex-protected shared goal state between the planner and the
 *        reactive executor.
 *
 * Design notes
 * ------------
 * - No heap allocations in read/write; goal_t is a small fixed struct copied
 *   by value across the mutex boundary.
 * - Critical sections are kept as short as possible: the lock is taken,
 *   the struct is memcpy'd, the lock is released.  Logging and freshness
 *   evaluation happen *outside* the lock so they never add latency to either
 *   producer or consumer.
 * - A thin #ifdef seam replaces FreeRTOS primitives with POSIX equivalents
 *   when GOAL_STATE_HOST_TEST=1, making the module buildable and testable on
 *   any POSIX host without ESP-IDF.
 */

#include "goal_state.h"

#include <string.h>

/* =========================================================================
 * Platform abstraction
 * ========================================================================= */

#ifndef GOAL_STATE_HOST_TEST
/* ---- On-target: FreeRTOS + ESP-IDF ---- */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef SemaphoreHandle_t gs_mutex_t;

static inline esp_err_t gs_mutex_create(gs_mutex_t *m)
{
    *m = xSemaphoreCreateMutex();
    return (*m != NULL) ? ESP_OK : ESP_FAIL;
}

static inline void gs_mutex_destroy(gs_mutex_t *m)
{
    if (*m) {
        vSemaphoreDelete(*m);
        *m = NULL;
    }
}

/* 50 ms timeout — enough for any brief critical section; logs on timeout. */
#define GS_MUTEX_TIMEOUT_TICKS pdMS_TO_TICKS(50)

static inline bool gs_mutex_lock(gs_mutex_t *m)
{
    return xSemaphoreTake(*m, GS_MUTEX_TIMEOUT_TICKS) == pdTRUE;
}

static inline void gs_mutex_unlock(gs_mutex_t *m)
{
    xSemaphoreGive(*m);
}

static int64_t gs_default_now_us(void)
{
    return esp_timer_get_time();
}

#else
/* ---- Host-test: POSIX shims ---- */
#include <pthread.h>
#include <stdio.h>
#include <time.h>

typedef pthread_mutex_t gs_mutex_t;

static inline esp_err_t gs_mutex_create(gs_mutex_t *m)
{
    return (pthread_mutex_init(m, NULL) == 0) ? ESP_OK : ESP_FAIL;
}

static inline void gs_mutex_destroy(gs_mutex_t *m)
{
    pthread_mutex_destroy(m);
}

static inline bool gs_mutex_lock(gs_mutex_t *m)
{
    return pthread_mutex_lock(m) == 0;
}

static inline void gs_mutex_unlock(gs_mutex_t *m)
{
    pthread_mutex_unlock(m);
}

/* Minimal printf-based stubs so the module compiles without esp_log.h. */
#define ESP_LOGI(tag, fmt, ...) printf("[I] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[W] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[E] %s: " fmt "\n", tag, ##__VA_ARGS__)

static int64_t gs_default_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)(ts.tv_nsec / 1000);
}

#endif /* GOAL_STATE_HOST_TEST */

/* =========================================================================
 * Internal state
 * ========================================================================= */

static const char *TAG = "goal_state";

typedef struct {
    gs_mutex_t mutex;
    goal_t goal;
    int64_t written_at_us; /**< Timestamp of last write (us).           */
    uint32_t ttl_ms;       /**< TTL of the current goal (ms).           */
    bool has_been_written; /**< False until the first goal_state_write. */
    bool initialised;
} goal_state_t;

static goal_state_t g_gs;

/** Clock function pointer — defaults to gs_default_now_us, overridable. */
static int64_t (*g_now_us_fn)(void) = gs_default_now_us;

/* =========================================================================
 * Helpers
 * ========================================================================= */

/** Returns the current time in microseconds via the active clock function. */
static inline int64_t now_us(void)
{
    return g_now_us_fn();
}

/**
 * Evaluate freshness without holding the mutex.
 * Caller passes in the snapshot values already copied under the lock.
 */
static inline bool is_fresh(int64_t written_at_us, uint32_t ttl_ms)
{
    int64_t age_us = now_us() - written_at_us;
    return age_us >= 0 && age_us < (int64_t)ttl_ms * 1000LL;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t goal_state_init(void)
{
    if (g_gs.initialised) {
        ESP_LOGW(TAG, "Already initialised");
        return ESP_OK;
    }

    esp_err_t ret = gs_mutex_create(&g_gs.mutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    memset(&g_gs.goal, 0, sizeof(g_gs.goal));
    g_gs.goal.kind = GOAL_KIND_NONE;
    g_gs.written_at_us = 0;
    g_gs.ttl_ms = GOAL_STATE_DEFAULT_TTL_MS;
    g_gs.has_been_written = false;
    g_gs.initialised = true;

    ESP_LOGI(TAG, "Initialised (default TTL %u ms)", GOAL_STATE_DEFAULT_TTL_MS);
    return ESP_OK;
}

esp_err_t goal_state_write(const goal_t *goal, uint32_t ttl_ms)
{
    if (!goal) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_gs.initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t effective_ttl = (ttl_ms == 0) ? GOAL_STATE_DEFAULT_TTL_MS : ttl_ms;

    /* Snapshot the old kind outside the lock so we can log after releasing it. */
    goal_kind_t old_kind;

    if (!gs_mutex_lock(&g_gs.mutex)) {
        ESP_LOGE(TAG, "Mutex timeout in goal_state_write");
        return ESP_FAIL;
    }
    old_kind = g_gs.goal.kind;
    g_gs.goal = *goal;
    g_gs.written_at_us = now_us();
    g_gs.ttl_ms = effective_ttl;
    g_gs.has_been_written = true;
    gs_mutex_unlock(&g_gs.mutex);

    /* Log only when the goal kind changes to keep the serial log readable. */
    if (goal->kind != old_kind) {
        ESP_LOGI(TAG, "Goal kind: %d -> %d (ttl %u ms)", (int)old_kind, (int)goal->kind,
                 effective_ttl);
    }

    return ESP_OK;
}

esp_err_t goal_state_read(goal_t *out, bool *is_fresh_out)
{
    if (!out || !is_fresh_out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!g_gs.initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Snapshot under the lock — no computation, no logging. */
    int64_t written_at_us;
    uint32_t ttl_ms;
    bool has_been_written;

    if (!gs_mutex_lock(&g_gs.mutex)) {
        ESP_LOGE(TAG, "Mutex timeout in goal_state_read");
        return ESP_FAIL;
    }
    *out = g_gs.goal;
    written_at_us = g_gs.written_at_us;
    ttl_ms = g_gs.ttl_ms;
    has_been_written = g_gs.has_been_written;
    gs_mutex_unlock(&g_gs.mutex);

    /*
     * Evaluate freshness outside the lock.  Pre-first-write the flag is false
     * so we report stale without waiting on a TTL — the executor starts in the
     * safe STOP state.  This separates the sentinel from the timestamp so the
     * module works correctly with a test clock starting at 0.
     */
    bool fresh = has_been_written && is_fresh(written_at_us, ttl_ms);
    *is_fresh_out = fresh;

    if (!fresh) {
        out->kind = GOAL_KIND_STOP;
    }

    return ESP_OK;
}

esp_err_t goal_state_force_stop(void)
{
    if (!g_gs.initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    const goal_t stop = {.kind = GOAL_KIND_STOP};
    return goal_state_write(&stop, GOAL_STATE_DEFAULT_TTL_MS);
}

void goal_state_set_clock_override(int64_t (*now_us_fn)(void))
{
    g_now_us_fn = (now_us_fn != NULL) ? now_us_fn : gs_default_now_us;
}
