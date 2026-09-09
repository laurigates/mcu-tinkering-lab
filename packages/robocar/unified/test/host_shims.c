/**
 * @file host_shims.c
 * @brief Implementations behind the host-test shim headers in test/include.
 *
 * The one piece with real behaviour is the byte-buffer ring: audio_player.c's
 * accounting is only meaningful against a ring that actually accepts and
 * returns bytes. Everything else (I2S, task creation, notifications) is inert
 * on purpose — see the header comments for why the player task deliberately
 * does not run.
 */

#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/* -------------------------------------------------------------------------- */
/* Semaphores                                                                   */
/*                                                                              */
/* Always free. The host tests are single-threaded, so the contention path —     */
/* where activity_trace.c drops a counter update rather than block — is not      */
/* reachable, and a shim that simulated contention would be testing itself. The  */
/* handle is a fixed non-NULL token because callers null-check it.               */
/* -------------------------------------------------------------------------- */

static int s_mutex_token;

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return &s_mutex_token;
}

SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void)
{
    return &s_mutex_token;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait)
{
    (void)wait;
    return sem != NULL ? pdTRUE : pdFALSE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    return sem != NULL ? pdTRUE : pdFALSE;
}

BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t wait)
{
    (void)wait;
    return sem != NULL ? pdTRUE : pdFALSE;
}

BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem)
{
    return sem != NULL ? pdTRUE : pdFALSE;
}

void vSemaphoreDelete(SemaphoreHandle_t sem)
{
    (void)sem;
}

/* -------------------------------------------------------------------------- */
/* esp_err                                                                      */
/* -------------------------------------------------------------------------- */

const char *esp_err_to_name(esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return "ESP_OK";
        case ESP_ERR_INVALID_ARG:
            return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE:
            return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_NO_MEM:
            return "ESP_ERR_NO_MEM";
        case ESP_ERR_TIMEOUT:
            return "ESP_ERR_TIMEOUT";
        default:
            return "ESP_FAIL";
    }
}

/* -------------------------------------------------------------------------- */
/* Byte-buffer ring                                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t *buf;
    size_t capacity;
    size_t head;     /**< write offset */
    size_t tail;     /**< read offset */
    size_t received; /**< total bytes ever accepted */
    size_t out_len;  /**< length of the item currently checked out, 0 if none */
} fake_ring_t;

static fake_ring_t *s_last_created;

RingbufHandle_t xRingbufferCreateWithCaps(size_t size, RingbufferType_t type, uint32_t caps)
{
    (void)type;
    (void)caps;
    fake_ring_t *r = calloc(1, sizeof(*r));
    if (!r) {
        return NULL;
    }
    r->buf = malloc(size);
    if (!r->buf) {
        free(r);
        return NULL;
    }
    r->capacity = size;
    s_last_created = r;
    return r;
}

void vRingbufferDeleteWithCaps(RingbufHandle_t rb)
{
    fake_ring_t *r = rb;
    if (!r) {
        return;
    }
    free(r->buf);
    free(r);
}

BaseType_t xRingbufferSend(RingbufHandle_t rb, const void *data, size_t size, TickType_t wait)
{
    (void)wait;
    fake_ring_t *r = rb;
    if (!r || size == 0) {
        return pdTRUE;
    }
    /* No wraparound: the tests never queue more than the ring holds, and a
     * silent overwrite would corrupt the very counter under test. Refusing is
     * the honest failure, and it surfaces as a ring-full timeout exactly as the
     * device would report it. */
    if (r->head + size > r->capacity) {
        return pdFALSE;
    }
    memcpy(r->buf + r->head, data, size);
    r->head += size;
    r->received += size;
    return pdTRUE;
}

void *xRingbufferReceiveUpTo(RingbufHandle_t rb, size_t *out_size, TickType_t wait, size_t max)
{
    (void)wait;
    fake_ring_t *r = rb;
    if (!r || r->tail >= r->head) {
        if (out_size) {
            *out_size = 0;
        }
        return NULL;
    }
    const size_t avail = r->head - r->tail;
    const size_t n = (avail < max) ? avail : max;
    r->out_len = n;
    if (out_size) {
        *out_size = n;
    }
    return r->buf + r->tail;
}

void vRingbufferReturnItem(RingbufHandle_t rb, void *item)
{
    (void)item;
    fake_ring_t *r = rb;
    if (!r) {
        return;
    }
    r->tail += r->out_len;
    r->out_len = 0;
    /* Fully drained: rewind so a long test run is not bounded by capacity. */
    if (r->tail == r->head) {
        r->tail = 0;
        r->head = 0;
    }
}

size_t ringbuf_fake_bytes_received(RingbufHandle_t rb)
{
    const fake_ring_t *r = rb;
    return r ? r->received : 0;
}

RingbufHandle_t ringbuf_fake_last_created(void)
{
    return s_last_created;
}

/* -------------------------------------------------------------------------- */
/* Tasks — inert; see freertos/task.h                                           */
/* -------------------------------------------------------------------------- */

static int s_fake_task_object;

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                                   UBaseType_t prio, TaskHandle_t *out, BaseType_t core)
{
    (void)fn;
    (void)name;
    (void)stack;
    (void)arg;
    (void)prio;
    (void)core;
    if (out) {
        *out = &s_fake_task_object;
    }
    return pdPASS;
}

void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

void vTaskDelete(TaskHandle_t task)
{
    (void)task;
}

uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t wait)
{
    (void)clear_on_exit;
    (void)wait;
    return 0;
}

void xTaskNotifyGive(TaskHandle_t task)
{
    (void)task;
}

/* -------------------------------------------------------------------------- */
/* Software timers — inert; see freertos/timers.h                               */
/* -------------------------------------------------------------------------- */

static int s_fake_timer_object;

TimerHandle_t xTimerCreate(const char *name, TickType_t period, UBaseType_t auto_reload,
                           void *timer_id, TimerCallbackFunction_t callback)
{
    (void)name;
    (void)period;
    (void)auto_reload;
    (void)timer_id;
    (void)callback;
    return &s_fake_timer_object;
}

BaseType_t xTimerStart(TimerHandle_t timer, TickType_t block)
{
    (void)timer;
    (void)block;
    return pdPASS;
}

BaseType_t xTimerStop(TimerHandle_t timer, TickType_t block)
{
    (void)timer;
    (void)block;
    return pdPASS;
}

BaseType_t xTimerDelete(TimerHandle_t timer, TickType_t block)
{
    (void)timer;
    (void)block;
    return pdPASS;
}

BaseType_t xTimerChangePeriod(TimerHandle_t timer, TickType_t period, TickType_t block)
{
    (void)timer;
    (void)period;
    (void)block;
    return pdPASS;
}

BaseType_t xTimerIsTimerActive(TimerHandle_t timer)
{
    (void)timer;
    return pdFALSE;
}

void *pvTimerGetTimerID(TimerHandle_t timer)
{
    (void)timer;
    return NULL;
}

/* -------------------------------------------------------------------------- */
/* I2S — inert; see driver/i2s_std.h                                            */
/* -------------------------------------------------------------------------- */

static int s_fake_i2s_chan;

esp_err_t i2s_new_channel(const i2s_chan_config_t *cfg, i2s_chan_handle_t *tx,
                          i2s_chan_handle_t *rx)
{
    (void)cfg;
    if (tx) {
        *tx = &s_fake_i2s_chan;
    }
    if (rx) {
        *rx = NULL;
    }
    return ESP_OK;
}

esp_err_t i2s_del_channel(i2s_chan_handle_t chan)
{
    (void)chan;
    return ESP_OK;
}

esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t chan, const i2s_std_config_t *cfg)
{
    (void)chan;
    (void)cfg;
    return ESP_OK;
}

esp_err_t i2s_channel_enable(i2s_chan_handle_t chan)
{
    (void)chan;
    return ESP_OK;
}

esp_err_t i2s_channel_disable(i2s_chan_handle_t chan)
{
    (void)chan;
    return ESP_OK;
}

esp_err_t i2s_channel_write(i2s_chan_handle_t chan, const void *src, size_t size, size_t *written,
                            uint32_t timeout_ms)
{
    (void)chan;
    (void)src;
    (void)timeout_ms;
    if (written) {
        *written = size;
    }
    return ESP_OK;
}

esp_err_t i2s_channel_preload_data(i2s_chan_handle_t chan, const void *src, size_t size,
                                   size_t *loaded)
{
    (void)chan;
    (void)src;
    (void)size;
    /* Report "buffers full" immediately so preload_silence()'s do/while ends. */
    if (loaded) {
        *loaded = 0;
    }
    return ESP_OK;
}
