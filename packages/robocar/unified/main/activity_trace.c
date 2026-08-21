/**
 * @file activity_trace.c
 * @brief Camera / endpoint activity indicators. See activity_trace.h.
 */

#include "activity_trace.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_controller.h"
#include "pin_config.h"

static const char *TAG = "trace";

/** How long a success pulse stays lit, in ms. Long enough to catch by eye at a
 *  glance across a room, short enough that back-to-back TTS chunks still read as
 *  separate blinks rather than one continuous glow. */
#define TRACE_PULSE_MS 180u

/** Indicator task tick, in ms. Sets the granularity of a pulse's end, so it must
 *  be comfortably shorter than TRACE_PULSE_MS or a pulse would outlive itself by
 *  a visible margin. Nothing here is timing-critical; this task exists only so
 *  that the I2C write to the PCA9685 happens somewhere other than the planner's
 *  or the TTS task's critical path. */
#define TRACE_TICK_MS 40u

/** Bound on the counter mutex. Short and non-fatal on purpose: instrumentation
 *  that can block the path it measures changes the number it is reporting. On
 *  contention the update is dropped. */
#define TRACE_LOCK_TIMEOUT_MS 20u

#define TRACE_TASK_STACK_SIZE 3072
#define TRACE_TASK_PRIORITY                              \
    2 /* below every task it observes: an indicator must \
       * never preempt the work it is indicating. */
#define TRACE_TASK_CORE 0

/* -------------------------------------------------------------------------- */
/* Counters                                                                     */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint32_t calls;
    uint32_t ok;
    uint32_t rate_limited; /**< HTTP 429 — a pacing problem, not a fault. */
    uint32_t failed;       /**< Transport error or any other non-200. */
    uint32_t in_flight;
    uint32_t begin_ms;    /**< Start of the most recent request. */
    uint32_t last_end_ms; /**< When the most recent request returned. */
    uint32_t last_ms;     /**< Round-trip of the most recent request. */
    uint32_t max_ms;      /**< Slowest round-trip since boot/reset. */
    uint64_t total_ms;    /**< For the mean; 64-bit so a long run cannot wrap. */
    int last_status;      /**< HTTP status of the most recent request. */
    esp_err_t last_err;   /**< Its transport result. */
} endpoint_stats_t;

typedef struct {
    uint32_t captures;
    uint32_t failures;
    uint32_t last_bytes;
    uint32_t last_ms;
    uint32_t max_ms;
    uint64_t total_ms;
    uint64_t total_bytes;
    uint32_t last_capture_ms; /**< Clock at the last capture, ok or not. */
    bool last_ok;
} camera_stats_t;

static const char *const k_endpoint_name[ACTIVITY_EP_COUNT] = {
    "planner",
    "narrate",
    "tts",
    "listen",
};

static endpoint_stats_t s_endpoint[ACTIVITY_EP_COUNT];
static camera_stats_t s_camera;
static SemaphoreHandle_t s_lock;
static bool s_started;
static volatile bool s_leds_enabled = true;

/* LED intent, written by the hot paths and read by the indicator task. Kept as
 * plain scalars rather than a struct so each is a single naturally-aligned word:
 * the task can read them without the mutex, and the worst a torn read can do is
 * show one frame of the wrong colour for 40 ms. */
static volatile uint32_t s_cam_pulse_ms; /**< Clock at the last OK capture. */
static volatile bool s_cam_failed;       /**< Last capture failed. */
static volatile uint32_t s_net_pulse_ms; /**< Clock at the last completion. */
static volatile int s_net_last_status;
static volatile bool s_net_last_ok;
static volatile uint32_t s_net_in_flight; /**< Requests currently outstanding. */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/** Take the counter mutex, or report that the caller must skip its update.
 *  Returns true only when the lock is held. */
static bool trace_lock(void)
{
    if (s_lock == NULL) {
        return false;
    }
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(TRACE_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static void trace_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

/* -------------------------------------------------------------------------- */
/* Indicator task                                                               */
/* -------------------------------------------------------------------------- */

/** True while @p stamp is within TRACE_PULSE_MS of now. Unsigned difference, so
 *  it behaves across the uint32 millisecond wrap at day 49 rather than latching
 *  a pulse on for the rest of the cycle. */
static bool pulse_active(uint32_t stamp)
{
    if (stamp == 0u) {
        return false;
    }
    return (uint32_t)(now_ms() - stamp) < TRACE_PULSE_MS;
}

/** Colour for the camera LED: a failure HOLDS (so a fault that happened while
 *  nobody was watching is still on the robot), a success only pulses. */
static rgb_color_t camera_color(void)
{
    if (s_cam_failed) {
        return LED_COLOR_RED;
    }
    if (pulse_active(s_cam_pulse_ms)) {
        return LED_COLOR_WHITE;
    }
    return LED_COLOR_OFF;
}

/** Colour for the network LED. In-flight outranks everything else: a held blue
 *  is the whole point of the indicator, because a request that never returns is
 *  otherwise indistinguishable from an idle robot until its timeout expires. */
static rgb_color_t network_color(void)
{
    if (s_net_in_flight > 0u) {
        return LED_COLOR_BLUE;
    }
    if (!s_net_last_ok) {
        /* Failures hold, for the same reason camera failures do. 429 is called
         * out in its own colour because "you are asking too fast" and "the call
         * broke" want different responses from whoever is watching. */
        return (s_net_last_status == 429) ? LED_COLOR_YELLOW : LED_COLOR_RED;
    }
    if (pulse_active(s_net_pulse_ms)) {
        return LED_COLOR_GREEN;
    }
    return LED_COLOR_OFF;
}

static bool color_equal(const rgb_color_t *a, const rgb_color_t *b)
{
    return a->red == b->red && a->green == b->green && a->blue == b->blue;
}

static void activity_trace_task(void *arg)
{
    (void)arg;

    /* Seeded to a colour that cannot be the first desired one, so the first tick
     * always writes and the LEDs are never left showing whatever the boot
     * indication put there. */
    rgb_color_t shown_cam = {0xFF, 0xFF, 0xFF};
    rgb_color_t shown_net = {0xFF, 0xFF, 0xFF};
    bool leds_were_enabled = true;

    ESP_LOGI(TAG, "Activity indicators running (left = camera, right = endpoints)");

    for (;;) {
        const bool enabled = s_leds_enabled;

        if (!enabled) {
            /* Release both LEDs once on the falling edge, then stop touching the
             * I2C bus entirely — "off" has to mean off, not "written black at
             * 25 Hz forever". */
            if (leds_were_enabled) {
                led_turn_off_all();
                shown_cam = (rgb_color_t){0xFF, 0xFF, 0xFF};
                shown_net = (rgb_color_t){0xFF, 0xFF, 0xFF};
                leds_were_enabled = false;
            }
            vTaskDelay(pdMS_TO_TICKS(TRACE_TICK_MS));
            continue;
        }
        leds_were_enabled = true;

        /* Write only on a change. At 25 Hz an unconditional write would put a
         * steady PCA9685 transaction load on the shared I2C bus for no benefit,
         * competing with the motor and servo controllers that share it. */
        const rgb_color_t want_cam = camera_color();
        if (!color_equal(&want_cam, &shown_cam)) {
            if (led_set_color(LED_LEFT, &want_cam) == ESP_OK) {
                shown_cam = want_cam;
            }
        }

        const rgb_color_t want_net = network_color();
        if (!color_equal(&want_net, &shown_net)) {
            if (led_set_color(LED_RIGHT, &want_net) == ESP_OK) {
                shown_net = want_net;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TRACE_TICK_MS));
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t activity_trace_init(void)
{
    if (s_started) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "mutex allocation failed");
        return ESP_ERR_NO_MEM;
    }

    memset(s_endpoint, 0, sizeof(s_endpoint));
    memset(&s_camera, 0, sizeof(s_camera));
    /* Nothing has failed yet, so the network LED must not open showing red. */
    s_net_last_ok = true;

    const BaseType_t ok =
        xTaskCreatePinnedToCore(activity_trace_task, "activity_trace", TRACE_TASK_STACK_SIZE, NULL,
                                TRACE_TASK_PRIORITY, NULL, TRACE_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create indicator task");
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}

void activity_trace_camera(bool ok, size_t bytes, uint32_t elapsed_ms)
{
    const uint32_t t = now_ms();

    /* LED intent first, and outside the lock: it is what someone watching the
     * robot sees, and it must not be the thing that gets dropped on contention. */
    s_cam_failed = !ok;
    if (ok) {
        s_cam_pulse_ms = (t == 0u) ? 1u : t; /* 0 is the "never" sentinel */
    }

    if (!trace_lock()) {
        return;
    }
    s_camera.last_ok = ok;
    s_camera.last_capture_ms = t;
    if (ok) {
        s_camera.captures++;
        s_camera.last_bytes = (uint32_t)bytes;
        s_camera.last_ms = elapsed_ms;
        s_camera.total_ms += elapsed_ms;
        s_camera.total_bytes += bytes;
        if (elapsed_ms > s_camera.max_ms) {
            s_camera.max_ms = elapsed_ms;
        }
    } else {
        s_camera.failures++;
    }
    const uint32_t captures = s_camera.captures;
    const uint32_t failures = s_camera.failures;
    trace_unlock();

    if (ok) {
        ESP_LOGI(TAG, "cam  #%u  %u bytes  %u ms", (unsigned)captures, (unsigned)bytes,
                 (unsigned)elapsed_ms);
    } else {
        ESP_LOGW(TAG, "cam  FAILED  (%u failures, %u ok)", (unsigned)failures, (unsigned)captures);
    }
}

void activity_trace_http_begin(activity_endpoint_t ep)
{
    if (ep >= ACTIVITY_EP_COUNT) {
        return;
    }
    const uint32_t t = now_ms();

    if (trace_lock()) {
        s_endpoint[ep].calls++;
        s_endpoint[ep].in_flight++;
        s_endpoint[ep].begin_ms = t;
        trace_unlock();
    }

    /* Counted outside the lock as well, so the LED still shows a request in
     * flight even on the tick where the counter update was dropped. */
    s_net_in_flight++;

    /* The `>` half of a matched pair. A `>` with no `<` after it in a capture is
     * a hung request, and is the reason this is logged at all rather than only
     * on completion. */
    ESP_LOGI(TAG, "http > %s", k_endpoint_name[ep]);
}

void activity_trace_http_end(activity_endpoint_t ep, esp_err_t err, int status, uint32_t elapsed_ms)
{
    if (ep >= ACTIVITY_EP_COUNT) {
        return;
    }
    const uint32_t t = now_ms();
    const bool ok = (err == ESP_OK && status == 200);

    if (s_net_in_flight > 0u) {
        s_net_in_flight--;
    }
    s_net_last_ok = ok;
    s_net_last_status = status;
    s_net_pulse_ms = (t == 0u) ? 1u : t;

    if (trace_lock()) {
        endpoint_stats_t *st = &s_endpoint[ep];
        if (st->in_flight > 0u) {
            st->in_flight--;
        }
        st->last_end_ms = t;
        st->last_ms = elapsed_ms;
        st->last_status = status;
        st->last_err = err;
        st->total_ms += elapsed_ms;
        if (elapsed_ms > st->max_ms) {
            st->max_ms = elapsed_ms;
        }
        if (ok) {
            st->ok++;
        } else if (status == 429) {
            st->rate_limited++;
        } else {
            st->failed++;
        }
        trace_unlock();
    }

    if (ok) {
        ESP_LOGI(TAG, "http < %s  200  %u ms", k_endpoint_name[ep], (unsigned)elapsed_ms);
    } else if (status == 429) {
        /* Not an error: the free tier is a per-model RPM cap and the retryDelay
         * grows while the client keeps asking, so this is a signal to slow down.
         * See .claude/rules/gemini-api.md §4. */
        ESP_LOGW(TAG, "http < %s  429 RATE LIMITED  %u ms — loop is outrunning the quota",
                 k_endpoint_name[ep], (unsigned)elapsed_ms);
    } else {
        ESP_LOGW(TAG, "http < %s  status=%d err=%s  %u ms", k_endpoint_name[ep], status,
                 esp_err_to_name(err), (unsigned)elapsed_ms);
    }
}

void activity_trace_report(void)
{
    endpoint_stats_t ep[ACTIVITY_EP_COUNT];
    camera_stats_t cam;

    if (!trace_lock()) {
        printf("trace: counters busy, try again\n");
        return;
    }
    memcpy(ep, s_endpoint, sizeof(ep));
    cam = s_camera;
    trace_unlock();

    const uint32_t t = now_ms();

    printf("trace: leds=%s\n", s_leds_enabled ? "on" : "off");

    printf("  camera: %u ok, %u failed", (unsigned)cam.captures, (unsigned)cam.failures);
    if (cam.captures > 0u) {
        printf("   last %u bytes in %u ms   mean %u ms   max %u ms", (unsigned)cam.last_bytes,
               (unsigned)cam.last_ms, (unsigned)(cam.total_ms / cam.captures),
               (unsigned)cam.max_ms);
    }
    printf("\n");
    if (cam.last_capture_ms == 0u) {
        printf("          no capture yet\n");
    } else {
        printf("          last attempt %u ms ago (%s)\n", (unsigned)(t - cam.last_capture_ms),
               cam.last_ok ? "ok" : "FAILED");
    }

    printf("  %-8s %6s %5s %5s %5s  %8s %8s %8s  %s\n", "endpoint", "calls", "ok", "429", "fail",
           "last ms", "mean ms", "max ms", "state");
    for (int i = 0; i < ACTIVITY_EP_COUNT; ++i) {
        const endpoint_stats_t *s = &ep[i];
        const uint32_t done = s->ok + s->rate_limited + s->failed;

        char state[48];
        if (s->in_flight > 0u) {
            /* The number that matters when something is wedged: how long the
             * outstanding request has been outstanding. A value climbing past
             * the endpoint's timeout means the timeout is not firing. */
            snprintf(state, sizeof(state), "IN FLIGHT %u ms", (unsigned)(t - s->begin_ms));
        } else if (done == 0u) {
            snprintf(state, sizeof(state), "-");
        } else {
            snprintf(state, sizeof(state), "%d %s %u ms ago", s->last_status,
                     esp_err_to_name(s->last_err), (unsigned)(t - s->last_end_ms));
        }

        printf("  %-8s %6u %5u %5u %5u  %8u %8u %8u  %s\n", k_endpoint_name[i], (unsigned)s->calls,
               (unsigned)s->ok, (unsigned)s->rate_limited, (unsigned)s->failed,
               (unsigned)s->last_ms, (unsigned)(done > 0u ? (uint32_t)(s->total_ms / done) : 0u),
               (unsigned)s->max_ms, state);
    }
    printf("  usage: trace | trace led on|off | trace reset\n");
}

void activity_trace_reset(void)
{
    if (!trace_lock()) {
        printf("trace: counters busy, not reset\n");
        return;
    }
    memset(s_endpoint, 0, sizeof(s_endpoint));
    memset(&s_camera, 0, sizeof(s_camera));
    trace_unlock();

    /* Clear the held failure colours too — a reset that left the robot showing
     * red for a fault it no longer counts would be lying. In-flight is NOT
     * cleared: a request that is genuinely outstanding stays outstanding, and
     * its end() will decrement this. */
    s_cam_failed = false;
    s_cam_pulse_ms = 0u;
    s_net_last_ok = true;
    s_net_last_status = 0;
    s_net_pulse_ms = 0u;
}

void activity_trace_set_leds(bool enabled)
{
    s_leds_enabled = enabled;
}

bool activity_trace_leds_enabled(void)
{
    return s_leds_enabled;
}
