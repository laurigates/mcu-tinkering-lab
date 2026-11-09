/**
 * @file event_report.c
 * @brief Error events to GlitchTip over MQTT (ADR-033) — ESP-IDF half.
 *
 * See event_report.h. The JSON and the ring logic are pure C in
 * event_report_core.c, host-tested in test/test_event_report_core.c.
 *
 * The breadcrumb ring lives in RTC slow memory (RTC_NOINIT_ATTR): a panic, a
 * watchdog reset or esp_restart() leaves it alone, power-on leaves garbage
 * that the magic word rejects. Whether a brownout reset keeps it is measured,
 * not assumed (ADR-033); `report` prints whether the last boot's ring
 * survived.
 *
 * The write counter is a DRAM atomic, not a field in RTC memory, because
 * atomic read-modify-write is only guaranteed on internal SRAM; the ring's
 * own `next` is a plain store each writer makes after its copy.
 *
 * The event id and the rest of the JSON are made on the publishing task at
 * send time, not when the event is queued: at boot the RF is still off and
 * esp_random() has not got its entropy source yet, and two boots drawing the
 * same id would make GlitchTip drop the second event as a duplicate.
 */

#include "event_report.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "config.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "event_report_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_logger.h"
#include "note_gate.h"
#include "ota_manager.h"

static const char *TAG = "event_report";

typedef enum {
    PENDING_RESET,
    PENDING_TEST,
    PENDING_NOTE,
} pending_kind_t;

/** An event_report_note_t copied into storage the queue owns (PSRAM). */
typedef struct {
    char trigger[16];
    char kind[16];
    char persona[24];
    char text[NOTE_TEXT_MAX + 1];
    uint32_t reflex_trips;
    uint32_t scene_score;
    uint32_t still_score;
    uint32_t plan_requests;
    char goals[EVENT_REPORT_NOTE_GOALS][40];
    size_t goal_count;
} note_payload_t;

/** What the queue carries; the task turns it into JSON when MQTT is up. */
typedef struct {
    pending_kind_t kind;
    char reason[16];
    const char *crumbs_state; /**< reset only: the "breadcrumbs" tag */
    int64_t uptime_ms;
    event_crumbs_t *crumbs; /**< owned: freed after publishing; may be NULL */
    note_payload_t *note;   /**< owned, PENDING_NOTE only */
} pending_t;

static RTC_NOINIT_ATTR event_crumbs_t s_rtc_crumbs;

static atomic_bool s_crumbs_ready;
static atomic_uint_fast32_t s_seq;

/* The last boot's ring, copied to PSRAM by event_report_boot_capture(). Handed
 * to the reset event, which frees it. */
static event_crumbs_t *s_boot_crumbs;
static size_t s_boot_crumb_count;
static bool s_boot_crumbs_present;
static bool s_reset_sent;

static QueueHandle_t s_queue;
static atomic_uint_fast32_t s_queued;
static atomic_uint_fast32_t s_sent;
static atomic_uint_fast32_t s_dropped;

void event_report_boot_capture(void)
{
    if (atomic_load(&s_crumbs_ready)) {
        return;
    }
    if (event_crumbs_adopt(&s_rtc_crumbs)) {
        s_boot_crumbs_present = true;
        s_boot_crumbs = heap_caps_malloc(sizeof(*s_boot_crumbs), MALLOC_CAP_SPIRAM);
        if (s_boot_crumbs != NULL) {
            memcpy(s_boot_crumbs, &s_rtc_crumbs, sizeof(*s_boot_crumbs));
            const char *lines[EVENT_CRUMBS_LINES];
            s_boot_crumb_count = event_crumbs_collect(s_boot_crumbs, lines, EVENT_CRUMBS_LINES);
        }
    }
    event_crumbs_reset(&s_rtc_crumbs);
    atomic_store(&s_seq, 0);
    atomic_store(&s_crumbs_ready, true);
}

void event_report_breadcrumb(const char *line, size_t len)
{
    if (!atomic_load(&s_crumbs_ready) || xPortInIsrContext()) {
        return;
    }
    const uint32_t seq = (uint32_t)atomic_fetch_add(&s_seq, 1);
    (void)event_crumbs_write(&s_rtc_crumbs, seq, line, len);
}

static void fill_common(event_report_t *ev, const char *event_id, const char *release,
                        const pending_t *p)
{
    ev->event_id = event_id;
    ev->timestamp = 0; /* no SNTP on the robot: the bridge stamps receive time */
    ev->source = "firmware";
    ev->release = release;
    ev->tags[ev->tag_count++] = (event_report_tag_t){"build_sha", ota_manager_get_build_sha()};
    ev->tags[ev->tag_count++] = (event_report_tag_t){"version", ota_manager_get_version()};
    ev->tags[ev->tag_count++] = (event_report_tag_t){"device", MQTT_CLIENT_ID};
    ev->extras[ev->extra_count++] = (event_report_extra_t){"uptime_ms", p->uptime_ms};
    ev->extras[ev->extra_count++] = (event_report_extra_t){
        "free_internal", (int64_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL)};
}

/** The planner-note half of render_pending(): @p ev already holds the common
 *  fields; @p goals must outlive the render. */
static void fill_note(event_report_t *ev, const note_payload_t *n, const char **goals)
{
    ev->source = "planner"; /* the bridge routes it to robocar-ai */
    ev->level = strcmp(n->kind, "difficulty") == 0 ? "warning" : "info";
    ev->message = n->text;
    ev->fingerprint[ev->fingerprint_count++] = n->trigger;
    ev->fingerprint[ev->fingerprint_count++] = n->kind;
    ev->tags[ev->tag_count++] = (event_report_tag_t){"trigger", n->trigger};
    ev->tags[ev->tag_count++] = (event_report_tag_t){"kind", n->kind};
    ev->tags[ev->tag_count++] = (event_report_tag_t){"persona", n->persona};
    ev->extras[ev->extra_count++] = (event_report_extra_t){"reflex_trips", n->reflex_trips};
    ev->extras[ev->extra_count++] = (event_report_extra_t){"scene_score", n->scene_score};
    ev->extras[ev->extra_count++] = (event_report_extra_t){"still_score", n->still_score};
    ev->extras[ev->extra_count++] = (event_report_extra_t){"plan_requests", n->plan_requests};
    for (size_t i = 0; i < n->goal_count; i++) {
        goals[i] = n->goals[i];
    }
    ev->breadcrumbs = goals;
    ev->breadcrumb_count = n->goal_count;
    ev->breadcrumb_category = "goal";
}

/** Render @p p as JSON, or NULL on allocation failure. */
static char *render_pending(const pending_t *p)
{
    uint32_t words[4];
    for (size_t i = 0; i < 4; i++) {
        words[i] = esp_random();
    }
    char event_id[EVENT_REPORT_ID_LEN + 1];
    event_report_format_id(words, event_id);

    char release[48];
    snprintf(release, sizeof(release), "robocar-unified@%s", ota_manager_get_version());

    const char *lines[EVENT_CRUMBS_LINES];
    const size_t line_count =
        p->crumbs != NULL ? event_crumbs_collect(p->crumbs, lines, EVENT_CRUMBS_LINES) : 0;

    event_report_t ev = {0};
    fill_common(&ev, event_id, release, p);
    ev.breadcrumbs = lines;
    ev.breadcrumb_count = line_count;

    char message[48];
    const char *goals[EVENT_REPORT_NOTE_GOALS];
    if (p->kind == PENDING_NOTE && p->note != NULL) {
        fill_note(&ev, p->note, goals);
    } else if (p->kind == PENDING_RESET) {
        snprintf(message, sizeof(message), "Reset: %s", p->reason);
        ev.level = "error";
        ev.message = message;
        ev.fingerprint[ev.fingerprint_count++] = "reset";
        ev.fingerprint[ev.fingerprint_count++] = p->reason;
        ev.tags[ev.tag_count++] = (event_report_tag_t){"reset_reason", p->reason};
        ev.tags[ev.tag_count++] = (event_report_tag_t){"breadcrumbs", p->crumbs_state};
    } else {
        ev.level = "info";
        ev.message = "report test";
        ev.fingerprint[ev.fingerprint_count++] = "report-test";
    }
    return event_report_render(&ev);
}

static void publish_task(void *arg)
{
    (void)arg;
    for (;;) {
        pending_t p;
        if (xQueuePeek(s_queue, &p, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!mqtt_logger_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(EVENT_REPORT_RETRY_MS));
            continue;
        }
        char *json = render_pending(&p);
        if (json == NULL) {
            ESP_LOGW(TAG, "out of memory rendering an event; retrying");
            vTaskDelay(pdMS_TO_TICKS(EVENT_REPORT_RETRY_MS));
            continue;
        }
        /* QoS 1: the MQTT client keeps the message in its outbox and resends it
         * across a reconnect until the broker acknowledges it. */
        const esp_err_t err = mqtt_logger_publish(EVENT_REPORT_TOPIC, json, 1, false);
        cJSON_free(json);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(EVENT_REPORT_RETRY_MS));
            continue;
        }
        (void)xQueueReceive(s_queue, &p, 0);
        heap_caps_free(p.crumbs);
        heap_caps_free(p.note);
        atomic_fetch_add(&s_sent, 1);
        ESP_LOGI(TAG, "event published on %s", EVENT_REPORT_TOPIC);
    }
}

esp_err_t event_report_start(void)
{
    if (s_queue != NULL) {
        return ESP_OK;
    }
    s_queue = xQueueCreate(EVENT_REPORT_QUEUE_DEPTH, sizeof(pending_t));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(publish_task, "event_report", EVENT_REPORT_TASK_STACK_SIZE, NULL,
                                EVENT_REPORT_TASK_PRIORITY, NULL,
                                EVENT_REPORT_TASK_CORE) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t enqueue(pending_t *p)
{
    if (xQueueSend(s_queue, p, 0) != pdTRUE) {
        atomic_fetch_add(&s_dropped, 1);
        return ESP_ERR_NO_MEM;
    }
    atomic_fetch_add(&s_queued, 1);
    return ESP_OK;
}

/** What the reset event's "breadcrumbs" tag says about the last boot's ring:
 *  "lost" when RTC memory did not keep it (power-on, possibly a brownout),
 *  "empty" when it was kept but nothing was written to it (the log_mirror tap
 *  was not running), "unavailable" when it was kept but could not be copied. */
static const char *boot_crumbs_state(void)
{
    if (!s_boot_crumbs_present) {
        return "lost";
    }
    if (s_boot_crumbs == NULL) {
        return "unavailable";
    }
    return s_boot_crumb_count > 0 ? "survived" : "empty";
}

esp_err_t event_report_send_reset(const char *reason_name)
{
    esp_err_t err;
    if (s_queue == NULL || s_reset_sent) {
        err = ESP_ERR_INVALID_STATE;
    } else if (!event_report_reset_is_reportable(reason_name)) {
        err = ESP_ERR_NOT_SUPPORTED;
    } else {
        pending_t p = {
            .kind = PENDING_RESET,
            .crumbs_state = boot_crumbs_state(),
            .uptime_ms = esp_timer_get_time() / 1000,
            .crumbs = s_boot_crumbs,
        };
        snprintf(p.reason, sizeof(p.reason), "%s", reason_name != NULL ? reason_name : "UNKNOWN");
        err = enqueue(&p);
        if (err == ESP_OK) {
            s_boot_crumbs = NULL; /* the queue owns it now */
        }
    }
    /* One chance per boot: whatever happened, the snapshot is not needed
     * again, so a reset that is not reported does not hold 3 kB of PSRAM. */
    s_reset_sent = true;
    heap_caps_free(s_boot_crumbs);
    s_boot_crumbs = NULL;
    return err;
}

esp_err_t event_report_send_test(void)
{
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    pending_t p = {
        .kind = PENDING_TEST,
        .uptime_ms = esp_timer_get_time() / 1000,
        .crumbs = heap_caps_malloc(sizeof(event_crumbs_t), MALLOC_CAP_SPIRAM),
    };
    if (p.crumbs != NULL) {
        /* Other tasks keep writing while this copies; at worst one line in
         * the copy is torn, and adopt() keeps it terminated. */
        memcpy(p.crumbs, &s_rtc_crumbs, sizeof(*p.crumbs));
        (void)event_crumbs_adopt(p.crumbs);
    }
    const esp_err_t err = enqueue(&p);
    if (err != ESP_OK) {
        heap_caps_free(p.crumbs);
    }
    return err;
}

void event_report_get_stats(event_report_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->queued = (uint32_t)atomic_load(&s_queued);
    out->sent = (uint32_t)atomic_load(&s_sent);
    out->dropped = (uint32_t)atomic_load(&s_dropped);
    out->pending = s_queue != NULL ? (uint32_t)uxQueueMessagesWaiting(s_queue) : 0;
    out->boot_crumbs = s_boot_crumb_count;
    out->boot_crumbs_present = s_boot_crumbs_present;
}

esp_err_t event_report_send_note(const event_report_note_t *note)
{
    if (note == NULL || note->trigger == NULL || note->kind == NULL || note->text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    note_payload_t *n = heap_caps_calloc(1, sizeof(*n), MALLOC_CAP_SPIRAM);
    if (n == NULL) {
        atomic_fetch_add(&s_dropped, 1);
        return ESP_ERR_NO_MEM;
    }
    snprintf(n->trigger, sizeof(n->trigger), "%s", note->trigger);
    snprintf(n->kind, sizeof(n->kind), "%s", note->kind);
    snprintf(n->persona, sizeof(n->persona), "%s", note->persona != NULL ? note->persona : "");
    snprintf(n->text, sizeof(n->text), "%s", note->text);
    n->reflex_trips = note->reflex_trips;
    n->scene_score = note->scene_score;
    n->still_score = note->still_score;
    n->plan_requests = note->plan_requests;
    const size_t count =
        note->goal_count < EVENT_REPORT_NOTE_GOALS ? note->goal_count : EVENT_REPORT_NOTE_GOALS;
    for (size_t i = 0; i < count; i++) {
        snprintf(n->goals[n->goal_count], sizeof(n->goals[0]), "%s",
                 note->goals[i] != NULL ? note->goals[i] : "");
        if (n->goals[n->goal_count][0] != '\0') {
            n->goal_count++;
        }
    }

    pending_t p = {
        .kind = PENDING_NOTE,
        .uptime_ms = esp_timer_get_time() / 1000,
        .note = n,
    };
    const esp_err_t err = enqueue(&p);
    if (err != ESP_OK) {
        heap_caps_free(n);
    }
    return err;
}
