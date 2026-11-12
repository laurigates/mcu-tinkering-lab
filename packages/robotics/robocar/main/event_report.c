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
#include "sdkconfig.h"

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
#include "esp_core_dump.h"
#define EVENT_REPORT_HAVE_COREDUMP 1
#else
#define EVENT_REPORT_HAVE_COREDUMP 0
#endif

static const char *TAG = "event_report";

typedef enum {
    PENDING_RESET,
    PENDING_TEST,
    PENDING_CRASH,
    PENDING_NOTE,
} pending_kind_t;

/** The parts of a core dump's summary the crash event carries. */
typedef struct {
    char task[16];
    char reason[128]; /**< panic details from the dump; may be empty */
    uint32_t pc;
    uint32_t cause;
    uint32_t vaddr;
    uint32_t frames[EVENT_REPORT_FRAMES_MAX]; /**< innermost first */
    size_t depth;
    bool corrupted;
} crash_info_t;

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
    crash_info_t *crash;    /**< owned, crash only */
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

/* Core dump state (ADR-033). The dump is erased only once the broker has
 * acknowledged the crash event. The acknowledgement (PUBACK) arrives on the
 * MQTT task; this module's task does the slow flash erase.
 *
 * Every forwarded log line is also a QoS 1 publish, so PUBACKs are frequent:
 * rather than keep a history of acknowledged ids, the callback compares each
 * one with the crash event's id while that is awaited. The one window it
 * cannot cover is a PUBACK that lands before mqtt_logger_publish_tracked()
 * has returned the id; ids seen during that one call (DUMP_PUBLISHING) go
 * into a small ring that the task checks right after it stores the id.
 *
 * esp-mqtt drops an unacknowledged message from its outbox after 30 s with no
 * event, so a crash event not acknowledged within EVENT_REPORT_ACK_TIMEOUT_MS
 * is queued again. A reboot before the acknowledgement leaves the dump, and
 * the next boot sends it again. */
typedef enum {
    DUMP_NONE,
    DUMP_QUEUED,
    DUMP_PUBLISHING,
    DUMP_AWAITING_ACK,
    DUMP_ACKED,
    DUMP_ERASED,
    DUMP_ERASE_FAILED,
} dump_state_t;

static _Atomic dump_state_t s_dump_state = DUMP_NONE;
static atomic_int s_dump_msg_id = -1;
#define EARLY_ACK_RING 8
static atomic_int s_early_acks[EARLY_ACK_RING];
static atomic_uint s_early_next;
/* The crash event while it waits for its acknowledgement, kept so it can be
 * sent again; owned by this module's task. */
static pending_t s_inflight;
static int64_t s_inflight_sent_us;

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
    snprintf(release, sizeof(release), "robocar@%s", ota_manager_get_version());

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
    } else if (p->kind == PENDING_CRASH) {
        const crash_info_t *c = p->crash;
        char type[EVENT_REPORT_CRASH_TYPE_MAX + 1];
        event_report_crash_type(c->reason, event_report_xtensa_cause_name(c->cause), type,
                                sizeof(type));
        char value[200];
        snprintf(value, sizeof(value), "%s (task %s, PC 0x%08x, EXCVADDR 0x%08x)",
                 c->reason[0] != '\0' ? c->reason : type, c->task, (unsigned)c->pc,
                 (unsigned)c->vaddr);
        ev.level = "fatal";
        ev.message = value;
        ev.exception_type = type;
        ev.exception_value = value;
        memcpy(ev.frames, c->frames, sizeof(ev.frames));
        ev.frame_count = c->depth;
        /* No fingerprint: GlitchTip groups crashes by their stack trace. */
        ev.tags[ev.tag_count++] = (event_report_tag_t){"reset_reason", p->reason};
        ev.tags[ev.tag_count++] = (event_report_tag_t){"breadcrumbs", p->crumbs_state};
        ev.tags[ev.tag_count++] = (event_report_tag_t){"crash_task", c->task};
        ev.tags[ev.tag_count++] =
            (event_report_tag_t){"backtrace", c->corrupted ? "corrupted" : "complete"};
    } else {
        ev.level = "info";
        ev.message = "report test";
        ev.fingerprint[ev.fingerprint_count++] = "report-test";
    }
    return event_report_render(&ev);
}

static void on_published(int msg_id)
{
    const dump_state_t state = atomic_load(&s_dump_state);
    if (state == DUMP_PUBLISHING) {
        const unsigned slot = atomic_fetch_add(&s_early_next, 1) % EARLY_ACK_RING;
        atomic_store(&s_early_acks[slot], msg_id);
    } else if (state == DUMP_AWAITING_ACK && msg_id == atomic_load(&s_dump_msg_id)) {
        dump_state_t expected = DUMP_AWAITING_ACK;
        atomic_compare_exchange_strong(&s_dump_state, &expected, DUMP_ACKED);
    }
}

static bool acked_early(int msg_id)
{
    for (size_t i = 0; i < EARLY_ACK_RING; i++) {
        if (atomic_load(&s_early_acks[i]) == msg_id) {
            return true;
        }
    }
    return false;
}

#if EVENT_REPORT_HAVE_COREDUMP
static void release_inflight(void)
{
    heap_caps_free(s_inflight.crumbs);
    heap_caps_free(s_inflight.crash);
    memset(&s_inflight, 0, sizeof(s_inflight));
}
#endif

/** Erase the core dump once its event is acknowledged, or queue the event
 *  again if the acknowledgement is overdue. Runs on this module's task, never
 *  the MQTT one: erasing 128 kB of flash takes a while. */
static void service_dump(void)
{
#if EVENT_REPORT_HAVE_COREDUMP
    const dump_state_t state = atomic_load(&s_dump_state);
    if (state == DUMP_ACKED) {
        release_inflight();
        const esp_err_t err = esp_core_dump_image_erase();
        atomic_store(&s_dump_state, err == ESP_OK ? DUMP_ERASED : DUMP_ERASE_FAILED);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "crash event acknowledged; core dump erased");
        } else {
            ESP_LOGW(TAG, "crash event acknowledged; core dump erase failed (%s)",
                     esp_err_to_name(err));
        }
    } else if (state == DUMP_AWAITING_ACK &&
               esp_timer_get_time() - s_inflight_sent_us > EVENT_REPORT_ACK_TIMEOUT_MS * 1000LL) {
        dump_state_t expected = DUMP_AWAITING_ACK;
        if (atomic_compare_exchange_strong(&s_dump_state, &expected, DUMP_QUEUED)) {
            if (xQueueSendToFront(s_queue, &s_inflight, 0) == pdTRUE) {
                memset(&s_inflight, 0, sizeof(s_inflight)); /* the queue owns it */
                ESP_LOGW(TAG, "crash event not acknowledged; sending it again");
            } else {
                atomic_store(&s_dump_state, DUMP_AWAITING_ACK); /* try next pass */
            }
        }
    }
#endif
}

static void publish_task(void *arg)
{
    (void)arg;
    for (;;) {
        service_dump();
        pending_t p;
        /* Bounded, so an acknowledgement is acted on with the queue empty. */
        if (xQueuePeek(s_queue, &p, pdMS_TO_TICKS(EVENT_REPORT_RETRY_MS)) != pdTRUE) {
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
        const bool crash = (p.kind == PENDING_CRASH);
        if (crash) {
            for (size_t i = 0; i < EARLY_ACK_RING; i++) {
                atomic_store(&s_early_acks[i], -1);
            }
            atomic_store(&s_dump_state, DUMP_PUBLISHING);
        }
        int msg_id = 0;
        const esp_err_t err =
            mqtt_logger_publish_tracked(EVENT_REPORT_TOPIC, json, 1, false, &msg_id);
        cJSON_free(json);
        if (err != ESP_OK) {
            if (crash) {
                atomic_store(&s_dump_state, DUMP_QUEUED);
            }
            vTaskDelay(pdMS_TO_TICKS(EVENT_REPORT_RETRY_MS));
            continue;
        }
        (void)xQueueReceive(s_queue, &p, 0);
        if (crash) {
            s_inflight = p; /* kept until acknowledged, to send again if not */
            s_inflight_sent_us = esp_timer_get_time();
            atomic_store(&s_dump_msg_id, msg_id);
            atomic_store(&s_dump_state, acked_early(msg_id) ? DUMP_ACKED : DUMP_AWAITING_ACK);
        } else {
            heap_caps_free(p.crumbs);
            heap_caps_free(p.crash);
            heap_caps_free(p.note);
        }
        atomic_fetch_add(&s_sent, 1);
        ESP_LOGI(TAG, "event published on %s", EVENT_REPORT_TOPIC);
    }
}

esp_err_t event_report_start(void)
{
    if (s_queue != NULL) {
        return ESP_OK;
    }
    for (size_t i = 0; i < EARLY_ACK_RING; i++) {
        atomic_store(&s_early_acks[i], -1);
    }
    mqtt_logger_set_published_handler(on_published);
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

/** Read the core dump's summary into a new crash_info_t, or NULL when there
 *  is no valid dump (or core dumps are not configured). */
static crash_info_t *read_crash(void)
{
#if EVENT_REPORT_HAVE_COREDUMP
    if (esp_core_dump_image_check() != ESP_OK) {
        return NULL;
    }
    /* Zeroed: get_summary() can return ESP_OK and leave the task, PC and
     * backtrace untouched when it finds no segment for the crashing TCB. */
    esp_core_dump_summary_t *sum = heap_caps_calloc(1, sizeof(*sum), MALLOC_CAP_SPIRAM);
    crash_info_t *c = heap_caps_calloc(1, sizeof(*c), MALLOC_CAP_SPIRAM);
    if (sum == NULL || c == NULL || esp_core_dump_get_summary(sum) != ESP_OK) {
        heap_caps_free(sum);
        heap_caps_free(c);
        return NULL;
    }
    snprintf(c->task, sizeof(c->task), "%.*s", (int)sizeof(sum->exc_task), sum->exc_task);
    c->pc = sum->exc_pc;
    c->cause = sum->ex_info.exc_cause;
    c->vaddr = sum->ex_info.exc_vaddr;
    c->depth = sum->exc_bt_info.depth < EVENT_REPORT_FRAMES_MAX ? sum->exc_bt_info.depth
                                                                : EVENT_REPORT_FRAMES_MAX;
    memcpy(c->frames, sum->exc_bt_info.bt, c->depth * sizeof(c->frames[0]));
    c->corrupted = sum->exc_bt_info.corrupted;
    heap_caps_free(sum);
    if (c->pc == 0 && c->depth == 0) {
        c->corrupted = true; /* no usable summary: report the dump without frames */
    }
    /* A task name the summary never filled is empty; keep it printable. */
    for (size_t i = 0; c->task[i] != '\0'; i++) {
        if ((unsigned char)c->task[i] < 0x20 || (unsigned char)c->task[i] > 0x7e) {
            c->task[i] = '?';
        }
    }
    if (c->task[0] == '\0') {
        snprintf(c->task, sizeof(c->task), "unknown");
    }
    if (esp_core_dump_get_panic_reason(c->reason, sizeof(c->reason)) != ESP_OK) {
        c->reason[0] = '\0';
    }
    return c;
#else
    return NULL;
#endif
}

esp_err_t event_report_send_reset(const char *reason_name)
{
    esp_err_t err;
    crash_info_t *crash = NULL;
    if (s_queue == NULL || s_reset_sent) {
        err = ESP_ERR_INVALID_STATE;
    } else if ((crash = read_crash()) != NULL) {
        /* A core dump is reported whatever this boot's reset reason: it may
         * be left over from a boot that crashed and then reset again before
         * the event was acknowledged. It replaces the plain reset event. */
        pending_t p = {
            .kind = PENDING_CRASH,
            .crumbs_state = boot_crumbs_state(),
            .uptime_ms = esp_timer_get_time() / 1000,
            .crumbs = s_boot_crumbs,
            .crash = crash,
        };
        snprintf(p.reason, sizeof(p.reason), "%s", reason_name != NULL ? reason_name : "UNKNOWN");
        err = enqueue(&p);
        if (err == ESP_OK) {
            s_boot_crumbs = NULL;
            atomic_store(&s_dump_state, DUMP_QUEUED);
        } else {
            heap_caps_free(crash);
        }
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
    static const char *const k_dump[] = {
        [DUMP_NONE] = "none",
        [DUMP_QUEUED] = "queued",
        [DUMP_PUBLISHING] = "publishing",
        [DUMP_AWAITING_ACK] = "sent, awaiting acknowledgement",
        [DUMP_ACKED] = "acknowledged, erasing",
        [DUMP_ERASED] = "acknowledged and erased",
        [DUMP_ERASE_FAILED] = "acknowledged, erase failed",
    };
    out->core_dump = k_dump[atomic_load(&s_dump_state)];
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
