/**
 * @file event_report.h
 * @brief Error events to GlitchTip over MQTT (ADR-033).
 *
 * The robot publishes Sentry-shaped JSON (event_report_core.h) on
 * EVENT_REPORT_TOPIC at QoS 1; tools/glitchtip/bridge on the host forwards it
 * to GlitchTip. The firmware holds no GlitchTip address and opens no new TLS
 * connection.
 *
 * What is reported:
 *   - one event per boot whose reset was not deliberate
 *     (event_report_reset_is_reportable()), carrying the last log lines of
 *     the boot that ended, from a ring in RTC memory;
 *   - `report test` on the console: a synthetic event for checking the path;
 *   - a note the planner wrote with its `note` tool (note_gate.h), routed by
 *     the bridge to the `robocar-ai` project (source "planner").
 *
 * Events wait in a short queue until MQTT is connected; nothing here blocks
 * the caller, and nothing waits on the network but this module's own task.
 */

#ifndef EVENT_REPORT_H
#define EVENT_REPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Take the previous boot's breadcrumbs out of RTC memory and start a fresh
 * ring for this boot. Call first in app_main, before log_mirror_init() and
 * before anything logs: the first line logged after this overwrites the ring.
 * Makes no logging calls.
 */
void event_report_boot_capture(void);

/**
 * Add one log line to the ring. Called from the log_mirror hook on whichever
 * task logged: makes no logging calls, takes no lock and never waits. Ignored
 * before event_report_boot_capture() and from an ISR.
 */
void event_report_breadcrumb(const char *line, size_t len);

/** Create the queue and the publishing task (Core 1). Idempotent. */
esp_err_t event_report_start(void);

/**
 * Queue the reset event for this boot if @p reason_name (reset_reason_name()'s
 * spelling) is not a deliberate reset. Carries the breadcrumbs taken by
 * event_report_boot_capture(). At most once per boot, and call it even when
 * event_report_start() failed: it releases the snapshot on every path.
 *
 * @return ESP_OK when queued, ESP_ERR_NOT_SUPPORTED when the reason is not
 *         reported, ESP_ERR_INVALID_STATE before event_report_start() or on a
 *         second call, ESP_ERR_NO_MEM when the queue is full.
 */
esp_err_t event_report_send_reset(const char *reason_name);

/** Queue a synthetic `info` event carrying this boot's breadcrumbs so far. */
esp_err_t event_report_send_test(void);

/** Goals a planner note carries, newest last. */
#define EVENT_REPORT_NOTE_GOALS 5

/**
 * A planner note. The model wrote only @c text and chose @c kind; everything
 * else is attached by the firmware, so GlitchTip can group and filter notes on
 * facts rather than on prose.
 */
typedef struct {
    const char *trigger; /**< note_trigger_name(): why the tool was offered */
    const char *kind;    /**< note_kind_name() */
    const char *text;    /**< the model's prose (note_text_clean()) */
    const char *persona; /**< voice_persona_t.slug */
    uint32_t reflex_trips;
    uint32_t scene_score; /**< scene_change_score() */
    uint32_t still_score; /**< plan_activity_scene_score() */
    uint32_t plan_requests;
    /** The planner's last goals as short lines ("track 40%"), oldest first. */
    const char *goals[EVENT_REPORT_NOTE_GOALS];
    size_t goal_count;
} event_report_note_t;

/**
 * Queue a planner note: source "planner", fingerprint [trigger, kind], level
 * "warning" for a difficulty and "info" otherwise. Copies everything, so the
 * caller's strings need not outlive the call.
 *
 * @return ESP_OK when queued, ESP_ERR_INVALID_ARG on a missing field,
 *         ESP_ERR_INVALID_STATE before event_report_start(), ESP_ERR_NO_MEM when
 *         the queue is full or the copy cannot be allocated.
 */
esp_err_t event_report_send_note(const event_report_note_t *note);

typedef struct {
    uint32_t queued;          /**< events accepted into the queue */
    uint32_t sent;            /**< handed to the MQTT client at QoS 1 */
    uint32_t dropped;         /**< refused: queue full or out of memory */
    uint32_t pending;         /**< waiting in the queue now */
    size_t boot_crumbs;       /**< lines carried over from the last boot */
    bool boot_crumbs_present; /**< the RTC ring survived the last reset */
} event_report_stats_t;

void event_report_get_stats(event_report_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif  // EVENT_REPORT_H
