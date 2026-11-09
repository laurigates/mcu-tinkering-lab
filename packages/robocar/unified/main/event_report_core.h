/**
 * @file event_report_core.h
 * @brief Sentry-shaped event JSON and the breadcrumb ring behind it (ADR-033).
 *
 * Pure C plus cJSON, no ESP-IDF dependency — see .claude/rules/testing.md's
 * shared-core pattern. event_report.c is the ESP-IDF half: it owns the ring in
 * RTC memory, the queue and the MQTT publish.
 *
 * The robot publishes each event on MQTT (`robocar/events`); a bridge on the
 * host (tools/glitchtip/bridge) forwards it to GlitchTip. The JSON is the
 * contract between the two:
 *
 *   {"event_id": "<32 hex>", "timestamp": 0, "level": "error",
 *    "source": "firmware", "release": "robocar-unified@1.2.3",
 *    "message": "...", "fingerprint": ["reset", "BROWNOUT"],
 *    "tags": {"reset_reason": "BROWNOUT", ...}, "extra": {"uptime_ms": 1234},
 *    "breadcrumbs": [{"message": "...", "category": "log"}],
 *    "exception": {"type": "...", "value": "...",
 *                  "frames": [{"instruction_addr": "0x42001234"}]}}
 *
 * `timestamp` 0 means the robot has no wall-clock time (it runs no SNTP); the
 * bridge substitutes the time it received the event. Frames are innermost
 * first, as the backtrace walks them; the bridge reverses them for Sentry.
 */

#ifndef EVENT_REPORT_CORE_H
#define EVENT_REPORT_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Lines the breadcrumb ring holds. */
#define EVENT_CRUMBS_LINES 32
/** Bytes per line, NUL included. A longer log line is cut. */
#define EVENT_CRUMBS_LINE_MAX 96
/** Marks a ring written by this firmware layout; anything else is garbage
 *  (RTC memory after power-on) and is discarded. Change it when the layout
 *  changes, so an OTA across a layout change does not read the old shape. */
#define EVENT_CRUMBS_MAGIC 0x43524D43u /* "CRMC" */

/** Most backtrace frames an event carries (esp_core_dump_summary_t's limit). */
#define EVENT_REPORT_FRAMES_MAX 16
#define EVENT_REPORT_TAGS_MAX 12
#define EVENT_REPORT_EXTRAS_MAX 12
#define EVENT_REPORT_FINGERPRINT_MAX 4

/** Length of a hex event id, NUL not counted. */
#define EVENT_REPORT_ID_LEN 32

/**
 * The last log lines, oldest overwritten first. Lives in RTC memory that a
 * panic, watchdog or software reset leaves alone, so the lines leading up to a
 * reset can travel in the next boot's event.
 *
 * Line `s` is stored in `lines[s % EVENT_CRUMBS_LINES]` and `s` in the same
 * slot of `seqs[]`; a slot whose line is empty is unused. Each slot carries its own sequence
 * number rather than the ring keeping one shared counter, so writers racing on
 * two cores cannot leave the ring believing a line is older than it is or
 * missing.
 */
typedef struct {
    uint32_t magic;
    uint32_t seqs[EVENT_CRUMBS_LINES];
    char lines[EVENT_CRUMBS_LINES][EVENT_CRUMBS_LINE_MAX];
} event_crumbs_t;

/** Empty the ring and stamp it valid. */
void event_crumbs_reset(event_crumbs_t *ring);

/**
 * Whether @p ring holds a ring this firmware wrote, and make it safe to read:
 * every line is NUL-terminated afterwards, so a line cut off mid-write by a
 * reset cannot run into the next. Returns false (and leaves the ring as it
 * is) when the magic does not match.
 */
bool event_crumbs_adopt(event_crumbs_t *ring);

/**
 * Store one log line under sequence number @p seq. The text is cleaned for
 * transport: ANSI colour escapes, CR and LF are dropped, other control bytes
 * become '?', and it is cut to EVENT_CRUMBS_LINE_MAX - 1 bytes. A line that is
 * empty after cleaning is not stored and returns false.
 *
 * Callers on several tasks take distinct @p seq values (an atomic counter);
 * then their writes touch distinct slots unless EVENT_CRUMBS_LINES writes race
 * at once. The slot's sequence number is stored before the text, so a line cut
 * off by a reset keeps its place (adopt() terminates it). The text is cleaned
 * straight into the slot with no stack copy: this runs inside the log hook on
 * every task that logs.
 */
bool event_crumbs_write(event_crumbs_t *ring, uint32_t seq, const char *text, size_t len);

/**
 * Collect the ring's lines oldest first into @p out (up to @p max pointers
 * into the ring itself, the newest @p max when there are more). Returns how
 * many were written.
 */
size_t event_crumbs_collect(const event_crumbs_t *ring, const char **out, size_t max);

/**
 * Whether a reset with this reason (reset_reason_name()'s spelling) is worth
 * an event. The deliberate ones are not: power-on, a software restart (OTA,
 * `reboot`), the reset button (EXT), a USB-Serial-JTAG reset (flashing), JTAG,
 * and waking from deep sleep. Everything else, including names this list does
 * not know, is reported.
 */
bool event_report_reset_is_reportable(const char *reason_name);

/** Render 16 random bytes as a 32-character lowercase hex id plus NUL. */
void event_report_format_id(const uint32_t words[4], char out[EVENT_REPORT_ID_LEN + 1]);

typedef struct {
    const char *key;
    const char *value;
} event_report_tag_t;

typedef struct {
    const char *key;
    int64_t value;
} event_report_extra_t;

typedef struct {
    const char *event_id; /**< 32 hex characters */
    int64_t timestamp;    /**< Unix seconds, 0 when unknown */
    const char *level;    /**< "error", "warning" or "info" */
    const char *source;   /**< "firmware" or "planner": routes to a project */
    const char *release;  /**< "robocar-unified@<version>" */
    const char *message;

    const char *fingerprint[EVENT_REPORT_FINGERPRINT_MAX];
    size_t fingerprint_count;

    event_report_tag_t tags[EVENT_REPORT_TAGS_MAX];
    size_t tag_count;

    event_report_extra_t extras[EVENT_REPORT_EXTRAS_MAX];
    size_t extra_count;

    const char *const *breadcrumbs; /**< oldest first */
    size_t breadcrumb_count;

    /** Optional: set exception_type to include an "exception" object. */
    const char *exception_type;
    const char *exception_value;
    uint32_t frames[EVENT_REPORT_FRAMES_MAX]; /**< innermost first */
    size_t frame_count;
} event_report_t;

/**
 * Render @p ev as unformatted JSON. Returns a heap string the caller releases
 * with cJSON_free(), or NULL when a required field (event_id, level, source,
 * release, message) is missing or allocation fails. Counts above the array
 * sizes are clamped. NULL entries in tags, extras, fingerprint and
 * breadcrumbs are skipped.
 */
char *event_report_render(const event_report_t *ev);

#ifdef __cplusplus
}
#endif

#endif  // EVENT_REPORT_CORE_H
