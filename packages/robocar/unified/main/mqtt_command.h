/**
 * @file mqtt_command.h
 * @brief Validate and route commands received on the MQTT command topic.
 *
 * Pure C, no ESP-IDF or FreeRTOS dependency — see mqtt_command.c for why, and
 * `just robocar-unified::test` for the host suite (test_mqtt_command.c).
 *
 * mqtt_logger.c owns everything MQTT-protocol-specific (the event struct, the
 * subscribe call, connection state); this module owns the *content* of a
 * command message once it has been reduced to plain bytes. That split is what
 * makes the parsing/validation/dispatch logic testable on the host without
 * pulling in esp-mqtt, and it is why mqtt_command_extract_line() below takes
 * plain ints rather than an esp_mqtt_event_t.
 */

#ifndef MQTT_COMMAND_H
#define MQTT_COMMAND_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Longest command line accepted. Comfortably covers every console command in
 * this firmware ("voice budget 3 300" is 19 chars) with headroom, while
 * staying far below esp-mqtt's default receive buffer — a legitimate command
 * can therefore never legitimately arrive fragmented across multiple
 * MQTT_EVENT_DATA events (see mqtt_command_extract_line()).
 */
#define MQTT_COMMAND_MAX_LEN 63

/**
 * @brief Callback for a recognised movement command.
 *
 * Implementations MUST route through reactive_controller_manual() — this
 * firmware does so via the same queue + motor_control_task the serial
 * console uses (see main.c's dispatch_movement()) — and MUST NOT call
 * motor_controller.c directly.
 *
 * @param word One of "forward"/"backward"/"left"/"right"/"rotate_cw"/
 *             "rotate_ccw"/"stop".
 * @param ctx  The mqtt_command_ops_t::ctx pointer, unchanged.
 */
typedef void (*mqtt_command_movement_fn)(const char *word, void *ctx);

/**
 * @brief Callback for any other recognised console-equivalent command line.
 *
 * @param line NUL-terminated line, e.g. "plan resume" or "voice quiet 30",
 *             forwarded verbatim to the same per-prefix handler chain the
 *             serial console dispatches to, so the two entry points can
 *             never disagree about what a command does.
 * @param ctx  The mqtt_command_ops_t::ctx pointer, unchanged.
 */
typedef void (*mqtt_command_line_fn)(const char *line, void *ctx);

/**
 * @brief How much of the command set the MQTT topic may reach (issue #626).
 *
 * READ_ONLY is deliberately the zero value: an ops table or status struct
 * that never set the field is locked down, not open.
 */
typedef enum {
    MQTT_COMMAND_ACCESS_READ_ONLY = 0,  ///< No broker credentials: status commands only.
    MQTT_COMMAND_ACCESS_FULL,           ///< Credentials configured: everything the console has.
} mqtt_command_access_t;

/** Outcome of classifying (and possibly dispatching) one command line. */
typedef enum {
    MQTT_COMMAND_OK = 0,             ///< Allowed (and, from dispatch, delivered).
    MQTT_COMMAND_UNRECOGNISED,       ///< Not a command this topic knows, or its category is off.
    MQTT_COMMAND_REFUSED_READ_ONLY,  ///< Recognised, but not on the read-only allow-list.
} mqtt_command_result_t;

typedef struct {
    mqtt_command_movement_fn movement;  ///< NULL disables movement commands.
    mqtt_command_line_fn console_line;  ///< NULL disables everything else.
    void *ctx;                          ///< Passed through to both callbacks unchanged.
    mqtt_command_access_t access;       ///< Zero-initialised = READ_ONLY.
} mqtt_command_ops_t;

/**
 * @brief Decide the access mode from the configured broker credentials.
 *
 * FULL only when BOTH a username and a password are non-empty. Anything less
 * (NULL, "", or one half missing) is READ_ONLY. This is the single definition
 * of "credentials configured"; main.c and self_report.c both call it, so the
 * boot log, the facts line and the dispatcher cannot disagree about the mode.
 *
 * Note what this does and does not buy: credentials only protect the command
 * topic if the broker refuses anonymous clients and ACLs the topic. The
 * firmware cannot see the broker's policy, only whether it was given
 * credentials to present.
 */
mqtt_command_access_t mqtt_command_access_for_credentials(const char *username,
                                                          const char *password);

/** "full" or "read-only" — the word the boot log and the facts line print. */
const char *mqtt_command_access_name(mqtt_command_access_t access);

/** A short reason for a log line, e.g. "refused: read-only (no broker credentials)". */
const char *mqtt_command_result_reason(mqtt_command_result_t result);

/**
 * @brief Whether @p line is on the read-only allow-list.
 *
 * Exact whole-line match only. The console handlers parse loosely (`trace foo`
 * falls through to the report, `voice <slug>` switches and persists the
 * persona), so a prefix test would let a state-changing line through. A
 * command added to the console later is therefore locked out of read-only
 * mode until somebody adds it here on purpose.
 */
bool mqtt_command_is_read_only(const char *line);

/**
 * @brief Classify @p line under @p access without dispatching it.
 *
 * Lets the caller decide what to do *before* a command runs — main.c uses it
 * to wake the planner only for a command that will actually be carried out,
 * and before rather than after it so that `plan sleep` is not undone by the
 * wake.
 */
mqtt_command_result_t mqtt_command_check(const char *line, mqtt_command_access_t access);

/**
 * @brief Bound-copy an untrusted MQTT payload into a NUL-terminated command line.
 *
 * `event->data` from esp-mqtt is not NUL-terminated, and esp-mqtt can deliver
 * a single logical message across several MQTT_EVENT_DATA callbacks for
 * payloads larger than its internal buffer (`event->total_data_len` /
 * `event->current_data_offset`). A fragment is a topic ("target") already
 * resolved on the first callback, with more of the payload still arriving in
 * later ones — dispatching on a fragment would silently act on a truncated
 * command. This rejects anything that is not a single, complete, in-bounds,
 * printable-ASCII line, so the caller never dispatches a truncated,
 * oversized, or binary payload.
 *
 * @param data                 Raw payload bytes for this fragment (need not
 *                             be NUL-terminated).
 * @param data_len             Bytes of @p data in this fragment.
 * @param total_data_len       Total length of the full (possibly fragmented)
 *                             message, per esp_mqtt_event_t.
 * @param current_data_offset  Byte offset of this fragment within the full
 *                             message, per esp_mqtt_event_t.
 * @param out_line             Destination buffer, at least @p out_line_size bytes.
 * @param out_line_size        Size of @p out_line; must exceed MQTT_COMMAND_MAX_LEN.
 * @return true if @p out_line now holds a NUL-terminated, trimmed command
 *         line; false if the payload was rejected, in which case @p out_line
 *         MUST NOT be dispatched (its contents are unspecified).
 */
bool mqtt_command_extract_line(const char *data, int data_len, int total_data_len,
                               int current_data_offset, char *out_line, size_t out_line_size);

/**
 * @brief Parse a validated command line and dispatch it via @p ops.
 *
 * Recognises the console's single-letter movement vocabulary (F/B/L/R/C/W/S,
 * case-insensitive) and its word form ("forward".."stop"), plus the same
 * command-line prefixes the serial console dispatches on ("plan", "voice",
 * "trace", "mic", "cam", "servo", "led", "sound", "gpio", "snap", "listen").
 * Anything else — including an empty line — is rejected and dispatches
 * neither callback. Under ops->access == READ_ONLY, a recognised line that is
 * not on the read-only allow-list (every movement command included) is
 * refused and likewise dispatches nothing.
 *
 * @param line NUL-terminated command line, e.g. from mqtt_command_extract_line().
 * @param ops  Callback table. A NULL member disables that whole category (a
 *             movement command is then rejected exactly like an unrecognised
 *             one if ops->movement is NULL).
 * @return MQTT_COMMAND_OK if a callback was invoked; otherwise the reason it
 *         was not.
 */
mqtt_command_result_t mqtt_command_dispatch(const char *line, const mqtt_command_ops_t *ops);

#ifdef __cplusplus
}
#endif

#endif  // MQTT_COMMAND_H
