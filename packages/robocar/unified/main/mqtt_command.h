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

typedef struct {
    mqtt_command_movement_fn movement;  ///< NULL disables movement commands.
    mqtt_command_line_fn console_line;  ///< NULL disables everything else.
    void *ctx;                          ///< Passed through to both callbacks unchanged.
} mqtt_command_ops_t;

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
 * neither callback.
 *
 * @param line NUL-terminated command line, e.g. from mqtt_command_extract_line().
 * @param ops  Callback table. A NULL member disables that whole category (a
 *             movement command is then rejected exactly like an unrecognised
 *             one if ops->movement is NULL).
 * @return true if the line was recognised and a callback was invoked.
 */
bool mqtt_command_dispatch(const char *line, const mqtt_command_ops_t *ops);

#ifdef __cplusplus
}
#endif

#endif  // MQTT_COMMAND_H
