/**
 * @file response_parser.h
 * @brief Parse LLM-generated JSON into ThinkPack command dispatches.
 *
 * The LLM is instructed to respond with a JSON object of the form:
 * @code
 * {"commands":[{"target":"glowbug","type":"led_pattern","r":80,"g":40,
 *               "b":200,"pattern":"sparkle"}, ...]}
 * @endcode
 *
 * response_parser_parse() walks the array and invokes a caller-supplied
 * callback for each recognised command. Unknown targets or types are logged
 * and skipped — the parser never crashes on malformed input.
 */

#ifndef RESPONSE_PARSER_H
#define RESPONSE_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked for each command parsed from the LLM response.
 *
 * @param target_box_type  thinkpack_box_type_t of the intended recipient;
 *                         BOX_UNKNOWN (0) means broadcast.
 * @param command_id       thinkpack_command_id_t value.
 * @param payload          Packed payload bytes (lifetime: duration of callback).
 * @param payload_len      Byte count of @p payload.
 * @param user_ctx         Opaque pointer supplied to response_parser_parse().
 */
typedef void (*response_command_cb_t)(uint8_t target_box_type, uint8_t command_id,
                                      const uint8_t *payload, uint8_t payload_len, void *user_ctx);

/**
 * @brief Parse a JSON response produced by the LLM and invoke @p cb for each command.
 *
 * Safety clamping applied before invoking the callback:
 *   - LED brightness channels clamped to 153 (60 % of 255)
 *   - note_count clamped to CMD_SEQUENCE_MAX_NOTES (20)
 *   - led pattern index clamped to valid enum range
 *
 * @param json_text    NUL-terminated JSON string; must not be NULL.
 * @param json_length  Length of @p json_text (used by cJSON to limit parsing).
 * @param cb           Callback invoked per command; must not be NULL.
 * @param user_ctx     Forwarded verbatim to @p cb.
 * @return ESP_OK if JSON parsed successfully (even if zero commands found),
 *         ESP_ERR_INVALID_ARG for NULL arguments,
 *         ESP_FAIL if the root JSON object cannot be parsed.
 */
esp_err_t response_parser_parse(const char *json_text, size_t json_length, response_command_cb_t cb,
                                void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* RESPONSE_PARSER_H */
