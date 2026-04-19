/**
 * @file command_executor.h
 * @brief Follower-side helpers: register handlers and dispatch incoming commands.
 *
 * Followers call command_executor_register() once at startup for each command
 * they handle.  On THINKPACK_EVENT_COMMAND_RECEIVED the mesh event callback
 * passes the packet to command_executor_dispatch(), which unpacks the envelope
 * and invokes the appropriate registered handler.
 */

#ifndef COMMAND_EXECUTOR_H
#define COMMAND_EXECUTOR_H

#include <stdint.h>

#include "esp_err.h"
#include "thinkpack_commands.h"

/**
 * @brief Handler for a single command_id.
 *
 * Payload and length are extracted from the thinkpack_command_data_t envelope.
 *
 * @param command_id  The command identifier that triggered this handler.
 * @param payload     Pointer to the command-specific payload bytes.
 * @param length      Number of payload bytes.
 * @param user_ctx    Opaque pointer supplied at registration time.
 */
typedef void (*command_handler_t)(uint8_t command_id, const uint8_t *payload, uint8_t length,
                                  void *user_ctx);

/**
 * @brief Register a handler for a command id.
 *
 * @param command_id  Command identifier to handle.
 * @param handler     Callback to invoke when command_id is received.
 * @param user_ctx    Opaque pointer forwarded to every handler invocation.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if command_id already has a registered handler.
 * @return ESP_ERR_NO_MEM if the registry is full (max 16 entries).
 */
esp_err_t command_executor_register(uint8_t command_id, command_handler_t handler, void *user_ctx);

/**
 * @brief Look up the handler for a packet's command and invoke it.
 *
 * Unpacks the thinkpack_command_data_t from packet->data and dispatches to
 * the registered handler (if any).  Unknown command IDs are logged and ignored.
 * Packets with unexpected msg_type or insufficient data_length are also ignored.
 *
 * @param packet  Non-NULL pointer to the received packet.
 */
void command_executor_dispatch(const thinkpack_packet_t *packet);

/**
 * @brief Clear all registered handlers.
 *
 * Resets the registry to an empty state.  Useful between tests or on
 * device re-initialisation.
 */
void command_executor_reset(void);

#endif /* COMMAND_EXECUTOR_H */
