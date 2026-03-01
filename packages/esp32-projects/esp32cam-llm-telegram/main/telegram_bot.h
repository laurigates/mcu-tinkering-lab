#ifndef TELEGRAM_BOT_H
#define TELEGRAM_BOT_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Telegram message types
typedef enum { TELEGRAM_MSG_TEXT, TELEGRAM_MSG_PHOTO, TELEGRAM_MSG_COMMAND } telegram_msg_type_t;

// Telegram message structure
typedef struct {
    char *text;
    uint8_t *photo_data;
    size_t photo_size;
    char *caption;
    telegram_msg_type_t type;
    int64_t chat_id;
    int32_t message_id;
} telegram_message_t;

// Telegram bot context
typedef struct {
    char *bot_token;
    int64_t chat_id;
    bool is_connected;
    uint32_t last_update_id;
} telegram_bot_t;

// Initialize Telegram bot
esp_err_t telegram_bot_init(telegram_bot_t *bot, const char *token, int64_t chat_id);

// Send text message
esp_err_t telegram_send_text(telegram_bot_t *bot, const char *text);

// Send photo with caption
esp_err_t telegram_send_photo(telegram_bot_t *bot, const uint8_t *photo_data, size_t photo_size,
                              const char *caption);

// Send formatted status message
esp_err_t telegram_send_status(telegram_bot_t *bot, const char *format, ...);

// Poll for updates (commands from user)
esp_err_t telegram_poll_updates(telegram_bot_t *bot, telegram_message_t *msg);

// Parse incoming command
bool telegram_parse_command(const char *text, char *command, char *args);

// Free message resources
void telegram_free_message(telegram_message_t *msg);

// Cleanup bot resources
void telegram_bot_cleanup(telegram_bot_t *bot);

#endif  // TELEGRAM_BOT_H
