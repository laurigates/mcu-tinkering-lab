#include "telegram_bot.h"
#include "config.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char* TAG = "TELEGRAM_BOT";

// Helper function to perform HTTP request
static esp_err_t telegram_http_request(const char* url, const char* post_data,
                                       char* response_buffer, size_t buffer_size) {
    esp_err_t err = ESP_OK;

    esp_http_client_config_t config = {
        .url = url,
        .method = post_data ? HTTP_METHOD_POST : HTTP_METHOD_GET,
        .timeout_ms = 10000,
        .buffer_size = buffer_size,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    if (post_data) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, post_data, strlen(post_data));
    }

    err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        if (content_length > 0 && content_length < buffer_size && response_buffer) {
            int read_len = esp_http_client_read(client, response_buffer, content_length);
            response_buffer[read_len] = '\0';
        }
    }

    esp_http_client_cleanup(client);
    return err;
}

// Initialize Telegram bot
esp_err_t telegram_bot_init(telegram_bot_t* bot, const char* token, int64_t chat_id) {
    if (!bot || !token) {
        return ESP_ERR_INVALID_ARG;
    }

    bot->bot_token = strdup(token);
    bot->chat_id = chat_id;
    bot->is_connected = false;
    bot->last_update_id = 0;

    ESP_LOGI(TAG, "Telegram bot initialized");

    // Test connection by getting bot info
    char url[256];
    char response[1024];
    snprintf(url, sizeof(url), "%s%s/getMe", TELEGRAM_API_URL, bot->bot_token);

    esp_err_t err = telegram_http_request(url, NULL, response, sizeof(response));
    if (err == ESP_OK) {
        cJSON* root = cJSON_Parse(response);
        if (root) {
            cJSON* ok = cJSON_GetObjectItem(root, "ok");
            if (ok && cJSON_IsBool(ok) && cJSON_IsTrue(ok)) {
                bot->is_connected = true;
                ESP_LOGI(TAG, "Successfully connected to Telegram bot");
            }
            cJSON_Delete(root);
        }
    }

    return err;
}

// Send text message
esp_err_t telegram_send_text(telegram_bot_t* bot, const char* text) {
    if (!bot || !text || !bot->is_connected) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[256];
    snprintf(url, sizeof(url), "%s%s/sendMessage", TELEGRAM_API_URL, bot->bot_token);

    // Create JSON payload
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "chat_id", bot->chat_id);
    cJSON_AddStringToObject(root, "text", text);
    cJSON_AddStringToObject(root, "parse_mode", "Markdown");

    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    char response[1024];
    esp_err_t err = telegram_http_request(url, json_str, response, sizeof(response));

    free(json_str);

    if (err == ESP_OK) {
        ESP_LOGD(TAG, "Message sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send message: %s", esp_err_to_name(err));
    }

    return err;
}

// Send photo with caption
esp_err_t telegram_send_photo(telegram_bot_t* bot, const uint8_t* photo_data,
                              size_t photo_size, const char* caption) {
    if (!bot || !photo_data || photo_size == 0 || !bot->is_connected) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[256];
    snprintf(url, sizeof(url), "%s%s/sendPhoto", TELEGRAM_API_URL, bot->bot_token);

    // For photo sending, we need multipart/form-data
    // This is a simplified version - in production, use proper multipart encoding

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    // Build multipart form data
    const char* boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    char header[512];
    snprintf(header, sizeof(header),
             "------%s\r\n"
             "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
             "%lld\r\n"
             "------%s\r\n"
             "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n"
             "Content-Type: image/jpeg\r\n\r\n",
             boundary, bot->chat_id, boundary);

    char footer[256];
    if (caption) {
        snprintf(footer, sizeof(footer),
                 "\r\n------%s\r\n"
                 "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
                 "%s\r\n"
                 "------%s--\r\n",
                 boundary, caption, boundary);
    } else {
        snprintf(footer, sizeof(footer), "\r\n------%s--\r\n", boundary);
    }

    size_t total_len = strlen(header) + photo_size + strlen(footer);

    char content_type[128];
    snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type);

    esp_http_client_open(client, total_len);
    esp_http_client_write(client, header, strlen(header));
    esp_http_client_write(client, (char*)photo_data, photo_size);
    esp_http_client_write(client, footer, strlen(footer));

    char response[1024];
    int content_length = esp_http_client_fetch_headers(client);
    if (content_length > 0 && content_length < sizeof(response)) {
        esp_http_client_read(client, response, content_length);
        response[content_length] = '\0';
        ESP_LOGD(TAG, "Photo send response: %s", response);
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "Photo sent with caption: %s", caption ? caption : "(none)");

    return ESP_OK;
}

// Send formatted status message
esp_err_t telegram_send_status(telegram_bot_t* bot, const char* format, ...) {
    char buffer[TELEGRAM_MAX_MESSAGE_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    return telegram_send_text(bot, buffer);
}

// Poll for updates
esp_err_t telegram_poll_updates(telegram_bot_t* bot, telegram_message_t* msg) {
    if (!bot || !msg || !bot->is_connected) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[512];
    snprintf(url, sizeof(url), "%s%s/getUpdates?offset=%d&timeout=10",
             TELEGRAM_API_URL, bot->bot_token, bot->last_update_id + 1);

    char* response = malloc(HTTP_BUFFER_SIZE);
    if (!response) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = telegram_http_request(url, NULL, response, HTTP_BUFFER_SIZE);

    if (err == ESP_OK) {
        cJSON* root = cJSON_Parse(response);
        if (root) {
            cJSON* ok = cJSON_GetObjectItem(root, "ok");
            cJSON* result = cJSON_GetObjectItem(root, "result");

            if (ok && cJSON_IsTrue(ok) && result && cJSON_IsArray(result)) {
                cJSON* update = cJSON_GetArrayItem(result, 0);
                if (update) {
                    cJSON* update_id = cJSON_GetObjectItem(update, "update_id");
                    cJSON* message = cJSON_GetObjectItem(update, "message");

                    if (update_id && message) {
                        bot->last_update_id = update_id->valueint;

                        cJSON* text = cJSON_GetObjectItem(message, "text");
                        cJSON* chat = cJSON_GetObjectItem(message, "chat");
                        cJSON* message_id = cJSON_GetObjectItem(message, "message_id");

                        if (text && chat) {
                            msg->text = strdup(text->valuestring);
                            msg->type = TELEGRAM_MSG_TEXT;

                            cJSON* chat_id = cJSON_GetObjectItem(chat, "id");
                            if (chat_id) {
                                msg->chat_id = chat_id->valueint;
                            }

                            if (message_id) {
                                msg->message_id = message_id->valueint;
                            }

                            // Check if it's a command
                            if (msg->text[0] == '/') {
                                msg->type = TELEGRAM_MSG_COMMAND;
                            }

                            err = ESP_OK;
                        }
                    }
                }
            }
            cJSON_Delete(root);
        }
    }

    free(response);
    return err;
}

// Parse incoming command
bool telegram_parse_command(const char* text, char* command, char* args) {
    if (!text || text[0] != '/') {
        return false;
    }

    const char* space = strchr(text, ' ');
    if (space) {
        // Command with arguments
        size_t cmd_len = space - text - 1;  // Exclude '/'
        strncpy(command, text + 1, cmd_len);
        command[cmd_len] = '\0';
        strcpy(args, space + 1);
    } else {
        // Command without arguments
        strcpy(command, text + 1);  // Skip '/'
        args[0] = '\0';
    }

    return true;
}

// Free message resources
void telegram_free_message(telegram_message_t* msg) {
    if (msg) {
        if (msg->text) {
            free(msg->text);
            msg->text = NULL;
        }
        if (msg->photo_data) {
            free(msg->photo_data);
            msg->photo_data = NULL;
        }
        if (msg->caption) {
            free(msg->caption);
            msg->caption = NULL;
        }
    }
}

// Cleanup bot resources
void telegram_bot_cleanup(telegram_bot_t* bot) {
    if (bot) {
        if (bot->bot_token) {
            free(bot->bot_token);
            bot->bot_token = NULL;
        }
        bot->is_connected = false;
    }
}