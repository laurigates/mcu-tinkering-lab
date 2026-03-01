/**
 * @file command_handler.c
 * @brief Command Pattern implementation for robot command processing
 */

#include "command_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "display_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_controller.h"
#include "motor_controller.h"
#include "servo_controller.h"

static const char *TAG = "command_handler";

// Forward declarations of command handlers
static esp_err_t handle_move_command(const robot_command_t *cmd);
static esp_err_t handle_sound_command(const robot_command_t *cmd);
static esp_err_t handle_servo_command(const robot_command_t *cmd);
static esp_err_t handle_led_command(const robot_command_t *cmd);
static esp_err_t handle_display_command(const robot_command_t *cmd);
static esp_err_t handle_status_command(const robot_command_t *cmd);
static esp_err_t handle_system_command(const robot_command_t *cmd);

// Command lookup table - replaces string parsing
static const command_entry_t command_table[] = {
    // Movement commands
    {"stop", CMD_TYPE_MOVE, MOVE_STOP, handle_move_command, 0, "Stop all movement"},
    {"forward", CMD_TYPE_MOVE, MOVE_FORWARD, handle_move_command, 1, "Move forward at speed"},
    {"backward", CMD_TYPE_MOVE, MOVE_BACKWARD, handle_move_command, 1, "Move backward at speed"},
    {"left", CMD_TYPE_MOVE, MOVE_LEFT, handle_move_command, 1, "Turn left at speed"},
    {"right", CMD_TYPE_MOVE, MOVE_RIGHT, handle_move_command, 1, "Turn right at speed"},
    {"rotate_cw", CMD_TYPE_MOVE, MOVE_ROTATE_CW, handle_move_command, 1, "Rotate clockwise"},
    {"rotate_ccw", CMD_TYPE_MOVE, MOVE_ROTATE_CCW, handle_move_command, 1,
     "Rotate counter-clockwise"},

    // Sound commands
    {"beep", CMD_TYPE_SOUND, SOUND_BEEP, handle_sound_command, 0, "Play beep sound"},
    {"chirp", CMD_TYPE_SOUND, SOUND_CHIRP, handle_sound_command, 0, "Play chirp sound"},
    {"alert", CMD_TYPE_SOUND, SOUND_ALERT, handle_sound_command, 0, "Play alert sound"},
    {"success", CMD_TYPE_SOUND, SOUND_SUCCESS, handle_sound_command, 0, "Play success sound"},
    {"error", CMD_TYPE_SOUND, SOUND_ERROR, handle_sound_command, 0, "Play error sound"},

    // Servo commands
    {"pan", CMD_TYPE_SERVO, SERVO_PAN, handle_servo_command, 1, "Set pan servo angle"},
    {"tilt", CMD_TYPE_SERVO, SERVO_TILT, handle_servo_command, 1, "Set tilt servo angle"},
    {"center", CMD_TYPE_SERVO, SERVO_CENTER, handle_servo_command, 0, "Center all servos"},
    {"servo_pos", CMD_TYPE_SERVO, SERVO_POSITION, handle_servo_command, 2, "Set pan/tilt position"},

    // LED commands
    {"led_off", CMD_TYPE_LED, LED_OFF, handle_led_command, 0, "Turn off LEDs"},
    {"led_red", CMD_TYPE_LED, LED_RED, handle_led_command, 0, "Set LEDs to red"},
    {"led_green", CMD_TYPE_LED, LED_GREEN, handle_led_command, 0, "Set LEDs to green"},
    {"led_blue", CMD_TYPE_LED, LED_BLUE, handle_led_command, 0, "Set LEDs to blue"},
    {"led_white", CMD_TYPE_LED, LED_WHITE, handle_led_command, 0, "Set LEDs to white"},
    {"led_yellow", CMD_TYPE_LED, LED_YELLOW, handle_led_command, 0, "Set LEDs to yellow"},
    {"led_purple", CMD_TYPE_LED, LED_PURPLE, handle_led_command, 0, "Set LEDs to purple"},
    {"led_cyan", CMD_TYPE_LED, LED_CYAN, handle_led_command, 0, "Set LEDs to cyan"},
    {"led_rgb", CMD_TYPE_LED, LED_RGB, handle_led_command, 3, "Set LEDs to RGB color"},
    {"led_blink", CMD_TYPE_LED, LED_BLINK, handle_led_command, 3, "Blink LEDs with color"},

    // Status commands
    {"status", CMD_TYPE_STATUS, 0, handle_status_command, 0, "Get robot status"},
    {"ping", CMD_TYPE_STATUS, 1, handle_status_command, 0, "Ping robot"},

    // System commands
    {"reset", CMD_TYPE_SYSTEM, 0, handle_system_command, 0, "Reset robot"},
    {"info", CMD_TYPE_SYSTEM, 1, handle_system_command, 0, "Get system info"}};

static const size_t command_table_size = sizeof(command_table) / sizeof(command_entry_t);

// System state
static struct {
    bool initialized;
    uint32_t command_count;
} handler_state = {0};

/**
 * @brief Find command entry by name
 */
static const command_entry_t *find_command(const char *name)
{
    if (!name)
        return NULL;

    for (size_t i = 0; i < command_table_size; i++) {
        if (strcasecmp(command_table[i].name, name) == 0) {
            return &command_table[i];
        }
    }
    return NULL;
}

/**
 * @brief Parse numeric parameter from string
 */
static bool parse_param(const char *str, uint16_t *value)
{
    if (!str || !value)
        return false;

    char *endptr;
    long val = strtol(str, &endptr, 10);

    if (endptr == str || val < 0 || val > 65535) {
        return false;
    }

    *value = (uint16_t)val;
    return true;
}

// Command handler implementations
static esp_err_t handle_move_command(const robot_command_t *cmd)
{
    uint8_t speed = cmd->params.param1;

    switch ((move_command_t)cmd->subtype) {
        case MOVE_STOP:
            return motor_stop();
        case MOVE_FORWARD:
            return motor_move_forward(speed);
        case MOVE_BACKWARD:
            return motor_move_backward(speed);
        case MOVE_LEFT:
            return motor_turn_left(speed);
        case MOVE_RIGHT:
            return motor_turn_right(speed);
        case MOVE_ROTATE_CW:
            return motor_rotate_cw(speed);
        case MOVE_ROTATE_CCW:
            return motor_rotate_ccw(speed);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t handle_sound_command(const robot_command_t *cmd)
{
    // Placeholder for sound implementation
    sound_command_t sound_type = (sound_command_t)cmd->subtype;
    uint16_t duration = cmd->params.param4 ? cmd->params.param4 : 200;  // Default 200ms

    ESP_LOGI(TAG, "Playing sound type %d for %d ms", sound_type, duration);

    // Would interface with piezo buzzer here
    // For now, just log the action
    return ESP_OK;
}

static esp_err_t handle_servo_command(const robot_command_t *cmd)
{
    switch ((servo_command_t)cmd->subtype) {
        case SERVO_PAN:
            return servo_set_pan((int16_t)cmd->params.param1 - 90);  // Convert 0-180 to -90-90
        case SERVO_TILT:
            return servo_set_tilt((int16_t)cmd->params.param1 - 90);  // Convert 0-180 to -90-90
        case SERVO_CENTER:
            return servo_center_all();
        case SERVO_POSITION:
            return servo_set_position((int16_t)cmd->params.param1 - 90,
                                      (int16_t)cmd->params.param2 - 90);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t handle_led_command(const robot_command_t *cmd)
{
    led_command_t led_type = (led_command_t)cmd->subtype;

    switch (led_type) {
        case LED_OFF:
            return led_turn_off_all();
        case LED_RED:
            return led_set_both(&LED_COLOR_RED);
        case LED_GREEN:
            return led_set_both(&LED_COLOR_GREEN);
        case LED_BLUE:
            return led_set_both(&LED_COLOR_BLUE);
        case LED_WHITE:
            return led_set_both(&LED_COLOR_WHITE);
        case LED_YELLOW:
            return led_set_both(&LED_COLOR_YELLOW);
        case LED_PURPLE:
            return led_set_both(&LED_COLOR_PURPLE);
        case LED_CYAN:
            return led_set_both(&LED_COLOR_CYAN);
        case LED_RGB: {
            rgb_color_t color = {
                .red = cmd->params.param1, .green = cmd->params.param2, .blue = cmd->params.param3};
            return led_set_both(&color);
        }
        case LED_BLINK: {
            rgb_color_t color = {
                .red = cmd->params.param1, .green = cmd->params.param2, .blue = cmd->params.param3};
            uint32_t duration = cmd->params.param4 ? cmd->params.param4 : 500;
            return led_blink(LED_BOTH, &color, duration, duration, 3);  // 3 blinks
        }
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t handle_display_command(const robot_command_t *cmd)
{
    // Placeholder for display commands
    ESP_LOGI(TAG, "Display command not implemented");
    return ESP_OK;
}

static esp_err_t handle_status_command(const robot_command_t *cmd)
{
    if (cmd->subtype == 0) {  // status
        ESP_LOGI(TAG, "Robot Status: Commands processed: %lu", handler_state.command_count);
    } else if (cmd->subtype == 1) {  // ping
        ESP_LOGI(TAG, "Pong");
    }
    return ESP_OK;
}

static esp_err_t handle_system_command(const robot_command_t *cmd)
{
    if (cmd->subtype == 0) {  // reset
        ESP_LOGI(TAG, "System reset requested");
        // Would reset system state here
    } else if (cmd->subtype == 1) {  // info
        ESP_LOGI(TAG, "System Info: Commands: %zu, Processed: %lu", command_table_size,
                 handler_state.command_count);
    }
    return ESP_OK;
}

// Public interface implementations
esp_err_t command_handler_init(void)
{
    if (handler_state.initialized) {
        ESP_LOGW(TAG, "Command handler already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing command handler with %zu commands", command_table_size);

    handler_state.command_count = 0;
    handler_state.initialized = true;

    ESP_LOGI(TAG, "Command handler initialized successfully");
    return ESP_OK;
}

esp_err_t command_parse_string(const char *cmd_string, robot_command_t *cmd)
{
    if (!cmd_string || !cmd || !handler_state.initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse command string: "command param1 param2 ..."
    char cmd_copy[128];
    strncpy(cmd_copy, cmd_string, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    char *token = strtok(cmd_copy, " ");
    if (!token) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find command in lookup table
    const command_entry_t *entry = find_command(token);
    if (!entry) {
        ESP_LOGW(TAG, "Unknown command: %s", token);
        return ESP_ERR_NOT_FOUND;
    }

    // Initialize command structure
    memset(cmd, 0, sizeof(robot_command_t));
    cmd->type = entry->type;
    cmd->subtype = entry->subtype;
    cmd->timestamp = esp_timer_get_time() / 1000;  // Convert to milliseconds

    // Parse parameters
    uint16_t params[4] = {0};
    int param_count = 0;

    while ((token = strtok(NULL, " ")) != NULL && param_count < 4) {
        if (!parse_param(token, &params[param_count])) {
            ESP_LOGW(TAG, "Invalid parameter: %s", token);
            return ESP_ERR_INVALID_ARG;
        }
        param_count++;
    }

    // Check minimum parameter count
    if (param_count < entry->min_params) {
        ESP_LOGW(TAG, "Command %s requires at least %d parameters, got %d", entry->name,
                 entry->min_params, param_count);
        return ESP_ERR_INVALID_ARG;
    }

    // Store parameters
    cmd->params.param1 = (uint8_t)params[0];
    cmd->params.param2 = (uint8_t)params[1];
    cmd->params.param3 = (uint8_t)params[2];
    cmd->params.param4 = params[3];

    return ESP_OK;
}

esp_err_t command_execute(const robot_command_t *cmd)
{
    if (!cmd || !handler_state.initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Track action for display
    if (display_is_initialized()) {
        const command_entry_t *entry = NULL;
        for (size_t i = 0; i < command_table_size; i++) {
            if (command_table[i].type == cmd->type && command_table[i].subtype == cmd->subtype) {
                entry = &command_table[i];
                break;
            }
        }

        if (entry) {
            display_track_action(entry->name, cmd->source);
        }
    }

    // Find and execute command handler
    for (size_t i = 0; i < command_table_size; i++) {
        if (command_table[i].type == cmd->type && command_table[i].subtype == cmd->subtype) {
            ESP_LOGD(TAG, "Executing command: %s from %s", command_table[i].name, cmd->source);

            esp_err_t ret = command_table[i].handler(cmd);

            if (ret == ESP_OK) {
                handler_state.command_count++;
            } else {
                ESP_LOGW(TAG, "Command execution failed: %s", esp_err_to_name(ret));
            }

            return ret;
        }
    }

    ESP_LOGW(TAG, "No handler found for command type %d, subtype %d", cmd->type, cmd->subtype);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t command_execute_string(const char *cmd_string, const char *source)
{
    robot_command_t cmd;

    esp_err_t ret = command_parse_string(cmd_string, &cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set command source
    if (source) {
        strncpy(cmd.source, source, sizeof(cmd.source) - 1);
        cmd.source[sizeof(cmd.source) - 1] = '\0';
    } else {
        strcpy(cmd.source, "UNKNOWN");
    }

    return command_execute(&cmd);
}

const command_entry_t *command_get_info(const char *name)
{
    return find_command(name);
}

int command_list_all(char *buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return 0;
    }

    int count = 0;
    size_t pos = 0;

    for (size_t i = 0; i < command_table_size && pos < buffer_size - 1; i++) {
        int written = snprintf(buffer + pos, buffer_size - pos, "%s: %s\n", command_table[i].name,
                               command_table[i].description);

        if (written > 0 && pos + written < buffer_size) {
            pos += written;
            count++;
        } else {
            break;
        }
    }

    buffer[pos] = '\0';
    return count;
}

bool command_validate(const robot_command_t *cmd)
{
    if (!cmd)
        return false;

    // Find command entry
    for (size_t i = 0; i < command_table_size; i++) {
        if (command_table[i].type == cmd->type && command_table[i].subtype == cmd->subtype) {
            return true;  // Found valid command
        }
    }

    return false;  // Unknown command
}

const char *command_type_to_string(command_type_t type)
{
    switch (type) {
        case CMD_TYPE_MOVE:
            return "MOVE";
        case CMD_TYPE_SOUND:
            return "SOUND";
        case CMD_TYPE_SERVO:
            return "SERVO";
        case CMD_TYPE_LED:
            return "LED";
        case CMD_TYPE_DISPLAY:
            return "DISPLAY";
        case CMD_TYPE_STATUS:
            return "STATUS";
        case CMD_TYPE_SYSTEM:
            return "SYSTEM";
        default:
            return "UNKNOWN";
    }
}

// Command creation helpers
robot_command_t command_create_move(move_command_t move_type, uint8_t speed, const char *source)
{
    robot_command_t cmd = {0};
    cmd.type = CMD_TYPE_MOVE;
    cmd.subtype = move_type;
    cmd.params.param1 = speed;
    cmd.timestamp = esp_timer_get_time() / 1000;

    if (source) {
        strncpy(cmd.source, source, sizeof(cmd.source) - 1);
    }

    return cmd;
}

robot_command_t command_create_servo(servo_command_t servo_type, uint8_t param1, uint8_t param2,
                                     const char *source)
{
    robot_command_t cmd = {0};
    cmd.type = CMD_TYPE_SERVO;
    cmd.subtype = servo_type;
    cmd.params.param1 = param1;
    cmd.params.param2 = param2;
    cmd.timestamp = esp_timer_get_time() / 1000;

    if (source) {
        strncpy(cmd.source, source, sizeof(cmd.source) - 1);
    }

    return cmd;
}

robot_command_t command_create_led(led_command_t led_type, uint8_t param1, uint8_t param2,
                                   uint8_t param3, const char *source)
{
    robot_command_t cmd = {0};
    cmd.type = CMD_TYPE_LED;
    cmd.subtype = led_type;
    cmd.params.param1 = param1;
    cmd.params.param2 = param2;
    cmd.params.param3 = param3;
    cmd.timestamp = esp_timer_get_time() / 1000;

    if (source) {
        strncpy(cmd.source, source, sizeof(cmd.source) - 1);
    }

    return cmd;
}

robot_command_t command_create_sound(sound_command_t sound_type, uint16_t duration,
                                     uint16_t frequency, const char *source)
{
    robot_command_t cmd = {0};
    cmd.type = CMD_TYPE_SOUND;
    cmd.subtype = sound_type;
    cmd.params.param4 = duration;
    cmd.timestamp = esp_timer_get_time() / 1000;

    if (source) {
        strncpy(cmd.source, source, sizeof(cmd.source) - 1);
    }

    return cmd;
}
