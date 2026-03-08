/**
 * @file display_manager.c
 * @brief Hardware abstraction layer for OLED display and action tracking
 */

#include "display_manager.h"
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "pin_config_idf.h"

static const char *TAG = "display_manager";

// Display state
static struct {
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_io_handle_t io_handle;
    bool initialized;
    display_action_t current_action;
} display_state = {0};

// State name lookup table
static const char *state_names[] = {
    [DISPLAY_STATE_STOPPED] = "STOP",   [DISPLAY_STATE_FORWARD] = "FWD",
    [DISPLAY_STATE_BACKWARD] = "BACK",  [DISPLAY_STATE_LEFT] = "LEFT",
    [DISPLAY_STATE_RIGHT] = "RGHT",     [DISPLAY_STATE_ROTATE_CW] = "CW",
    [DISPLAY_STATE_ROTATE_CCW] = "CCW", [DISPLAY_STATE_UNKNOWN] = "UNK"};

/**
 * @brief Get current time in milliseconds
 */
static unsigned long get_millis(void)
{
    return esp_timer_get_time() / 1000;
}

/**
 * @brief Draw text at specified position
 */
static esp_err_t draw_text(int x, int y, const char *text, bool clear_line)
{
    if (!display_state.initialized || !text) {
        return ESP_ERR_INVALID_STATE;
    }

    // For SSD1306, we would implement pixel-level text rendering here
    // This is a simplified implementation focusing on the API structure
    ESP_LOGD(TAG, "Drawing text at (%d,%d): %s", x, y, text);

    // In a real implementation, this would render text to the display buffer
    // and call esp_lcd_panel_draw_bitmap() to update the display

    return ESP_OK;
}

esp_err_t display_manager_init(void)
{
    if (display_state.initialized) {
        ESP_LOGW(TAG, "Display manager already initialized");
        return ESP_OK;
    }

    if (!OLED_ENABLED) {
        ESP_LOGI(TAG, "OLED display disabled in configuration");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing display manager");

    // Configure I2C bus for OLED
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

    // Configure LCD panel IO
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_1, &io_config,
                                             &display_state.io_handle));

    // Configure SSD1306 panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = OLED_RST_PIN,
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(
        esp_lcd_new_panel_ssd1306(display_state.io_handle, &panel_config, &display_state.panel));

    // Reset and initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(display_state.panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(display_state.panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(display_state.panel, true));

    // Initialize action tracking
    memset(&display_state.current_action, 0, sizeof(display_action_t));
    strcpy(display_state.current_action.action, "STARTUP");
    strcpy(display_state.current_action.source, "INIT");
    display_state.current_action.timestamp = get_millis();

    display_state.initialized = true;
    ESP_LOGI(TAG, "Display manager initialized successfully");

    return display_show_startup();
}

esp_err_t display_show_startup(void)
{
    if (!display_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_ERROR_CHECK(display_clear());

    ESP_ERROR_CHECK(draw_text(0, 0, "ROBOCAR v1.0", false));
    ESP_ERROR_CHECK(draw_text(0, 16, "Initializing...", false));
    ESP_ERROR_CHECK(draw_text(0, 32, "Motor: OK", false));
    ESP_ERROR_CHECK(draw_text(0, 48, "I2C: OK", false));

    ESP_LOGI(TAG, "Startup screen displayed");
    return ESP_OK;
}

esp_err_t display_update_status(display_robot_state_t state, int pan_angle, int tilt_angle,
                                unsigned long last_command_time)
{
    if (!display_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char line_text[DISPLAY_MAX_LINE_LENGTH + 1];

    // Show current state
    const char *state_name =
        (state < sizeof(state_names) / sizeof(state_names[0])) ? state_names[state] : "UNK";
    snprintf(line_text, sizeof(line_text), "STATE: %s", state_name);
    ESP_ERROR_CHECK(draw_text(0, 16, line_text, true));

    // Show servo positions
    snprintf(line_text, sizeof(line_text), "PAN:%d TILT:%d", pan_angle, tilt_angle);
    ESP_ERROR_CHECK(draw_text(0, 40, line_text, true));

    return ESP_OK;
}

esp_err_t display_show_action_debug(const display_action_t *action, display_robot_state_t state,
                                    int pan_angle, int tilt_angle, unsigned long last_command_time)
{
    if (!display_state.initialized || !action) {
        return ESP_ERR_INVALID_STATE;
    }

    char line_text[DISPLAY_MAX_LINE_LENGTH + 1];

    // Line 0: System status with action counter
    snprintf(line_text, sizeof(line_text), "ROBO #%lu", action->counter);
    ESP_ERROR_CHECK(draw_text(0, 0, line_text, true));

    // Line 1: Current state with command source (truncated to fit)
    const char *state_name =
        (state < sizeof(state_names) / sizeof(state_names[0])) ? state_names[state] : "UNK";
    char short_source[5];
    strncpy(short_source, action->source, 4);
    short_source[4] = '\0';
    snprintf(line_text, sizeof(line_text), "%s [%s]", state_name, short_source);
    ESP_ERROR_CHECK(draw_text(0, 8, line_text, true));

    // Line 2: Last action taken (truncated to fit)
    char short_action[12];
    strncpy(short_action, action->action, 11);
    short_action[11] = '\0';
    snprintf(line_text, sizeof(line_text), "ACT:%s", short_action);
    ESP_ERROR_CHECK(draw_text(0, 16, line_text, true));

    // Line 3: Servo positions
    snprintf(line_text, sizeof(line_text), "PAN:%d TLT:%d", pan_angle, tilt_angle);
    ESP_ERROR_CHECK(draw_text(0, 24, line_text, true));

    // Line 4: Time since last command
    unsigned long time_since_cmd = get_millis() - last_command_time;
    if (time_since_cmd < 10000) {
        snprintf(line_text, sizeof(line_text), "CMD:%lums ago", time_since_cmd);
    } else {
        snprintf(line_text, sizeof(line_text), "CMD:>10s ago");
    }
    ESP_ERROR_CHECK(draw_text(0, 32, line_text, true));

    return ESP_OK;
}

esp_err_t display_show_message(int line, const char *message)
{
    if (!display_state.initialized || !message) {
        return ESP_ERR_INVALID_STATE;
    }

    // Calculate Y position based on line number (8 pixels per line)
    int y_pos = line * 8;

    // Ensure line is within display bounds
    if (y_pos < 0 || y_pos >= 64) {
        ESP_LOGW(TAG, "Invalid display line: %d (0-7 valid)", line);
        return ESP_ERR_INVALID_ARG;
    }

    // Display the message with truncation if too long
    char display_msg[DISPLAY_MAX_LINE_LENGTH + 1];
    strncpy(display_msg, message, sizeof(display_msg) - 1);
    display_msg[sizeof(display_msg) - 1] = '\0';

    ESP_ERROR_CHECK(draw_text(0, y_pos, display_msg, true));
    ESP_LOGI(TAG, "Message displayed on line %d: %s", line, display_msg);

    return ESP_OK;
}

esp_err_t display_track_action(const char *action, const char *source)
{
    if (!action || !source) {
        return ESP_ERR_INVALID_ARG;
    }

    display_state.current_action.counter++;
    strncpy(display_state.current_action.action, action,
            sizeof(display_state.current_action.action) - 1);
    display_state.current_action.action[sizeof(display_state.current_action.action) - 1] = '\0';

    strncpy(display_state.current_action.source, source,
            sizeof(display_state.current_action.source) - 1);
    display_state.current_action.source[sizeof(display_state.current_action.source) - 1] = '\0';

    display_state.current_action.timestamp = get_millis();

    ESP_LOGI(TAG, "Action #%lu: %s from %s", display_state.current_action.counter, action, source);

    return ESP_OK;
}

unsigned long display_get_action_counter(void)
{
    return display_state.current_action.counter;
}

esp_err_t display_clear(void)
{
    if (!display_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Clear display buffer and update display
    // In a real implementation, this would clear the entire display buffer
    ESP_LOGD(TAG, "Display cleared");

    return ESP_OK;
}

bool display_is_initialized(void)
{
    return display_state.initialized;
}
