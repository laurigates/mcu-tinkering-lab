/**
 * @file led_controller.c
 * @brief Hardware abstraction layer for PCA9685 RGB LED control
 */

#include "led_controller.h"
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "pin_config_idf.h"

static const char *TAG = "led_controller";

// PCA9685 register addresses
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

// LED channel mappings (assuming 3 channels per LED: R, G, B)
#define LED_LEFT_R_CHANNEL 0
#define LED_LEFT_G_CHANNEL 1
#define LED_LEFT_B_CHANNEL 2
#define LED_RIGHT_R_CHANNEL 3
#define LED_RIGHT_G_CHANNEL 4
#define LED_RIGHT_B_CHANNEL 5

// LED controller state
static struct {
    bool initialized;
    rgb_color_t left_color;
    rgb_color_t right_color;
    TimerHandle_t blink_timer;
    bool blinking;
    uint32_t blink_count_remaining;
    led_position_t blink_position;
    rgb_color_t blink_color;
    rgb_color_t blink_off_color;
} led_state = {0};

// Predefined colors
const rgb_color_t LED_COLOR_OFF = {0, 0, 0};
const rgb_color_t LED_COLOR_RED = {255, 0, 0};
const rgb_color_t LED_COLOR_GREEN = {0, 255, 0};
const rgb_color_t LED_COLOR_BLUE = {0, 0, 255};
const rgb_color_t LED_COLOR_WHITE = {255, 255, 255};
const rgb_color_t LED_COLOR_YELLOW = {255, 255, 0};
const rgb_color_t LED_COLOR_PURPLE = {255, 0, 255};
const rgb_color_t LED_COLOR_CYAN = {0, 255, 255};

/**
 * @brief Write to PCA9685 register
 */
static esp_err_t pca9685_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(I2C_NUM_1, PCA9685_I2C_ADDR, write_buf, sizeof(write_buf),
                                      pdMS_TO_TICKS(100));
}

/**
 * @brief Set PWM value for specific PCA9685 channel
 */
static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t value)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    // PCA9685 uses 12-bit resolution (0-4095)
    value = (value > 4095) ? 4095 : value;

    uint8_t reg_base = PCA9685_LED0_ON_L + (channel * 4);

    // Write ON_L, ON_H (always 0 for simple PWM)
    esp_err_t ret = pca9685_write_reg(reg_base, 0);
    ret |= pca9685_write_reg(reg_base + 1, 0);

    // Write OFF_L, OFF_H
    ret |= pca9685_write_reg(reg_base + 2, value & 0xFF);
    ret |= pca9685_write_reg(reg_base + 3, (value >> 8) & 0xFF);

    return ret;
}

/**
 * @brief Convert 8-bit color value to 12-bit PWM value
 */
static uint16_t color_to_pwm(uint8_t color_value)
{
    return (color_value * 4095) / 255;
}

/**
 * @brief Set RGB color for specific LED hardware
 */
static esp_err_t led_set_hardware(led_position_t position, const rgb_color_t *color)
{
    if (!color) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (position == LED_LEFT || position == LED_BOTH) {
        ret |= pca9685_set_pwm(LED_LEFT_R_CHANNEL, color_to_pwm(color->red));
        ret |= pca9685_set_pwm(LED_LEFT_G_CHANNEL, color_to_pwm(color->green));
        ret |= pca9685_set_pwm(LED_LEFT_B_CHANNEL, color_to_pwm(color->blue));
        led_state.left_color = *color;
    }

    if (position == LED_RIGHT || position == LED_BOTH) {
        ret |= pca9685_set_pwm(LED_RIGHT_R_CHANNEL, color_to_pwm(color->red));
        ret |= pca9685_set_pwm(LED_RIGHT_G_CHANNEL, color_to_pwm(color->green));
        ret |= pca9685_set_pwm(LED_RIGHT_B_CHANNEL, color_to_pwm(color->blue));
        led_state.right_color = *color;
    }

    return ret;
}

/**
 * @brief Blink timer callback
 */
static void blink_timer_callback(TimerHandle_t xTimer)
{
    static bool blink_state = false;

    if (!led_state.blinking) {
        return;
    }

    // Toggle between blink color and off color
    const rgb_color_t *target_color =
        blink_state ? &led_state.blink_color : &led_state.blink_off_color;
    led_set_hardware(led_state.blink_position, target_color);

    blink_state = !blink_state;

    // Handle blink count
    if (led_state.blink_count_remaining > 0) {
        if (blink_state) {  // Just turned on, count as one blink
            led_state.blink_count_remaining--;
            if (led_state.blink_count_remaining == 0) {
                led_stop_blink();
            }
        }
    }
}

esp_err_t led_controller_init(void)
{
    if (led_state.initialized) {
        ESP_LOGW(TAG, "LED controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing LED controller");

    // Initialize PCA9685
    esp_err_t ret = pca9685_write_reg(PCA9685_MODE1, 0x00);  // Normal mode
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set PWM frequency to ~1kHz
    ret = pca9685_write_reg(PCA9685_PRESCALE, 0x79);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCA9685 prescale: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create blink timer
    led_state.blink_timer = xTimerCreate("led_blink",
                                         pdMS_TO_TICKS(500),  // Default 500ms
                                         pdTRUE,              // Auto-reload
                                         NULL, blink_timer_callback);
    if (!led_state.blink_timer) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        return ESP_ERR_NO_MEM;
    }

    // Initialize LED state
    led_state.left_color = LED_COLOR_OFF;
    led_state.right_color = LED_COLOR_OFF;
    led_state.blinking = false;

    // Turn off all LEDs
    led_turn_off_all();

    led_state.initialized = true;
    ESP_LOGI(TAG, "LED controller initialized successfully");
    return ESP_OK;
}

esp_err_t led_set_color(led_position_t position, const rgb_color_t *color)
{
    if (!led_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!color) {
        return ESP_ERR_INVALID_ARG;
    }

    // Stop any ongoing blink
    if (led_state.blinking) {
        led_stop_blink();
    }

    return led_set_hardware(position, color);
}

esp_err_t led_set_rgb(led_position_t position, uint8_t red, uint8_t green, uint8_t blue)
{
    rgb_color_t color = {red, green, blue};
    return led_set_color(position, &color);
}

esp_err_t led_set_left(const rgb_color_t *color)
{
    return led_set_color(LED_LEFT, color);
}

esp_err_t led_set_right(const rgb_color_t *color)
{
    return led_set_color(LED_RIGHT, color);
}

esp_err_t led_set_both(const rgb_color_t *color)
{
    return led_set_color(LED_BOTH, color);
}

esp_err_t led_turn_off(led_position_t position)
{
    return led_set_color(position, &LED_COLOR_OFF);
}

esp_err_t led_turn_off_all(void)
{
    return led_set_color(LED_BOTH, &LED_COLOR_OFF);
}

esp_err_t led_update(void)
{
    if (!led_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // For PCA9685, changes are immediate, so this is a no-op
    // Provided for API compatibility with other LED controllers
    return ESP_OK;
}

esp_err_t led_get_color(led_position_t position, rgb_color_t *color)
{
    if (!led_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!color) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (position) {
        case LED_LEFT:
            *color = led_state.left_color;
            break;
        case LED_RIGHT:
            *color = led_state.right_color;
            break;
        case LED_BOTH:
            return ESP_ERR_INVALID_ARG;  // Cannot get color for both
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t led_blink(led_position_t position, const rgb_color_t *color, uint32_t on_time_ms,
                    uint32_t off_time_ms, uint32_t blink_count)
{
    if (!led_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!color || position == LED_BOTH) {
        return ESP_ERR_INVALID_ARG;  // Blink doesn't support BOTH for simplicity
    }

    // Stop any existing blink
    led_stop_blink();

    // Store blink parameters
    led_state.blink_position = position;
    led_state.blink_color = *color;
    led_state.blink_off_color = LED_COLOR_OFF;
    led_state.blink_count_remaining = blink_count;
    led_state.blinking = true;

    // Calculate timer period (on_time + off_time) / 2 for toggle rate
    uint32_t timer_period = (on_time_ms + off_time_ms) / 2;
    if (timer_period < 10)
        timer_period = 10;  // Minimum 10ms

    // Update timer period and start
    xTimerChangePeriod(led_state.blink_timer, pdMS_TO_TICKS(timer_period), 0);
    xTimerStart(led_state.blink_timer, 0);

    ESP_LOGD(TAG, "Started blinking %s LED with period %lums, count %lu",
             (position == LED_LEFT) ? "left" : "right", timer_period, blink_count);

    return ESP_OK;
}

esp_err_t led_stop_blink(void)
{
    if (!led_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (led_state.blinking) {
        xTimerStop(led_state.blink_timer, 0);
        led_state.blinking = false;

        // Restore LED to off state
        led_set_hardware(led_state.blink_position, &LED_COLOR_OFF);

        ESP_LOGD(TAG, "Stopped LED blinking");
    }

    return ESP_OK;
}

bool led_is_initialized(void)
{
    return led_state.initialized;
}
