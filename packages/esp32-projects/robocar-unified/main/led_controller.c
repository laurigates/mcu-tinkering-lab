/**
 * @file led_controller.c
 * @brief RGB LED control via PCA9685 through TCA9548A I2C bus
 */

#include "led_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "i2c_bus.h"
#include "pin_config.h"

static const char *TAG = "led_controller";

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

const rgb_color_t LED_COLOR_OFF = {0, 0, 0};
const rgb_color_t LED_COLOR_RED = {255, 0, 0};
const rgb_color_t LED_COLOR_GREEN = {0, 255, 0};
const rgb_color_t LED_COLOR_BLUE = {0, 0, 255};
const rgb_color_t LED_COLOR_WHITE = {255, 255, 255};
const rgb_color_t LED_COLOR_YELLOW = {255, 255, 0};
const rgb_color_t LED_COLOR_PURPLE = {255, 0, 255};
const rgb_color_t LED_COLOR_CYAN = {0, 255, 255};

static uint16_t color_to_pwm(uint8_t color_value)
{
    return (uint16_t)((uint32_t)color_value * PCA9685_PWM_MAX / 255);
}

static esp_err_t led_set_hardware(led_position_t position, const rgb_color_t *color)
{
    if (!color)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ESP_OK;

    if (position == LED_LEFT || position == LED_BOTH) {
        uint16_t vals[3] = {color_to_pwm(color->red), color_to_pwm(color->green),
                            color_to_pwm(color->blue)};
        ret = i2c_bus_pca9685_set_multi(LED_LEFT_R_CHANNEL, 3, vals);
        if (ret == ESP_OK)
            led_state.left_color = *color;
    }

    if ((position == LED_RIGHT || position == LED_BOTH) && ret == ESP_OK) {
        uint16_t vals[3] = {color_to_pwm(color->red), color_to_pwm(color->green),
                            color_to_pwm(color->blue)};
        ret = i2c_bus_pca9685_set_multi(LED_RIGHT_R_CHANNEL, 3, vals);
        if (ret == ESP_OK)
            led_state.right_color = *color;
    }

    return ret;
}

static void blink_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    static bool blink_on = false;

    if (!led_state.blinking)
        return;

    const rgb_color_t *target = blink_on ? &led_state.blink_color : &led_state.blink_off_color;
    led_set_hardware(led_state.blink_position, target);
    blink_on = !blink_on;

    if (led_state.blink_count_remaining > 0 && blink_on) {
        led_state.blink_count_remaining--;
        if (led_state.blink_count_remaining == 0) {
            led_stop_blink();
        }
    }
}

esp_err_t led_controller_init(void)
{
    if (led_state.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing LED controller (PCA9685 via TCA9548A)");

    led_state.blink_timer =
        xTimerCreate("led_blink", pdMS_TO_TICKS(500), pdTRUE, NULL, blink_timer_callback);
    if (!led_state.blink_timer) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        return ESP_ERR_NO_MEM;
    }

    led_state.left_color = LED_COLOR_OFF;
    led_state.right_color = LED_COLOR_OFF;
    led_state.blinking = false;

    led_turn_off_all();

    led_state.initialized = true;
    ESP_LOGI(TAG, "LED controller initialized");
    return ESP_OK;
}

esp_err_t led_set_color(led_position_t position, const rgb_color_t *color)
{
    if (!led_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!color)
        return ESP_ERR_INVALID_ARG;
    if (led_state.blinking)
        led_stop_blink();
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
    // PCA9685 changes are immediate; no-op for API compatibility
    return led_state.initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t led_get_color(led_position_t position, rgb_color_t *color)
{
    if (!led_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!color)
        return ESP_ERR_INVALID_ARG;

    if (position == LED_LEFT)
        *color = led_state.left_color;
    else if (position == LED_RIGHT)
        *color = led_state.right_color;
    else
        return ESP_ERR_INVALID_ARG;

    return ESP_OK;
}

esp_err_t led_blink(led_position_t position, const rgb_color_t *color, uint32_t on_time_ms,
                    uint32_t off_time_ms, uint32_t blink_count)
{
    if (!led_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!color || position == LED_BOTH)
        return ESP_ERR_INVALID_ARG;

    led_stop_blink();

    led_state.blink_position = position;
    led_state.blink_color = *color;
    led_state.blink_off_color = LED_COLOR_OFF;
    led_state.blink_count_remaining = blink_count;
    led_state.blinking = true;

    uint32_t period = (on_time_ms + off_time_ms) / 2;
    if (period < 10)
        period = 10;

    xTimerChangePeriod(led_state.blink_timer, pdMS_TO_TICKS(period), 0);
    xTimerStart(led_state.blink_timer, 0);

    return ESP_OK;
}

esp_err_t led_stop_blink(void)
{
    if (!led_state.initialized)
        return ESP_ERR_INVALID_STATE;

    if (led_state.blinking) {
        xTimerStop(led_state.blink_timer, 0);
        led_state.blinking = false;
        led_set_hardware(led_state.blink_position, &LED_COLOR_OFF);
    }
    return ESP_OK;
}

bool led_is_initialized(void)
{
    return led_state.initialized;
}
