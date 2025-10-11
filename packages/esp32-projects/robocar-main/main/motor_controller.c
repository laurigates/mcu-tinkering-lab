/**
 * @file motor_controller.c
 * @brief Hardware abstraction layer for TB6612FNG motor control
 */

#include "motor_controller.h"
#include "pin_config_idf.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "motor_controller";

// Current motor state
static struct {
    uint8_t left_speed;
    uint8_t right_speed;
    uint8_t left_direction;  // 1 = forward, 0 = backward
    uint8_t right_direction; // 1 = forward, 0 = backward
    bool initialized;
} motor_state = {0};

/**
 * @brief Set motor speed using LEDC PWM
 */
static esp_err_t set_motor_speed(ledc_channel_t channel, uint8_t speed) {
    uint32_t duty = (speed * ((1 << PWM_RESOLUTION) - 1)) / 255;
    return ledc_set_duty(PWM_MODE, channel, duty) || ledc_update_duty(PWM_MODE, channel);
}

esp_err_t motor_controller_init(void) {
    if (motor_state.initialized) {
        ESP_LOGW(TAG, "Motor controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing motor controller");

    // Configure GPIO pins for motor direction control
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RIGHT_IN1_PIN) | (1ULL << RIGHT_IN2_PIN) |
                       (1ULL << LEFT_IN1_PIN) | (1ULL << LEFT_IN2_PIN) | 
                       (1ULL << STBY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Configure PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure PWM channels for motors
    ledc_channel_config_t right_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_RIGHT_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = RIGHT_PWMA_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_channel));

    ledc_channel_config_t left_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_LEFT_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEFT_PWMB_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_channel));

    // Enable motor driver (STBY pin high)
    gpio_set_level(STBY_PIN, 1);

    // Initialize motors to stopped state
    motor_stop();

    motor_state.initialized = true;
    ESP_LOGI(TAG, "Motor controller initialized successfully");
    return ESP_OK;
}

esp_err_t motor_move_forward(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for forward movement
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    // Set motor speeds
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed);

    // Update state
    motor_state.left_speed = speed;
    motor_state.right_speed = speed;
    motor_state.left_direction = 1;
    motor_state.right_direction = 1;

    ESP_LOGD(TAG, "Moving forward at speed %d", speed);
    return ret;
}

esp_err_t motor_move_backward(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for backward movement
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 1);
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 1);

    // Set motor speeds
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed);

    // Update state
    motor_state.left_speed = speed;
    motor_state.right_speed = speed;
    motor_state.left_direction = 0;
    motor_state.right_direction = 0;

    ESP_LOGD(TAG, "Moving backward at speed %d", speed);
    return ret;
}

esp_err_t motor_turn_left(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for left turn (right motor faster)
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    // Right motor full speed, left motor half speed
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed / 2);

    // Update state
    motor_state.left_speed = speed / 2;
    motor_state.right_speed = speed;
    motor_state.left_direction = 1;
    motor_state.right_direction = 1;

    ESP_LOGD(TAG, "Turning left at speed %d", speed);
    return ret;
}

esp_err_t motor_turn_right(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for right turn (left motor faster)
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    // Left motor full speed, right motor half speed
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed / 2);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed);

    // Update state
    motor_state.left_speed = speed;
    motor_state.right_speed = speed / 2;
    motor_state.left_direction = 1;
    motor_state.right_direction = 1;

    ESP_LOGD(TAG, "Turning right at speed %d", speed);
    return ret;
}

esp_err_t motor_rotate_cw(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for clockwise rotation (right backward, left forward)
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 1);
    gpio_set_level(LEFT_IN1_PIN, 1);
    gpio_set_level(LEFT_IN2_PIN, 0);

    // Set motor speeds
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed);

    // Update state
    motor_state.left_speed = speed;
    motor_state.right_speed = speed;
    motor_state.left_direction = 1;
    motor_state.right_direction = 0;

    ESP_LOGD(TAG, "Rotating clockwise at speed %d", speed);
    return ret;
}

esp_err_t motor_rotate_ccw(uint8_t speed) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set direction pins for counter-clockwise rotation (right forward, left backward)
    gpio_set_level(RIGHT_IN1_PIN, 1);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 1);

    // Set motor speeds
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, speed);

    // Update state
    motor_state.left_speed = speed;
    motor_state.right_speed = speed;
    motor_state.left_direction = 0;
    motor_state.right_direction = 1;

    ESP_LOGD(TAG, "Rotating counter-clockwise at speed %d", speed);
    return ret;
}

esp_err_t motor_stop(void) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set all direction pins low (brake mode)
    gpio_set_level(RIGHT_IN1_PIN, 0);
    gpio_set_level(RIGHT_IN2_PIN, 0);
    gpio_set_level(LEFT_IN1_PIN, 0);
    gpio_set_level(LEFT_IN2_PIN, 0);

    // Set PWM to 0
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, 0);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, 0);

    // Update state
    motor_state.left_speed = 0;
    motor_state.right_speed = 0;
    motor_state.left_direction = 0;
    motor_state.right_direction = 0;

    ESP_LOGD(TAG, "Motors stopped");
    return ret;
}

esp_err_t motor_set_individual(uint8_t left_speed, uint8_t right_speed,
                              uint8_t left_direction, uint8_t right_direction) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set right motor direction
    if (right_direction) {
        gpio_set_level(RIGHT_IN1_PIN, 1);
        gpio_set_level(RIGHT_IN2_PIN, 0);
    } else {
        gpio_set_level(RIGHT_IN1_PIN, 0);
        gpio_set_level(RIGHT_IN2_PIN, 1);
    }

    // Set left motor direction
    if (left_direction) {
        gpio_set_level(LEFT_IN1_PIN, 1);
        gpio_set_level(LEFT_IN2_PIN, 0);
    } else {
        gpio_set_level(LEFT_IN1_PIN, 0);
        gpio_set_level(LEFT_IN2_PIN, 1);
    }

    // Set motor speeds
    esp_err_t ret = set_motor_speed(PWM_RIGHT_CHANNEL, right_speed);
    ret |= set_motor_speed(PWM_LEFT_CHANNEL, left_speed);

    // Update state
    motor_state.left_speed = left_speed;
    motor_state.right_speed = right_speed;
    motor_state.left_direction = left_direction;
    motor_state.right_direction = right_direction;

    ESP_LOGD(TAG, "Set individual motors: L=%d/%d R=%d/%d", 
             left_speed, left_direction, right_speed, right_direction);
    return ret;
}

esp_err_t motor_get_state(uint8_t *left_speed, uint8_t *right_speed,
                         uint8_t *left_direction, uint8_t *right_direction) {
    if (!motor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (left_speed) *left_speed = motor_state.left_speed;
    if (right_speed) *right_speed = motor_state.right_speed;
    if (left_direction) *left_direction = motor_state.left_direction;
    if (right_direction) *right_direction = motor_state.right_direction;

    return ESP_OK;
}