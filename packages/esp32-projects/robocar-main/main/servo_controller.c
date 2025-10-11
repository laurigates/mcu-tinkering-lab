/**
 * @file servo_controller.c
 * @brief Hardware abstraction layer for servo motor control (pan/tilt camera mount)
 */

#include "servo_controller.h"
#include "pin_config_idf.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <math.h>

static const char *TAG = "servo_controller";

// PCA9685 servo channels (assuming channels 6 and 7 for pan/tilt)
#define SERVO_PAN_CHANNEL  6
#define SERVO_TILT_CHANNEL 7

// Servo PWM characteristics
#define SERVO_PWM_FREQ_HZ    50      // Standard servo frequency
#define SERVO_PWM_MIN_PULSE  500     // 0.5ms minimum pulse (0 degrees)
#define SERVO_PWM_MAX_PULSE  2500    // 2.5ms maximum pulse (180 degrees)
#define SERVO_PWM_CENTER     1500    // 1.5ms center pulse (90 degrees)

// PCA9685 specific values for servo control
#define PCA9685_PWM_FULL     4096    // 12-bit resolution
#define PCA9685_PULSE_LENGTH (1000000 / SERVO_PWM_FREQ_HZ)  // 20ms in microseconds

// Servo controller state
static struct {
    bool initialized;
    int16_t pan_angle;
    int16_t tilt_angle;
    bool pan_enabled;
    bool tilt_enabled;
    TaskHandle_t motion_task;
    bool motion_active;
} servo_state = {0};

/**
 * @brief Convert angle to PWM pulse width in microseconds
 */
static uint16_t angle_to_pulse_us(servo_id_t servo_id, int16_t angle) {
    int16_t min_angle, max_angle;
    
    if (servo_id == SERVO_PAN) {
        min_angle = SERVO_PAN_MIN_ANGLE;
        max_angle = SERVO_PAN_MAX_ANGLE;
    } else {
        min_angle = SERVO_TILT_MIN_ANGLE;
        max_angle = SERVO_TILT_MAX_ANGLE;
    }
    
    // Clamp angle to valid range
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;
    
    // Map angle to pulse width
    // Standard servos: -90° = 0.5ms, 0° = 1.5ms, +90° = 2.5ms
    float normalized = (float)(angle - min_angle) / (max_angle - min_angle);
    uint16_t pulse_us = SERVO_PWM_MIN_PULSE + (uint16_t)(normalized * (SERVO_PWM_MAX_PULSE - SERVO_PWM_MIN_PULSE));
    
    return pulse_us;
}

/**
 * @brief Convert pulse width to PCA9685 PWM value
 */
static uint16_t pulse_us_to_pwm(uint16_t pulse_us) {
    return (pulse_us * PCA9685_PWM_FULL) / PCA9685_PULSE_LENGTH;
}

/**
 * @brief Set servo PWM via PCA9685
 */
static esp_err_t servo_set_pwm(uint8_t channel, uint16_t pulse_us) {
    uint16_t pwm_value = pulse_us_to_pwm(pulse_us);
    
    // Write to PCA9685 registers (simplified - actual implementation would use proper I2C)
    uint8_t reg_base = 0x06 + (channel * 4); // LED0_ON_L + channel offset
    
    uint8_t write_buf[5] = {
        reg_base,
        0,                    // ON_L (always 0)
        0,                    // ON_H (always 0)
        pwm_value & 0xFF,     // OFF_L
        (pwm_value >> 8) & 0xFF // OFF_H
    };
    
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_1, PCA9685_I2C_ADDR,
                                              write_buf, sizeof(write_buf),
                                              pdMS_TO_TICKS(100));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo PWM: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Smooth motion task
 */
static void servo_motion_task(void *pvParameters) {
    // This would implement smooth servo movements and sweeps
    // For simplicity, this is a placeholder
    while (servo_state.motion_active) {
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
    }
    servo_state.motion_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t servo_controller_init(void) {
    if (servo_state.initialized) {
        ESP_LOGW(TAG, "Servo controller already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing servo controller");
    
    // Initialize servo positions to center
    servo_state.pan_angle = SERVO_PAN_CENTER;
    servo_state.tilt_angle = SERVO_TILT_CENTER;
    servo_state.pan_enabled = true;
    servo_state.tilt_enabled = true;
    servo_state.motion_active = false;
    
    // Set servos to center position
    esp_err_t ret = servo_center_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to center servos during initialization");
        return ret;
    }
    
    servo_state.initialized = true;
    ESP_LOGI(TAG, "Servo controller initialized successfully");
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_id_t servo_id, int16_t angle) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!servo_is_angle_valid(servo_id, angle)) {
        ESP_LOGW(TAG, "Invalid angle %d for servo %d", angle, servo_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t channel;
    bool *enabled;
    int16_t *current_angle;
    
    if (servo_id == SERVO_PAN) {
        channel = SERVO_PAN_CHANNEL;
        enabled = &servo_state.pan_enabled;
        current_angle = &servo_state.pan_angle;
    } else {
        channel = SERVO_TILT_CHANNEL;
        enabled = &servo_state.tilt_enabled;
        current_angle = &servo_state.tilt_angle;
    }
    
    if (!*enabled) {
        ESP_LOGW(TAG, "Servo %d is disabled", servo_id);
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t pulse_us = angle_to_pulse_us(servo_id, angle);
    esp_err_t ret = servo_set_pwm(channel, pulse_us);
    
    if (ret == ESP_OK) {
        *current_angle = angle;
        ESP_LOGD(TAG, "Set servo %d to angle %d (pulse %d us)", servo_id, angle, pulse_us);
    }
    
    return ret;
}

esp_err_t servo_set_pan(int16_t angle) {
    return servo_set_angle(SERVO_PAN, angle);
}

esp_err_t servo_set_tilt(int16_t angle) {
    return servo_set_angle(SERVO_TILT, angle);
}

esp_err_t servo_set_position(int16_t pan_angle, int16_t tilt_angle) {
    esp_err_t ret = servo_set_pan(pan_angle);
    ret |= servo_set_tilt(tilt_angle);
    return ret;
}

esp_err_t servo_center(servo_id_t servo_id) {
    int16_t center_angle = (servo_id == SERVO_PAN) ? SERVO_PAN_CENTER : SERVO_TILT_CENTER;
    return servo_set_angle(servo_id, center_angle);
}

esp_err_t servo_center_all(void) {
    esp_err_t ret = servo_center(SERVO_PAN);
    ret |= servo_center(SERVO_TILT);
    return ret;
}

esp_err_t servo_get_angle(servo_id_t servo_id, int16_t *angle) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!angle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (servo_id == SERVO_PAN) {
        *angle = servo_state.pan_angle;
    } else if (servo_id == SERVO_TILT) {
        *angle = servo_state.tilt_angle;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t servo_get_position(servo_position_t *position) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!position) {
        return ESP_ERR_INVALID_ARG;
    }
    
    position->pan_angle = servo_state.pan_angle;
    position->tilt_angle = servo_state.tilt_angle;
    
    return ESP_OK;
}

esp_err_t servo_disable(servo_id_t servo_id) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t channel = (servo_id == SERVO_PAN) ? SERVO_PAN_CHANNEL : SERVO_TILT_CHANNEL;
    
    // Set PWM to 0 to disable servo
    esp_err_t ret = servo_set_pwm(channel, 0);
    
    if (ret == ESP_OK) {
        if (servo_id == SERVO_PAN) {
            servo_state.pan_enabled = false;
        } else {
            servo_state.tilt_enabled = false;
        }
        ESP_LOGD(TAG, "Disabled servo %d", servo_id);
    }
    
    return ret;
}

esp_err_t servo_disable_all(void) {
    esp_err_t ret = servo_disable(SERVO_PAN);
    ret |= servo_disable(SERVO_TILT);
    return ret;
}

esp_err_t servo_enable(servo_id_t servo_id) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (servo_id == SERVO_PAN) {
        servo_state.pan_enabled = true;
        return servo_set_angle(SERVO_PAN, servo_state.pan_angle);
    } else {
        servo_state.tilt_enabled = true;
        return servo_set_angle(SERVO_TILT, servo_state.tilt_angle);
    }
}

esp_err_t servo_enable_all(void) {
    esp_err_t ret = servo_enable(SERVO_PAN);
    ret |= servo_enable(SERVO_TILT);
    return ret;
}

bool servo_is_initialized(void) {
    return servo_state.initialized;
}

bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle) {
    if (servo_id == SERVO_PAN) {
        return (angle >= SERVO_PAN_MIN_ANGLE && angle <= SERVO_PAN_MAX_ANGLE);
    } else if (servo_id == SERVO_TILT) {
        return (angle >= SERVO_TILT_MIN_ANGLE && angle <= SERVO_TILT_MAX_ANGLE);
    }
    return false;
}

esp_err_t servo_move_smooth(servo_id_t servo_id, int16_t target_angle, 
                           uint8_t step_size, uint32_t delay_ms) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!servo_is_angle_valid(servo_id, target_angle)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t current_angle;
    esp_err_t ret = servo_get_angle(servo_id, &current_angle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Simple smooth movement implementation
    while (current_angle != target_angle) {
        if (current_angle < target_angle) {
            current_angle += step_size;
            if (current_angle > target_angle) current_angle = target_angle;
        } else {
            current_angle -= step_size;
            if (current_angle < target_angle) current_angle = target_angle;
        }
        
        ret = servo_set_angle(servo_id, current_angle);
        if (ret != ESP_OK) {
            return ret;
        }
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    return ESP_OK;
}

esp_err_t servo_sweep(servo_id_t servo_id, int16_t start_angle, int16_t end_angle,
                     uint8_t step_size, uint32_t delay_ms, uint32_t cycles) {
    if (!servo_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!servo_is_angle_valid(servo_id, start_angle) || 
        !servo_is_angle_valid(servo_id, end_angle)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // This would be better implemented as a background task
    // For now, simple blocking implementation
    uint32_t cycle_count = 0;
    
    while (cycles == 0 || cycle_count < cycles) {
        // Move from start to end
        esp_err_t ret = servo_move_smooth(servo_id, end_angle, step_size, delay_ms);
        if (ret != ESP_OK) return ret;
        
        // Move from end to start
        ret = servo_move_smooth(servo_id, start_angle, step_size, delay_ms);
        if (ret != ESP_OK) return ret;
        
        cycle_count++;
    }
    
    return ESP_OK;
}

esp_err_t servo_stop_motion(void) {
    if (servo_state.motion_task) {
        servo_state.motion_active = false;
        // Task will clean itself up
    }
    return ESP_OK;
}