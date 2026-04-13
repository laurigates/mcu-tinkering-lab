/**
 * @file servo_controller.c
 * @brief Servo control via PCA9685 at 200Hz through TCA9548A I2C bus
 *
 * At 200Hz (5ms period), pulse width range 500-2500us maps to:
 *   PCA9685 count = (pulse_us * 4096) / 5000
 */

#include "servo_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "pin_config.h"

static const char *TAG = "servo_controller";

static struct {
    bool initialized;
    int16_t pan_angle;
    int16_t tilt_angle;
    bool pan_enabled;
    bool tilt_enabled;
    TaskHandle_t motion_task;
    bool motion_active;
} servo_state = {0};

static uint16_t angle_to_count(servo_id_t id, int16_t angle)
{
    int16_t min_angle = (id == SERVO_PAN) ? SERVO_PAN_MIN_ANGLE : SERVO_TILT_MIN_ANGLE;
    int16_t max_angle = (id == SERVO_PAN) ? SERVO_PAN_MAX_ANGLE : SERVO_TILT_MAX_ANGLE;

    if (angle < min_angle)
        angle = min_angle;
    if (angle > max_angle)
        angle = max_angle;

    float normalized = (float)(angle - min_angle) / (max_angle - min_angle);
    uint16_t pulse_us =
        SERVO_MIN_PULSE_US + (uint16_t)(normalized * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));

    return SERVO_PULSE_TO_COUNT(pulse_us);
}

esp_err_t servo_controller_init(void)
{
    if (servo_state.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing servo controller (PCA9685 @ 200Hz)");

    servo_state.pan_angle = SERVO_PAN_CENTER;
    servo_state.tilt_angle = SERVO_TILT_CENTER;
    servo_state.pan_enabled = true;
    servo_state.tilt_enabled = true;
    servo_state.motion_active = false;

    esp_err_t ret = servo_center_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to center servos");
        return ret;
    }

    servo_state.initialized = true;
    ESP_LOGI(TAG, "Servo controller initialized");
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_id_t servo_id, int16_t angle)
{
    if (!servo_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (servo_id != SERVO_PAN && servo_id != SERVO_TILT)
        return ESP_ERR_INVALID_ARG;
    if (!servo_is_angle_valid(servo_id, angle))
        return ESP_ERR_INVALID_ARG;

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

    if (!*enabled)
        return ESP_ERR_INVALID_STATE;

    uint16_t count = angle_to_count(servo_id, angle);
    esp_err_t ret = i2c_bus_pca9685_set(channel, count);

    if (ret == ESP_OK) {
        *current_angle = angle;
        ESP_LOGD(TAG, "Servo %d -> %d° (count=%d)", servo_id, angle, count);
    }
    return ret;
}

esp_err_t servo_set_pan(int16_t angle)
{
    return servo_set_angle(SERVO_PAN, angle);
}
esp_err_t servo_set_tilt(int16_t angle)
{
    return servo_set_angle(SERVO_TILT, angle);
}

esp_err_t servo_set_position(int16_t pan_angle, int16_t tilt_angle)
{
    esp_err_t ret = servo_set_pan(pan_angle);
    if (ret != ESP_OK)
        return ret;
    return servo_set_tilt(tilt_angle);
}

esp_err_t servo_center(servo_id_t servo_id)
{
    int16_t center = (servo_id == SERVO_PAN) ? SERVO_PAN_CENTER : SERVO_TILT_CENTER;
    return servo_set_angle(servo_id, center);
}

esp_err_t servo_center_all(void)
{
    esp_err_t ret = servo_center(SERVO_PAN);
    if (ret != ESP_OK)
        return ret;
    return servo_center(SERVO_TILT);
}

esp_err_t servo_get_angle(servo_id_t servo_id, int16_t *angle)
{
    if (!servo_state.initialized || !angle)
        return ESP_ERR_INVALID_STATE;
    if (servo_id == SERVO_PAN)
        *angle = servo_state.pan_angle;
    else if (servo_id == SERVO_TILT)
        *angle = servo_state.tilt_angle;
    else
        return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

esp_err_t servo_get_position(servo_position_t *position)
{
    if (!servo_state.initialized || !position)
        return ESP_ERR_INVALID_STATE;
    position->pan_angle = servo_state.pan_angle;
    position->tilt_angle = servo_state.tilt_angle;
    return ESP_OK;
}

esp_err_t servo_disable(servo_id_t servo_id)
{
    if (!servo_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint8_t channel = (servo_id == SERVO_PAN) ? SERVO_PAN_CHANNEL : SERVO_TILT_CHANNEL;
    esp_err_t ret = i2c_bus_pca9685_set(channel, 0);

    if (ret == ESP_OK) {
        if (servo_id == SERVO_PAN)
            servo_state.pan_enabled = false;
        else
            servo_state.tilt_enabled = false;
    }
    return ret;
}

esp_err_t servo_disable_all(void)
{
    esp_err_t ret = servo_disable(SERVO_PAN);
    if (ret != ESP_OK)
        return ret;
    return servo_disable(SERVO_TILT);
}

esp_err_t servo_enable(servo_id_t servo_id)
{
    if (!servo_state.initialized)
        return ESP_ERR_INVALID_STATE;

    if (servo_id == SERVO_PAN) {
        servo_state.pan_enabled = true;
        return servo_set_angle(SERVO_PAN, servo_state.pan_angle);
    } else {
        servo_state.tilt_enabled = true;
        return servo_set_angle(SERVO_TILT, servo_state.tilt_angle);
    }
}

esp_err_t servo_enable_all(void)
{
    esp_err_t ret = servo_enable(SERVO_PAN);
    if (ret != ESP_OK)
        return ret;
    return servo_enable(SERVO_TILT);
}

bool servo_is_initialized(void)
{
    return servo_state.initialized;
}

bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle)
{
    if (servo_id == SERVO_PAN)
        return angle >= SERVO_PAN_MIN_ANGLE && angle <= SERVO_PAN_MAX_ANGLE;
    if (servo_id == SERVO_TILT)
        return angle >= SERVO_TILT_MIN_ANGLE && angle <= SERVO_TILT_MAX_ANGLE;
    return false;
}

esp_err_t servo_move_smooth(servo_id_t servo_id, int16_t target_angle, uint8_t step_size,
                            uint32_t delay_ms)
{
    if (!servo_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!servo_is_angle_valid(servo_id, target_angle))
        return ESP_ERR_INVALID_ARG;

    int16_t current;
    esp_err_t ret = servo_get_angle(servo_id, &current);
    if (ret != ESP_OK)
        return ret;

    while (current != target_angle) {
        if (current < target_angle) {
            current += step_size;
            if (current > target_angle)
                current = target_angle;
        } else {
            current -= step_size;
            if (current < target_angle)
                current = target_angle;
        }
        ret = servo_set_angle(servo_id, current);
        if (ret != ESP_OK)
            return ret;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return ESP_OK;
}

esp_err_t servo_sweep(servo_id_t servo_id, int16_t start_angle, int16_t end_angle,
                      uint8_t step_size, uint32_t delay_ms, uint32_t cycles)
{
    if (!servo_state.initialized)
        return ESP_ERR_INVALID_STATE;
    if (!servo_is_angle_valid(servo_id, start_angle) || !servo_is_angle_valid(servo_id, end_angle))
        return ESP_ERR_INVALID_ARG;

    uint32_t count = 0;
    while (cycles == 0 || count < cycles) {
        esp_err_t ret = servo_move_smooth(servo_id, end_angle, step_size, delay_ms);
        if (ret != ESP_OK)
            return ret;
        ret = servo_move_smooth(servo_id, start_angle, step_size, delay_ms);
        if (ret != ESP_OK)
            return ret;
        count++;
    }
    return ESP_OK;
}

esp_err_t servo_stop_motion(void)
{
    if (servo_state.motion_task) {
        servo_state.motion_active = false;
    }
    return ESP_OK;
}
