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

/* Convert a pulse width to a PCA9685 count at the frequency the chip is
 * ACTUALLY running at.
 *
 * This used to be SERVO_PULSE_TO_COUNT(), which divided by a hardcoded
 * SERVO_PERIOD_US of 5000 — correct only at 200 Hz. That constant and the
 * chip's prescaler were two independent copies of the same fact, so changing
 * the frequency silently produced pulses several times too long: at 50 Hz the
 * macro's "1500 us" count is a 6 ms pulse, well outside any servo's range. */
static uint16_t pulse_to_count(uint16_t pulse_us)
{
    const uint16_t hz = i2c_bus_pca9685_frequency();
    const uint32_t period_us = (hz > 0u) ? (1000000u / hz) : SERVO_PERIOD_US;
    uint32_t count = ((uint32_t)pulse_us * (PCA9685_PWM_MAX + 1u)) / period_us;

    if (count > PCA9685_PWM_MAX) {
        count = PCA9685_PWM_MAX;
    }
    return (uint16_t)count;
}

uint16_t servo_angle_to_count(servo_id_t id, int16_t angle)
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

    return pulse_to_count(pulse_us);
}

static uint16_t angle_to_count(servo_id_t id, int16_t angle)
{
    return servo_angle_to_count(id, angle);
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

    // Centering goes through servo_set_angle(), which refuses with
    // ESP_ERR_INVALID_STATE until the module is initialised. Raise the flag
    // first and lower it again if the bus write fails, so a failed init can be
    // retried and the servos are never reported as ready when they are not.
    servo_state.initialized = true;
    esp_err_t ret = servo_center_all();
    if (ret != ESP_OK) {
        servo_state.initialized = false;
        ESP_LOGE(TAG, "Failed to center servos: %s", esp_err_to_name(ret));
        return ret;
    }

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
    const bool *enabled;
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
    /* step_size = 0 would never advance current and the loop would never exit. */
    if (step_size == 0)
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

/* Bench bring-up gesture: shake the head, then nod.
 *
 * Exists because "the servos are not moving" has at least four causes that look
 * identical from across the room — no V+ on the PCA9685 (VCC powers only the
 * logic), a failed init, a pulse train outside the servo's frame rate, and a
 * dead servo — and the console could not tell them apart. Every step logs the
 * angle, the PCA9685 count written, and the bus result, so a servo that does
 * not move while the writes succeed is a different diagnosis from one whose
 * writes are failing.
 *
 * Blocking, by several seconds. Called on peripheral_task, never from the
 * console task. */
esp_err_t servo_exercise(void)
{
    if (!servo_state.initialized) {
        ESP_LOGE(TAG, "exercise: servos not initialised — nothing was written");
        return ESP_ERR_INVALID_STATE;
    }

    static const struct {
        servo_id_t id;
        int16_t angle;
        const char *what;
    } k_steps[] = {
        {SERVO_PAN, SERVO_PAN_CENTER, "centre"},
        {SERVO_PAN, SERVO_PAN_MIN_ANGLE, "look left"},
        {SERVO_PAN, SERVO_PAN_MAX_ANGLE, "look right"},
        {SERVO_PAN, SERVO_PAN_CENTER, "centre"},
        {SERVO_TILT, SERVO_TILT_CENTER, "centre"},
        {SERVO_TILT, SERVO_TILT_MAX_ANGLE, "nod up"},
        {SERVO_TILT, SERVO_TILT_MIN_ANGLE, "nod down"},
        {SERVO_TILT, SERVO_TILT_MAX_ANGLE, "nod up"},
        {SERVO_TILT, SERVO_TILT_MIN_ANGLE, "nod down"},
        {SERVO_TILT, SERVO_TILT_CENTER, "centre"},
    };

    ESP_LOGI(TAG, "exercise: PCA9685 at %u Hz", (unsigned)i2c_bus_pca9685_frequency());

    esp_err_t first_error = ESP_OK;
    for (size_t i = 0; i < sizeof(k_steps) / sizeof(k_steps[0]); ++i) {
        const uint16_t count = servo_angle_to_count(k_steps[i].id, k_steps[i].angle);
        const esp_err_t ret = servo_set_angle(k_steps[i].id, k_steps[i].angle);

        ESP_LOGI(TAG, "exercise: %-10s %s=%+d deg -> count %u  %s", k_steps[i].what,
                 (k_steps[i].id == SERVO_PAN) ? "pan" : "tilt", k_steps[i].angle, (unsigned)count,
                 esp_err_to_name(ret));

        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_EXERCISE_STEP_MS));
    }

    if (first_error == ESP_OK) {
        ESP_LOGI(TAG,
                 "exercise: every write succeeded. If nothing moved, the fault is downstream of "
                 "the PCA9685 — check V+ (VCC powers only the logic), the servo leads, and "
                 "whether these servos track pulses at %u Hz (`servo freq 50`).",
                 (unsigned)i2c_bus_pca9685_frequency());
    }
    return first_error;
}

esp_err_t servo_stop_motion(void)
{
    if (servo_state.motion_task) {
        servo_state.motion_active = false;
    }
    return ESP_OK;
}
