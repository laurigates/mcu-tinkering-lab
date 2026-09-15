/**
 * @file servo_controller.c
 * @brief Servo control via PCA9685 through TCA9548A I2C bus
 *
 * An angle is a real servo angle about centre:
 *   pulse_us = SERVO_CENTER_PULSE_US + angle * SERVO_PULSE_US_PER_90_DEG / 90
 * clamped to [SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US], the SG90 datasheet
 * range. The PCA9685 count is that pulse as a fraction of the period the chip
 * is actually running at.
 */

#include "servo_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "pin_config.h"

static const char *TAG = "servo_controller";

/* The angles at which the pulse reaches the datasheet range: the widest travel
 * limits servo_set_limits() accepts. -90 and +81 with the shipped constants. */
#define SERVO_ANGLE_HW_MIN                                                           \
    ((int16_t)(((int32_t)SERVO_MIN_PULSE_US - (int32_t)SERVO_CENTER_PULSE_US) * 90 / \
               (int32_t)SERVO_PULSE_US_PER_90_DEG))
#define SERVO_ANGLE_HW_MAX                                                           \
    ((int16_t)(((int32_t)SERVO_MAX_PULSE_US - (int32_t)SERVO_CENTER_PULSE_US) * 90 / \
               (int32_t)SERVO_PULSE_US_PER_90_DEG))

_Static_assert(SERVO_PAN_LIMIT_MIN_DEFAULT >= SERVO_ANGLE_HW_MIN &&
                   SERVO_PAN_LIMIT_MAX_DEFAULT <= SERVO_ANGLE_HW_MAX &&
                   SERVO_PAN_LIMIT_MIN_DEFAULT <= SERVO_PAN_CENTER &&
                   SERVO_PAN_CENTER <= SERVO_PAN_LIMIT_MAX_DEFAULT,
               "pan limit defaults must lie inside the SG90 pulse range and contain centre");
_Static_assert(SERVO_TILT_LIMIT_MIN_DEFAULT >= SERVO_ANGLE_HW_MIN &&
                   SERVO_TILT_LIMIT_MAX_DEFAULT <= SERVO_ANGLE_HW_MAX &&
                   SERVO_TILT_LIMIT_MIN_DEFAULT <= SERVO_TILT_CENTER &&
                   SERVO_TILT_CENTER <= SERVO_TILT_LIMIT_MAX_DEFAULT,
               "tilt limit defaults must lie inside the SG90 pulse range and contain centre");

static struct {
    bool initialized;
    int16_t pan_angle;
    int16_t tilt_angle;
    bool pan_enabled;
    bool tilt_enabled;
    /* Live travel limits. Initialised statically so they gate angles before
     * init as well as after. */
    int16_t pan_min;
    int16_t pan_max;
    int16_t tilt_min;
    int16_t tilt_max;
    TaskHandle_t motion_task;
    bool motion_active;
} servo_state = {
    .pan_min = SERVO_PAN_LIMIT_MIN_DEFAULT,
    .pan_max = SERVO_PAN_LIMIT_MAX_DEFAULT,
    .tilt_min = SERVO_TILT_LIMIT_MIN_DEFAULT,
    .tilt_max = SERVO_TILT_LIMIT_MAX_DEFAULT,
};

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

/* The angle used to be stretched across the whole pulse span between the
 * configured min and max, so tilt's "±45 deg" was really the servo's full
 * ±90 deg travel and pan's +90 deg asked for 2500 us, past the SG90's 2400 us
 * ceiling. Both stalled the head against its end stops. */
static uint16_t angle_to_pulse_us(int16_t angle)
{
    int32_t pulse =
        (int32_t)SERVO_CENTER_PULSE_US + ((int32_t)angle * (int32_t)SERVO_PULSE_US_PER_90_DEG) / 90;

    if (pulse < (int32_t)SERVO_MIN_PULSE_US) {
        pulse = SERVO_MIN_PULSE_US;
    }
    if (pulse > (int32_t)SERVO_MAX_PULSE_US) {
        pulse = SERVO_MAX_PULSE_US;
    }
    return (uint16_t)pulse;
}

uint16_t servo_angle_to_count(servo_id_t id, int16_t angle)
{
    (void)id;
    return pulse_to_count(angle_to_pulse_us(angle));
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

    ESP_LOGI(TAG, "Initializing servo controller (PCA9685 @ %uHz)",
             (unsigned)i2c_bus_pca9685_frequency());

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

    ESP_LOGI(TAG, "Servo controller initialized (limits pan %+d..%+d, tilt %+d..%+d deg)",
             servo_state.pan_min, servo_state.pan_max, servo_state.tilt_min, servo_state.tilt_max);
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

bool servo_is_enabled(servo_id_t servo_id)
{
    if (!servo_state.initialized)
        return false;
    if (servo_id == SERVO_PAN)
        return servo_state.pan_enabled;
    if (servo_id == SERVO_TILT)
        return servo_state.tilt_enabled;
    return false;
}

bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle)
{
    if (servo_id == SERVO_PAN)
        return angle >= servo_state.pan_min && angle <= servo_state.pan_max;
    if (servo_id == SERVO_TILT)
        return angle >= servo_state.tilt_min && angle <= servo_state.tilt_max;
    return false;
}

void servo_get_hw_range(int16_t *min_deg, int16_t *max_deg)
{
    if (min_deg)
        *min_deg = SERVO_ANGLE_HW_MIN;
    if (max_deg)
        *max_deg = SERVO_ANGLE_HW_MAX;
}

esp_err_t servo_get_limits(servo_id_t servo_id, int16_t *min_deg, int16_t *max_deg)
{
    if (!min_deg || !max_deg)
        return ESP_ERR_INVALID_ARG;
    if (servo_id == SERVO_PAN) {
        *min_deg = servo_state.pan_min;
        *max_deg = servo_state.pan_max;
    } else if (servo_id == SERVO_TILT) {
        *min_deg = servo_state.tilt_min;
        *max_deg = servo_state.tilt_max;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t servo_set_limits(servo_id_t servo_id, int16_t min_deg, int16_t max_deg)
{
    if (servo_id != SERVO_PAN && servo_id != SERVO_TILT)
        return ESP_ERR_INVALID_ARG;

    const int16_t center = (servo_id == SERVO_PAN) ? SERVO_PAN_CENTER : SERVO_TILT_CENTER;
    /* Centre must stay reachable: init and servo_center() drive to it. */
    if (min_deg < SERVO_ANGLE_HW_MIN || max_deg > SERVO_ANGLE_HW_MAX || min_deg >= max_deg ||
        min_deg > center || max_deg < center)
        return ESP_ERR_INVALID_ARG;

    int16_t *current;
    bool enabled;
    if (servo_id == SERVO_PAN) {
        servo_state.pan_min = min_deg;
        servo_state.pan_max = max_deg;
        current = &servo_state.pan_angle;
        enabled = servo_state.pan_enabled;
    } else {
        servo_state.tilt_min = min_deg;
        servo_state.tilt_max = max_deg;
        current = &servo_state.tilt_angle;
        enabled = servo_state.tilt_enabled;
    }

    if (!servo_state.initialized)
        return ESP_OK;

    /* A narrowed range must not leave the servo parked outside it. */
    int16_t inside = *current;
    if (inside < min_deg)
        inside = min_deg;
    if (inside > max_deg)
        inside = max_deg;
    if (inside == *current)
        return ESP_OK;
    if (!enabled) {
        *current = inside;
        return ESP_OK;
    }
    return servo_set_angle(servo_id, inside);
}

esp_err_t servo_set_pwm_frequency(uint16_t hz)
{
    const bool pan_live = servo_state.initialized && servo_state.pan_enabled;
    const bool tilt_live = servo_state.initialized && servo_state.tilt_enabled;
    esp_err_t ret;

    /* Release first, so no output carries a count from the old period while the
     * prescaler changes. A failed release leaves the frequency alone: changing
     * it would re-time a count that is still being output. */
    if (pan_live) {
        ret = i2c_bus_pca9685_set(SERVO_PAN_CHANNEL, 0);
        if (ret != ESP_OK)
            return ret;
    }
    if (tilt_live) {
        ret = i2c_bus_pca9685_set(SERVO_TILT_CHANNEL, 0);
        if (ret != ESP_OK)
            return ret;
    }

    const esp_err_t freq_ret = i2c_bus_pca9685_set_frequency(hz);

    /* Re-send even if the change failed: the outputs were just released, and
     * the pulse maths reads back whichever frequency is now in force. */
    esp_err_t resend_ret = ESP_OK;
    if (pan_live) {
        ret = i2c_bus_pca9685_set(SERVO_PAN_CHANNEL,
                                  angle_to_count(SERVO_PAN, servo_state.pan_angle));
        if (ret != ESP_OK)
            resend_ret = ret;
    }
    if (tilt_live) {
        ret = i2c_bus_pca9685_set(SERVO_TILT_CHANNEL,
                                  angle_to_count(SERVO_TILT, servo_state.tilt_angle));
        if (ret != ESP_OK && resend_ret == ESP_OK)
            resend_ret = ret;
    }

    return (freq_ret != ESP_OK) ? freq_ret : resend_ret;
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
 * Exists because "the servos are not moving" has several causes that look
 * identical from across the room — no VCC on the PCA9685 (its logic), no V+
 * (servo power), a failed init, a pulse train outside the servo's frame rate,
 * and a dead servo — and the console could not tell them apart. Every step logs
 * the angle, the PCA9685 count written, and the bus result, so a servo that
 * does not move while the writes succeed is a different diagnosis from one
 * whose writes are failing.
 *
 * Travels between the live limits, so it can never drive past an end stop
 * someone has measured. Blocking, by several seconds. Called on
 * peripheral_task, never from the console task. */
esp_err_t servo_exercise(void)
{
    if (!servo_state.initialized) {
        ESP_LOGE(TAG, "exercise: servos not initialised — nothing was written");
        return ESP_ERR_INVALID_STATE;
    }

    const struct {
        servo_id_t id;
        int16_t angle;
        const char *what;
    } k_steps[] = {
        {SERVO_PAN, SERVO_PAN_CENTER, "centre"},
        {SERVO_PAN, servo_state.pan_min, "look left"},
        {SERVO_PAN, servo_state.pan_max, "look right"},
        {SERVO_PAN, SERVO_PAN_CENTER, "centre"},
        {SERVO_TILT, SERVO_TILT_CENTER, "centre"},
        {SERVO_TILT, servo_state.tilt_max, "nod up"},
        {SERVO_TILT, servo_state.tilt_min, "nod down"},
        {SERVO_TILT, servo_state.tilt_max, "nod up"},
        {SERVO_TILT, servo_state.tilt_min, "nod down"},
        {SERVO_TILT, SERVO_TILT_CENTER, "centre"},
    };

    ESP_LOGI(TAG, "exercise: PCA9685 at %u Hz, limits pan %+d..%+d tilt %+d..%+d deg",
             (unsigned)i2c_bus_pca9685_frequency(), servo_state.pan_min, servo_state.pan_max,
             servo_state.tilt_min, servo_state.tilt_max);

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
                 "the PCA9685's registers — check VCC (3.3 V logic), V+ (servo power), the servo "
                 "leads, and whether these servos track pulses at %u Hz (`servo freq 50`).",
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
