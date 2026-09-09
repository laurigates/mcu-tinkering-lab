/**
 * @file motor_controller.c
 * @brief TB6612FNG motor control via PCA9685 I2C PWM driver
 *
 * Motor direction pins (IN1/IN2) are driven as digital outputs on the PCA9685
 * using full-on (4096) or full-off (0) values. Motor speed (PWM) uses the
 * PCA9685 12-bit resolution for finer control than the original 8-bit LEDC.
 *
 * TB6612FNG STBY pin is connected directly to a GPIO on the XIAO.
 */

#include "motor_controller.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c_bus.h"
#include "pin_config.h"

static const char *TAG = "motor_controller";

/** Number of PCA9685 channels written per motor update: IN1, IN2, PWM x2. */
#define MOTOR_CHANNEL_COUNT 6

/** Position of a motor channel within the single consecutive block written by
 * i2c_bus_pca9685_set_multi(). Values are placed by channel rather than by
 * argument order because the roles are NOT in a tidy per-motor sequence: the
 * channel numbering follows the TB6612FNG's control header top to bottom
 * (PWMA, AIN2, AIN1, BIN1, BIN2, PWMB) so the jumpers do not cross. A
 * positional array would still compile and still write six consecutive
 * channels after any renumber in pin_config.h — it would just drive the wrong
 * pins, which on a motor driver means a wheel that spins when it was told to
 * stop. */
#define MOTOR_CH_SLOT(ch) ((ch) - MOTOR_FIRST_CHANNEL)

/* One expression proving all three properties the multi-write depends on: every
 * channel lies inside the block, no two share a slot, and the block has no
 * holes. Renumber in pin_config.h and this fails at compile time. */
_Static_assert(((1u << MOTOR_CH_SLOT(MOTOR_RIGHT_PWM_CHANNEL)) |
                (1u << MOTOR_CH_SLOT(MOTOR_RIGHT_IN2_CHANNEL)) |
                (1u << MOTOR_CH_SLOT(MOTOR_RIGHT_IN1_CHANNEL)) |
                (1u << MOTOR_CH_SLOT(MOTOR_LEFT_IN1_CHANNEL)) |
                (1u << MOTOR_CH_SLOT(MOTOR_LEFT_IN2_CHANNEL)) |
                (1u << MOTOR_CH_SLOT(MOTOR_LEFT_PWM_CHANNEL))) ==
                   ((1u << MOTOR_CHANNEL_COUNT) - 1u),
               "the six motor channels must be a gap-free, non-overlapping block of "
               "MOTOR_CHANNEL_COUNT starting at MOTOR_FIRST_CHANNEL");

static struct {
    uint8_t left_speed;
    uint8_t right_speed;
    uint8_t left_direction;
    uint8_t right_direction;
    bool initialized;
} motor_state = {0};

/** Last state successfully written to the PCA9685. `valid` is false whenever
 *  the chip's contents are unknown — before the first write, and after any
 *  failed one, so a transaction that did not land is never remembered as
 *  applied. */
static struct {
    uint16_t values[MOTOR_CHANNEL_COUNT];
    uint32_t written_ms;
    bool valid;
} s_last_write = {0};

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Map 8-bit speed (0-255) to 12-bit PCA9685 value (0-4095)
static uint16_t speed_to_pwm(uint8_t speed)
{
    return (uint16_t)((uint32_t)speed * PCA9685_PWM_MAX / 255);
}

/* Set both motors in a single I2C transaction (6 channels: IN1, IN2, PWM x2).
 *
 * An unchanged state is not re-written. reactive_controller's 30 Hz loop calls
 * motor_stop() unconditionally on every iteration it is not driving, so a
 * parked robot re-wrote six identical registers 30 times a second for as long
 * as it was powered — the firmware's only continuous I2C traffic, and it
 * existed solely to restate what the chip already held. See
 * MOTOR_REFRESH_INTERVAL_MS for why the suppression expires rather than
 * lasting until the value changes.
 *
 * Suppression is fail-safe in the direction that matters: a PCA9685 that reset
 * under a stale cache holds its power-on defaults, which are all outputs off —
 * a stop. So the worst a suppressed write can do is keep the robot stopped for
 * up to MOTOR_REFRESH_INTERVAL_MS, never keep it moving. */
static esp_err_t set_motors(uint16_t r_in1, uint16_t r_in2, uint16_t r_pwm, uint16_t l_in1,
                            uint16_t l_in2, uint16_t l_pwm)
{
    const uint16_t values[MOTOR_CHANNEL_COUNT] = {
        [MOTOR_CH_SLOT(MOTOR_RIGHT_IN1_CHANNEL)] = r_in1,
        [MOTOR_CH_SLOT(MOTOR_RIGHT_IN2_CHANNEL)] = r_in2,
        [MOTOR_CH_SLOT(MOTOR_RIGHT_PWM_CHANNEL)] = r_pwm,
        [MOTOR_CH_SLOT(MOTOR_LEFT_IN1_CHANNEL)] = l_in1,
        [MOTOR_CH_SLOT(MOTOR_LEFT_IN2_CHANNEL)] = l_in2,
        [MOTOR_CH_SLOT(MOTOR_LEFT_PWM_CHANNEL)] = l_pwm,
    };
    const uint32_t now = now_ms();

    /* Unsigned subtraction, so the uint32 millisecond wrap at day 49 yields a
     * correct elapsed time rather than a refresh that never falls due again. */
    if (s_last_write.valid && memcmp(values, s_last_write.values, sizeof(values)) == 0 &&
        (now - s_last_write.written_ms) < MOTOR_REFRESH_INTERVAL_MS) {
        return ESP_OK;
    }

    const esp_err_t ret =
        i2c_bus_pca9685_set_multi(MOTOR_FIRST_CHANNEL, MOTOR_CHANNEL_COUNT, values);
    if (ret == ESP_OK) {
        memcpy(s_last_write.values, values, sizeof(values));
        s_last_write.written_ms = now;
        s_last_write.valid = true;
    } else {
        s_last_write.valid = false;
    }
    return ret;
}

esp_err_t motor_controller_init(void)
{
    if (motor_state.initialized) {
        ESP_LOGW(TAG, "Motor controller already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing motor controller (PCA9685-based)");

    // Configure STBY GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_STBY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Enable motor driver
    gpio_set_level(MOTOR_STBY_PIN, 1);

    /* Reaches the bus: s_last_write starts invalid, so this first stop is never
     * suppressed however the board got here. */
    motor_stop();

    motor_state.initialized = true;
    ESP_LOGI(TAG, "Motor controller initialized (12-bit PCA9685 PWM)");
    return ESP_OK;
}

esp_err_t motor_move_forward(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t pwm = speed_to_pwm(speed);
    esp_err_t ret =
        set_motors(PCA9685_FULL_ON, PCA9685_FULL_OFF, pwm, PCA9685_FULL_ON, PCA9685_FULL_OFF, pwm);

    if (ret == ESP_OK) {
        motor_state.left_speed = speed;
        motor_state.right_speed = speed;
        motor_state.left_direction = 1;
        motor_state.right_direction = 1;
        ESP_LOGD(TAG, "Forward speed=%d (pwm=%d)", speed, pwm);
    }
    return ret;
}

esp_err_t motor_move_backward(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t pwm = speed_to_pwm(speed);
    esp_err_t ret =
        set_motors(PCA9685_FULL_OFF, PCA9685_FULL_ON, pwm, PCA9685_FULL_OFF, PCA9685_FULL_ON, pwm);

    if (ret == ESP_OK) {
        motor_state.left_speed = speed;
        motor_state.right_speed = speed;
        motor_state.left_direction = 0;
        motor_state.right_direction = 0;
        ESP_LOGD(TAG, "Backward speed=%d", speed);
    }
    return ret;
}

esp_err_t motor_turn_left(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t full_pwm = speed_to_pwm(speed);
    uint16_t half_pwm = speed_to_pwm(speed / 2);
    esp_err_t ret = set_motors(PCA9685_FULL_ON, PCA9685_FULL_OFF, full_pwm, PCA9685_FULL_ON,
                               PCA9685_FULL_OFF, half_pwm);

    if (ret == ESP_OK) {
        motor_state.right_speed = speed;
        motor_state.left_speed = speed / 2;
        motor_state.left_direction = 1;
        motor_state.right_direction = 1;
        ESP_LOGD(TAG, "Turn left speed=%d", speed);
    }
    return ret;
}

esp_err_t motor_turn_right(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t full_pwm = speed_to_pwm(speed);
    uint16_t half_pwm = speed_to_pwm(speed / 2);
    esp_err_t ret = set_motors(PCA9685_FULL_ON, PCA9685_FULL_OFF, half_pwm, PCA9685_FULL_ON,
                               PCA9685_FULL_OFF, full_pwm);

    if (ret == ESP_OK) {
        motor_state.right_speed = speed / 2;
        motor_state.left_speed = speed;
        motor_state.left_direction = 1;
        motor_state.right_direction = 1;
        ESP_LOGD(TAG, "Turn right speed=%d", speed);
    }
    return ret;
}

esp_err_t motor_rotate_cw(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t pwm = speed_to_pwm(speed);
    // Right backward, left forward
    esp_err_t ret =
        set_motors(PCA9685_FULL_OFF, PCA9685_FULL_ON, pwm, PCA9685_FULL_ON, PCA9685_FULL_OFF, pwm);

    if (ret == ESP_OK) {
        motor_state.left_speed = speed;
        motor_state.right_speed = speed;
        motor_state.left_direction = 1;
        motor_state.right_direction = 0;
        ESP_LOGD(TAG, "Rotate CW speed=%d", speed);
    }
    return ret;
}

esp_err_t motor_rotate_ccw(uint8_t speed)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t pwm = speed_to_pwm(speed);
    // Right forward, left backward
    esp_err_t ret =
        set_motors(PCA9685_FULL_ON, PCA9685_FULL_OFF, pwm, PCA9685_FULL_OFF, PCA9685_FULL_ON, pwm);

    if (ret == ESP_OK) {
        motor_state.left_speed = speed;
        motor_state.right_speed = speed;
        motor_state.left_direction = 0;
        motor_state.right_direction = 1;
        ESP_LOGD(TAG, "Rotate CCW speed=%d", speed);
    }
    return ret;
}

bool motor_is_initialized(void)
{
    return motor_state.initialized;
}

esp_err_t motor_stop(void)
{
    // Brake mode: all direction pins low, PWM = 0
    esp_err_t ret =
        set_motors(PCA9685_FULL_OFF, PCA9685_FULL_OFF, 0, PCA9685_FULL_OFF, PCA9685_FULL_OFF, 0);

    if (ret == ESP_OK) {
        motor_state.left_speed = 0;
        motor_state.right_speed = 0;
        motor_state.left_direction = 0;
        motor_state.right_direction = 0;
        ESP_LOGD(TAG, "Motors stopped");
    }
    return ret;
}

esp_err_t motor_set_individual(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction,
                               uint8_t right_direction)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    uint16_t r_in1 = right_direction ? PCA9685_FULL_ON : PCA9685_FULL_OFF;
    uint16_t r_in2 = right_direction ? PCA9685_FULL_OFF : PCA9685_FULL_ON;
    uint16_t l_in1 = left_direction ? PCA9685_FULL_ON : PCA9685_FULL_OFF;
    uint16_t l_in2 = left_direction ? PCA9685_FULL_OFF : PCA9685_FULL_ON;

    esp_err_t ret =
        set_motors(r_in1, r_in2, speed_to_pwm(right_speed), l_in1, l_in2, speed_to_pwm(left_speed));

    if (ret == ESP_OK) {
        motor_state.left_speed = left_speed;
        motor_state.right_speed = right_speed;
        motor_state.left_direction = left_direction;
        motor_state.right_direction = right_direction;
    }
    return ret;
}

esp_err_t motor_get_state(uint8_t *left_speed, uint8_t *right_speed, uint8_t *left_direction,
                          uint8_t *right_direction)
{
    if (!motor_state.initialized)
        return ESP_ERR_INVALID_STATE;

    if (left_speed)
        *left_speed = motor_state.left_speed;
    if (right_speed)
        *right_speed = motor_state.right_speed;
    if (left_direction)
        *left_direction = motor_state.left_direction;
    if (right_direction)
        *right_direction = motor_state.right_direction;

    return ESP_OK;
}
