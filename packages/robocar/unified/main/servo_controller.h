/**
 * @file servo_controller.h
 * @brief Hardware abstraction layer for servo motor control (pan/tilt camera mount)
 *
 * Same public API as robocar-main, plus live travel limits and a frequency
 * change that preserves pulse widths. Internal implementation uses the PCA9685
 * via the TCA9548A I2C bus.
 *
 * Angles are real servo degrees about centre (pulse mapping in pin_config.h),
 * and every pulse is clamped to the SG90 datasheet range.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum { SERVO_PAN = 0, SERVO_TILT = 1 } servo_id_t;

typedef struct {
    int16_t pan_angle;
    int16_t tilt_angle;
} servo_position_t;

#define SERVO_PAN_CENTER 0
#define SERVO_TILT_CENTER 0

/* Boot defaults for the travel limits, in real servo degrees about centre.
 *
 * Deliberately narrow. The pan/tilt bracket's end stops have not been measured,
 * and a servo commanded past one stalls against it — loud, hot, and hard on an
 * SG90's plastic gears. Widen them on the bench with
 * `servo limit pan|tilt <min> <max>` (not persisted), then pin the measured
 * values here. */
#define SERVO_PAN_LIMIT_MIN_DEFAULT -60
#define SERVO_PAN_LIMIT_MAX_DEFAULT 60
#define SERVO_TILT_LIMIT_MIN_DEFAULT -30
#define SERVO_TILT_LIMIT_MAX_DEFAULT 30

esp_err_t servo_controller_init(void);
esp_err_t servo_set_angle(servo_id_t servo_id, int16_t angle);
esp_err_t servo_set_pan(int16_t angle);
esp_err_t servo_set_tilt(int16_t angle);
esp_err_t servo_set_position(int16_t pan_angle, int16_t tilt_angle);
esp_err_t servo_center(servo_id_t servo_id);
esp_err_t servo_center_all(void);
esp_err_t servo_get_angle(servo_id_t servo_id, int16_t *angle);
esp_err_t servo_get_position(servo_position_t *position);
esp_err_t servo_disable(servo_id_t servo_id);
esp_err_t servo_disable_all(void);
esp_err_t servo_enable(servo_id_t servo_id);
esp_err_t servo_enable_all(void);
bool servo_is_initialized(void);

/** True when initialised and the output has not been released by servo_disable(). */
bool servo_is_enabled(servo_id_t servo_id);

/** True when the angle is inside the servo's live travel limits. */
bool servo_is_angle_valid(servo_id_t servo_id, int16_t angle);

/**
 * @brief Set a servo's travel limits, in real degrees about centre. Not persisted.
 *
 * Requires hw_min <= min <= centre <= max <= hw_max and min < max, with the
 * hardware range from servo_get_hw_range(). A servo currently outside the new
 * range is moved to the nearest limit.
 */
esp_err_t servo_set_limits(servo_id_t servo_id, int16_t min_deg, int16_t max_deg);
esp_err_t servo_get_limits(servo_id_t servo_id, int16_t *min_deg, int16_t *max_deg);

/** The widest limits accepted: the angles at which the pulse reaches the SG90 range. */
void servo_get_hw_range(int16_t *min_deg, int16_t *max_deg);

/**
 * @brief Change the PCA9685 PWM frequency without changing any servo's pulse width.
 *
 * A PCA9685 count is a fraction of the period, so a count written at one
 * frequency is a different pulse at any other: the 50 Hz centre count replayed
 * at 200 Hz is a ~0.37 ms pulse that drives the servo into its end stop and holds
 * it there. This releases the enabled servo outputs, changes the frequency, then
 * re-sends each enabled servo's angle at the new period. If releasing an output
 * fails the frequency is left unchanged.
 *
 * Chip-wide: the motors and LEDs move to the new frequency too.
 */
esp_err_t servo_set_pwm_frequency(uint16_t hz);

esp_err_t servo_move_smooth(servo_id_t servo_id, int16_t target_angle, uint8_t step_size,
                            uint32_t delay_ms);
esp_err_t servo_sweep(servo_id_t servo_id, int16_t start_angle, int16_t end_angle,
                      uint8_t step_size, uint32_t delay_ms, uint32_t cycles);
esp_err_t servo_stop_motion(void);

/** How long each step of servo_exercise() is held, in ms. Long enough for an
 *  SG90 to traverse its limits and for a human to see it happen. */
#define SERVO_EXERCISE_STEP_MS 600u

/**
 * @brief The PCA9685 count a given angle maps to at the current PWM frequency.
 *
 * Exposed for diagnostics: printing the count next to the bus result is what
 * separates "the write never happened" from "the write happened and the servo
 * ignored it". Clamped to the SG90 pulse range, not to the travel limits.
 */
uint16_t servo_angle_to_count(servo_id_t id, int16_t angle);

/**
 * @brief Bench gesture — shake the head, then nod — logging every write.
 *
 * Travels between the live limits, never past them. Blocking, several seconds.
 * Run it on peripheral_task, not the console task.
 */
esp_err_t servo_exercise(void);

#endif  // SERVO_CONTROLLER_H
