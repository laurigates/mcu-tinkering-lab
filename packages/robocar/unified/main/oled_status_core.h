/**
 * @file oled_status_core.h
 * @brief The OLED status screen's four text rows, as a pure function.
 *
 * No ESP-IDF headers: oled_status.c gathers the facts on the device, and the
 * host test (test/test_oled_status_core.c) drives this formatter with states a
 * bench cannot stage, such as a planner fuse that has tripped or a rear
 * rangefinder failing while the front one works.
 *
 * The panel's font carries A-Z, 0-9, space and `- _ . / :` only (oled.c), so
 * every row is built from those characters.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_STATUS_ROWS 4
#define OLED_STATUS_COLS 21

/** Everything the screen shows, read once per refresh. */
typedef struct {
    bool wifi_up;  /**< STA has an IP */
    uint8_t ip[4]; /**< IPv4 address, most significant octet first */

    bool i2c_bus_ok; /**< TCA9548A + PCA9685 answered at boot */
    bool motors_ok;
    bool servos_ok;
    bool leds_ok;

    bool plan_gate_on;          /**< dormancy gate enabled (`plan on`) */
    bool plan_dormant;          /**< gate is holding the planner asleep */
    bool plan_fused;            /**< spend fuse tripped (`plan resume` clears it) */
    uint32_t plan_requests;     /**< requests made since boot or the last resume */
    uint32_t plan_max_requests; /**< request ceiling; 0 = no ceiling */

    bool sonar_known;   /**< the executor's telemetry was readable */
    bool reflex_active; /**< obstacle reflex holding the wheels */
    bool front_failed;  /**< front rangefinder stopped answering */
    bool rear_failed;   /**< rear rangefinder stopped answering */
    uint16_t front_cm;  /**< smoothed front distance */
    uint16_t rear_cm;   /**< latest rear distance */
} oled_status_t;

/**
 * @brief Render @p s into four NUL-terminated rows of at most
 *        OLED_STATUS_COLS characters each.
 *
 * Row 0: network (`IP 192.168.0.42` or `WIFI DOWN`).
 * Row 1: I2C peripherals (`HW OK`, `HW NO I2C BUS`, or `HW -MOT -SRV -LED`).
 * Row 2: planner (`PLAN SLEEP 12/500`, `AWAKE`, `NOGATE`, or `FUSED`).
 * Row 3: rangefinders (`SONAR F 123 R 045`, `STOP` while the reflex holds,
 *        `ERR` for a failed sensor, `SONAR --` before telemetry exists).
 */
void oled_status_format(const oled_status_t *s, char rows[OLED_STATUS_ROWS][OLED_STATUS_COLS + 1]);

/** The head view: two rows drawn at double size (oled_text_2x()), 10 cells
 *  of 12 px across the 128 px panel. */
#define OLED_HEAD_ROWS 2
#define OLED_HEAD_COLS 10

/**
 * @brief Render the commanded head angles as two rows for the double-size view.
 *
 * Row 0 `PAN  -45`, row 1 `TILT +12`: the sign is always shown, so 0 reads as
 * `+0`, and the numbers are right-aligned so they do not jump sideways while
 * the head sweeps. Angles past three digits are clamped to -999..999.
 */
void oled_head_format(int16_t pan_deg, int16_t tilt_deg,
                      char rows[OLED_HEAD_ROWS][OLED_HEAD_COLS + 1]);

#ifdef __cplusplus
}
#endif
