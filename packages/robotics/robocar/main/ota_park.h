/**
 * @file ota_park.h
 * @brief Find open floor before an OTA install: pure C, host-tested.
 *
 * Before the robot downloads an update and reboots it parks: it turns until
 * the front rangefinder sees open floor, moves forward a little if something
 * is close behind, and stops. A reboot mid-drive is the hazard: the PCA9685
 * has its own supply and keeps its last PWM outputs through an ESP32 reset,
 * so the motors could keep running until init rewrites them.
 *
 * There is no heading sensor, so the turn is a series of timed steps:
 *
 *   SETTLE  motors stopped for settle_ms, so the smoothed front reading
 *           (ULTRASONIC_SMOOTH_N readings, about 500 ms) describes this
 *           heading, then read it:
 *             front sensor failed      -> done, NO_SENSOR (stay put)
 *             front >= open_cm         -> rear check
 *             max_steps turns taken    -> done, BEST_EFFORT (stay put)
 *             otherwise                -> ROTATE
 *   ROTATE  turn in place for rotate_ms, then SETTLE again
 *   rear check: rear sensor working and rear < rear_min_cm -> NUDGE forward
 *           for nudge_ms (the executor's obstacle reflex still applies), else
 *           done, OPEN
 *   NUDGE   then done, OPEN
 *
 * The state machine only decides; the caller turns each command into a
 * reactive_controller_manual() lease and applies the executor hold once it is
 * done. Times are uint32 milliseconds compared by difference, so the day-49
 * wrap does not stall or skip a phase.
 */

#ifndef OTA_PARK_H
#define OTA_PARK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTA_PARK_CMD_STOP = 0,
    OTA_PARK_CMD_ROTATE,
    OTA_PARK_CMD_FORWARD,
} ota_park_cmd_t;

typedef enum {
    OTA_PARK_RESULT_NONE = 0,    /**< still parking */
    OTA_PARK_RESULT_OPEN,        /**< facing open floor */
    OTA_PARK_RESULT_BEST_EFFORT, /**< no open heading within max_steps; stopped */
    OTA_PARK_RESULT_NO_SENSOR,   /**< front rangefinder failed; stopped where it was */
} ota_park_result_t;

typedef struct {
    uint16_t open_cm;     /**< front clearance that counts as open floor */
    uint16_t rear_min_cm; /**< rear clearance below which it nudges forward */
    uint32_t settle_ms;   /**< stopped time before a heading is read */
    uint32_t rotate_ms;   /**< one turning step */
    uint32_t nudge_ms;    /**< forward nudge away from something behind */
    uint8_t max_steps;    /**< turning steps before giving up */
} ota_park_config_t;

/** Defaults: 80 cm open, 30 cm behind, 700 ms settle, 250 ms steps, 400 ms nudge, 16 steps. */
#define OTA_PARK_CONFIG_DEFAULT \
    {.open_cm = 80,             \
     .rear_min_cm = 30,         \
     .settle_ms = 700,          \
     .rotate_ms = 250,          \
     .nudge_ms = 400,           \
     .max_steps = 16}

typedef enum {
    OTA_PARK_PHASE_SETTLE = 0,
    OTA_PARK_PHASE_ROTATE,
    OTA_PARK_PHASE_NUDGE,
    OTA_PARK_PHASE_DONE,
} ota_park_phase_t;

typedef struct {
    ota_park_config_t cfg;
    ota_park_phase_t phase;
    ota_park_result_t result;
    uint32_t phase_start_ms;
    uint8_t steps;
} ota_park_t;

/** Begin parking at @p now_ms; the first phase is SETTLE. */
void ota_park_start(ota_park_t *p, const ota_park_config_t *cfg, uint32_t now_ms);

/**
 * @brief Advance the state machine and return the command to apply now.
 *
 * @param front_cm  Smoothed front distance (reactive_telemetry_t.distance_cm).
 * @param front_ok  False when the front sensor has failed (sensor_failed).
 * @param rear_cm   Latest rear distance.
 * @param rear_ok   False when the rear sensor has failed or has no reading.
 * @return STOP once done.
 */
ota_park_cmd_t ota_park_step(ota_park_t *p, uint32_t now_ms, uint16_t front_cm, bool front_ok,
                             uint16_t rear_cm, bool rear_ok);

/** NONE while parking, otherwise how it ended. */
ota_park_result_t ota_park_result(const ota_park_t *p);

/** Short name for a result, for the log ("open", "best_effort", "no_sensor", "parking"). */
const char *ota_park_result_name(ota_park_result_t r);

#ifdef __cplusplus
}
#endif

#endif /* OTA_PARK_H */
