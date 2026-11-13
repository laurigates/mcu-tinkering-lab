/**
 * @file twin_exec.h
 * @brief The twin's side of the firmware's executor (ADR-036 §1).
 *
 * reactive_controller.c (host path), motor_controller.c, ultrasonic_core.c,
 * head_aim.c, idle_gaze.c, cliff_core.c and cliff_sensor.c are compiled
 * unmodified. This module owns what sits under them on the robot: the two
 * rangefinders' bus, the PCA9685 the motor driver writes, the STBY pin and the
 * floor sensors' sample. twin_main.c calls twin_exec_tick() every
 * REACTIVE_LOOP_PERIOD_MS of virtual time and writes the trace records.
 *
 * Sensor values enter in two places only, twin_exec_set_range() and
 * twin_exec_set_floor(). Scripted world events call them, or with
 * "plant":"external" each plant_response does (ADR-036 §3).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ultrasonic.h"

/** twin_exec_set_range(): the sensor's next reads fail. */
#define TWIN_RANGE_FAILED (-1)

/** One wheel, decoded from the PCA9685 channels motor_controller.c wrote. */
typedef struct {
    const char *drive; /**< "forward", "reverse", "coast" or "brake". */
    uint16_t duty;     /**< The PWM count written, 0..4095.           */
} twin_wheel_t;

/** What one executor tick changed, for the `wheels` and `head` trace records. */
typedef struct {
    bool wheels_changed;
    twin_wheel_t left;
    twin_wheel_t right;
    bool head_changed; /**< Always false unless twin_exec_init() got head = true. */
    int pan_deg;
    int tilt_deg;
    const char *mode; /**< reactive_head_mode_name() of telemetry.head_mode. */
    bool settled;     /**< telemetry.head_settled.                           */
} twin_exec_report_t;

/**
 * @brief Bring the executor up as main.c does, before the first tick.
 *
 * @param seed   Scenario seed, for the idle gaze's random source.
 * @param head   Servos present: the executor aims the head.
 * @param cliff  Arm the cliff reflex. It starts as "no floor" and holds the
 *               wheels until twin_exec_set_floor() supplies floor readings.
 * @return NULL on success, otherwise the name of the call that failed.
 */
const char *twin_exec_init(uint32_t seed, bool head, bool cliff);

/**
 * @brief What is in front of a rangefinder from now on.
 *
 * Read at the sensor's next trigger, not at once: a reading takes the
 * firmware's read cycle. @p cm outside ULTRASONIC_MIN_RANGE_CM ..
 * ULTRASONIC_MAX_RANGE_CM, TWIN_RANGE_FAILED included, makes the reads fail.
 */
void twin_exec_set_range(ultrasonic_id_t sensor, int cm);

/**
 * @brief What the floor sensors read from the next tick on.
 *
 * @param valid  false: the read fails, which counts as no floor under both.
 */
void twin_exec_set_floor(bool valid, bool front, bool rear);

/**
 * @brief Run one executor tick.
 *
 * @param voice_busy  A voice turn is running (voice_turn_is_busy() on the robot).
 */
void twin_exec_tick(bool voice_busy, twin_exec_report_t *out);

/** Ticks run so far; the next tick's index. */
uint64_t twin_exec_ticks(void);

/** The wheels as the PCA9685 holds them now. */
void twin_exec_wheels(twin_wheel_t *left, twin_wheel_t *right);
