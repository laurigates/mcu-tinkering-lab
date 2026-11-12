/**
 * @file cliff_core.h
 * @brief Cliff reflex decision logic, with no ESP-IDF dependency.
 *
 * Two downward-facing floor sensors, front and rear, each reporting "floor
 * seen" or not every executor tick. This module debounces them, runs the
 * reflex (brake, then back away from the edge) and says which motions the
 * wheels may make while a sensor sees no floor. It knows nothing about the
 * sensor: the interim build uses two salvaged TCRT5000-style reflective
 * sensors on MCP23017 inputs (cliff_sensor.c); the planned VL53L4CD ToF pair
 * (issue #1010) would feed the same cliff_sample_t.
 *
 * Fail closed. A failed read counts as "no floor", and the state starts as
 * "no floor" on both sensors, so nothing moves until the floor is confirmed.
 * Starting there also means the boot is not an edge: the reflex fires only on
 * a floor-to-cliff transition, never on the first reading.
 *
 * Shared-core pattern (.claude/rules/testing.md): the firmware and
 * test/test_cliff_core.c compile this file unmodified.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Consecutive no-floor samples before a sensor is declared over a cliff. Two
 *  ticks (~66 ms) so a single glitch on a 100 kΩ pull-up line, next to the
 *  motor wiring, cannot brake the robot. */
#define CLIFF_TRIP_SAMPLES 2U

/** Consecutive floor samples before a cliff clears. Longer than the trip, so
 *  a sensor right at the edge does not chatter between the two. */
#define CLIFF_CLEAR_SAMPLES 3U

/** How long the reflex brakes before backing off. An estimate: the robot's
 *  stopping time at the executor's duties is unmeasured (issue #1010). */
#define CLIFF_BRAKE_MS 150U

/** How long the back-off drives away from the edge. An estimate; measure the
 *  distance it covers on the bench before trusting it near a real edge. */
#define CLIFF_BACKOFF_MS 400U

/** Back-off wheel speed, 0-255 as motor_move_*() takes it (~55 %). */
#define CLIFF_BACKOFF_SPEED 140U

typedef enum {
    CLIFF_FRONT = 0,
    CLIFF_REAR,
    CLIFF_COUNT,
} cliff_id_t;

/** One poll of both sensors. `valid` false means the read failed, which
 *  counts as no floor under both (fail closed). */
typedef struct {
    bool valid;
    bool floor[CLIFF_COUNT];
} cliff_sample_t;

typedef enum {
    CLIFF_PHASE_IDLE = 0, /**< No reflex running; permits() still applies. */
    CLIFF_PHASE_BRAKE,    /**< Braking after an edge, for CLIFF_BRAKE_MS.  */
    CLIFF_PHASE_BACKOFF,  /**< Driving away from the edge, CLIFF_BACKOFF_MS. */
} cliff_phase_t;

/** What the reflex wants from the wheels this tick. */
typedef enum {
    CLIFF_ACT_NONE = 0, /**< Nothing; the next layer drives (filtered by permits()). */
    CLIFF_ACT_BRAKE,    /**< Brake.                                      */
    CLIFF_ACT_REVERSE,  /**< Back away from a front edge.                */
    CLIFF_ACT_FORWARD,  /**< Pull away from a rear edge.                 */
} cliff_action_t;

/** The direction a wheel command moves the robot, for permits(). */
typedef enum {
    CLIFF_MOTION_NONE = 0, /**< Stopped. Always permitted.                  */
    CLIFF_MOTION_FORWARD,  /**< Straight or arcing forward.                 */
    CLIFF_MOTION_BACKWARD, /**< Straight back.                              */
    CLIFF_MOTION_ROTATE,   /**< In place: both ends of the chassis sweep.   */
} cliff_motion_t;

typedef struct {
    bool cliff[CLIFF_COUNT];  /**< Debounced: true = no floor under that sensor. */
    uint8_t run[CLIFF_COUNT]; /**< Consecutive samples disagreeing with cliff[]. */
    cliff_phase_t phase;
    cliff_id_t trigger; /**< The sensor whose edge started the phase.   */
    uint32_t phase_start_ms;
    uint32_t trips; /**< Edges that started a reflex, since reset.  */
} cliff_t;

/**
 * @brief Start over: both sensors read as cliffs, no reflex running.
 */
void cliff_reset(cliff_t *c);

/**
 * @brief One executor tick.
 *
 * @param c                The state.
 * @param s                This tick's poll.
 * @param now_ms           Milliseconds; wraps at day 49 without harm.
 * @param obstacle_ahead   The obstacle reflex is holding the wheels, so a
 *                         forward pull-away from a rear edge is skipped.
 * @return What the wheels should do. CLIFF_ACT_NONE hands them to the next
 *         layer, which must still pass cliff_permits().
 */
cliff_action_t cliff_step(cliff_t *c, const cliff_sample_t *s, uint32_t now_ms,
                          bool obstacle_ahead);

/**
 * @brief Whether a wheel motion is allowed with the sensors as they are now.
 *
 * Forward needs floor in front, backward needs floor behind, rotating in place
 * needs both (a turn swings both ends). Stopping is always allowed.
 */
bool cliff_permits(const cliff_t *c, cliff_motion_t m);

/** Short name for logs: "front" / "rear". */
const char *cliff_name(cliff_id_t id);

#ifdef __cplusplus
}
#endif
