/**
 * @file planner_range.h
 * @brief The planner prompt's rangefinder clause, with no ESP-IDF dependency.
 *
 * The planner request is stateless and carries one JPEG, so without this
 * clause the model judges distance from the image alone. The camera sits low on
 * the chassis, where anything near fills the frame, and the model reported
 * obstacles as closer than they were. The clause states what the front and rear
 * RCWL-1601s measured, and says "unavailable" for a sensor that stopped
 * answering: a failed read counts as max range in the executor's filter, so
 * reporting the number would tell the model the path is clear when nothing was
 * measured (stateless-model-gating.md §4).
 */
#ifndef PLANNER_RANGE_H
#define PLANNER_RANGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** What the executor last measured; filled from reactive_telemetry_t. */
typedef struct {
    uint16_t front_cm;  /**< Front running mean (telemetry distance_cm).      */
    bool front_failed;  /**< Front sensor failed a streak of reads.           */
    bool reflex_active; /**< The obstacle reflex is holding forward motion.   */
    uint16_t rear_cm;   /**< Rear latest reading; ULTRASONIC_DIST_ERROR if none. */
    bool rear_failed;   /**< Rear sensor failed a streak of reads.            */
} planner_range_t;

/** Upper bound on the clause's length, NUL included. */
#define PLANNER_RANGE_CLAUSE_MAX 448

/**
 * @brief Write the rangefinder clause for one planner request.
 *
 * @return Characters written (excluding NUL), or 0 with out[0] = '\0' when
 *         `r` is NULL or the buffer is too small for the whole clause; a
 *         partial clause is never written.
 */
size_t planner_range_clause(const planner_range_t *r, char *out, size_t out_size);

#endif /* PLANNER_RANGE_H */
