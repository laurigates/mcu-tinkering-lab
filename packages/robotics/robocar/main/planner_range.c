/**
 * @file planner_range.c
 * @brief The planner prompt's rangefinder clause (see planner_range.h).
 */
#include "planner_range.h"

#include <stdio.h>

#include "ultrasonic_core.h"

/** One sensor's reading as the model should read it. */
static void describe(uint16_t cm, bool failed, char *out, size_t out_size)
{
    if (failed || cm == ULTRASONIC_DIST_ERROR) {
        snprintf(out, out_size, "unavailable (the sensor is not answering)");
    } else if (cm >= ULTRASONIC_MAX_RANGE_CM) {
        snprintf(out, out_size, "no echo within %u cm", (unsigned)ULTRASONIC_MAX_RANGE_CM);
    } else {
        snprintf(out, out_size, "%u cm", (unsigned)cm);
    }
}

size_t planner_range_clause(const planner_range_t *r, char *out, size_t out_size)
{
    if (!out || out_size == 0) {
        return 0;
    }
    out[0] = '\0';
    if (!r) {
        return 0;
    }

    char ahead[48];
    char behind[48];
    describe(r->front_cm, r->front_failed, ahead, sizeof(ahead));
    describe(r->rear_cm, r->rear_failed, behind, sizeof(behind));

    const int n = snprintf(
        out, out_size,
        "Ultrasonic rangefinders measured, as this request was sent: ahead %s, behind %s.%s "
        "Judge distance from these measurements, not from how large things look in the "
        "image; the camera sits low, so nearby objects fill the frame. ",
        ahead, behind,
        r->reflex_active ? " The obstacle reflex is holding forward motion; it does not "
                           "hold reversing or turning in place."
                         : "");
    if (n < 0 || (size_t)n >= out_size) {
        out[0] = '\0';
        return 0;
    }
    return (size_t)n;
}
