/**
 * @file cliff_core.c
 * @brief Cliff reflex decision logic. See cliff_core.h.
 */

#include "cliff_core.h"

#include <string.h>

void cliff_reset(cliff_t *c)
{
    memset(c, 0, sizeof(*c));
    c->cliff[CLIFF_FRONT] = true;
    c->cliff[CLIFF_REAR] = true;
    c->phase = CLIFF_PHASE_IDLE;
}

/** Debounce one sensor. @return true on a floor-to-cliff transition. */
static bool debounce(cliff_t *c, cliff_id_t id, bool floor_seen)
{
    const bool no_floor = !floor_seen;
    if (no_floor == c->cliff[id]) {
        c->run[id] = 0;
        return false;
    }
    c->run[id]++;
    const uint8_t need = c->cliff[id] ? CLIFF_CLEAR_SAMPLES : CLIFF_TRIP_SAMPLES;
    if (c->run[id] < need) {
        return false;
    }
    c->run[id] = 0;
    c->cliff[id] = no_floor;
    return no_floor;
}

static cliff_id_t other(cliff_id_t id)
{
    return (id == CLIFF_FRONT) ? CLIFF_REAR : CLIFF_FRONT;
}

/** Whether a back-off away from `trigger` can start or continue. */
static bool backoff_allowed(const cliff_t *c, cliff_id_t trigger, bool obstacle_ahead)
{
    if (c->cliff[other(trigger)]) {
        return false; /* no floor on the side we would move toward */
    }
    if (trigger == CLIFF_REAR && obstacle_ahead) {
        return false; /* pulling away forward would drive into the obstacle */
    }
    return true;
}

static void enter(cliff_t *c, cliff_phase_t phase, uint32_t now_ms)
{
    c->phase = phase;
    c->phase_start_ms = now_ms;
}

cliff_action_t cliff_step(cliff_t *c, const cliff_sample_t *s, uint32_t now_ms, bool obstacle_ahead)
{
    const bool front_edge = debounce(c, CLIFF_FRONT, s->valid && s->floor[CLIFF_FRONT]);
    const bool rear_edge = debounce(c, CLIFF_REAR, s->valid && s->floor[CLIFF_REAR]);

    if (front_edge || rear_edge) {
        /* Front first when both arrive together: that is the side the robot
         * usually drives toward, and with both gone no back-off starts anyway. */
        c->trigger = front_edge ? CLIFF_FRONT : CLIFF_REAR;
        c->trips++;
        enter(c, CLIFF_PHASE_BRAKE, now_ms);
    }

    const uint32_t elapsed = now_ms - c->phase_start_ms;
    switch (c->phase) {
        case CLIFF_PHASE_BRAKE:
            if (elapsed < CLIFF_BRAKE_MS) {
                return CLIFF_ACT_BRAKE;
            }
            if (!backoff_allowed(c, c->trigger, obstacle_ahead)) {
                enter(c, CLIFF_PHASE_IDLE, now_ms);
                return CLIFF_ACT_NONE;
            }
            enter(c, CLIFF_PHASE_BACKOFF, now_ms);
            return (c->trigger == CLIFF_FRONT) ? CLIFF_ACT_REVERSE : CLIFF_ACT_FORWARD;

        case CLIFF_PHASE_BACKOFF:
            if (elapsed >= CLIFF_BACKOFF_MS || !backoff_allowed(c, c->trigger, obstacle_ahead)) {
                enter(c, CLIFF_PHASE_IDLE, now_ms);
                return CLIFF_ACT_NONE;
            }
            return (c->trigger == CLIFF_FRONT) ? CLIFF_ACT_REVERSE : CLIFF_ACT_FORWARD;

        case CLIFF_PHASE_IDLE:
        default:
            return CLIFF_ACT_NONE;
    }
}

bool cliff_permits(const cliff_t *c, cliff_motion_t m)
{
    switch (m) {
        case CLIFF_MOTION_FORWARD:
            return !c->cliff[CLIFF_FRONT];
        case CLIFF_MOTION_BACKWARD:
            return !c->cliff[CLIFF_REAR];
        case CLIFF_MOTION_ROTATE:
            return !c->cliff[CLIFF_FRONT] && !c->cliff[CLIFF_REAR];
        case CLIFF_MOTION_NONE:
        default:
            return true;
    }
}

const char *cliff_name(cliff_id_t id)
{
    return (id == CLIFF_FRONT) ? "front" : "rear";
}
