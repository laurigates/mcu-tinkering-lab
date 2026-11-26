/**
 * @file motion_bound_core.c
 * @brief Executor-side time bound for drive and rotate goals. See the header.
 */

#include "motion_bound_core.h"

#include <string.h>

static uint32_t cap(uint64_t ms)
{
    return ms > MOTION_BOUND_CAP_MS ? MOTION_BOUND_CAP_MS : (uint32_t)ms;
}

uint32_t motion_bound_drive_ms(uint16_t distance_cm, uint8_t speed_pct)
{
    if (distance_cm == 0 || speed_pct == 0) {
        return 0;
    }
    /* ms = distance / (full speed * pct/100), in integers rounded up. */
    const uint64_t num = (uint64_t)distance_cm * 1000u * 100u;
    const uint64_t den = (uint64_t)MOTION_DRIVE_CM_PER_S_FULL * speed_pct;
    return cap((num + den - 1u) / den);
}

uint32_t motion_bound_rotate_ms(int16_t angle_deg)
{
    const uint32_t mag = (uint32_t)(angle_deg < 0 ? -(int32_t)angle_deg : angle_deg);
    if (mag == 0) {
        return 0;
    }
    return cap(((uint64_t)mag * 1000u + MOTION_ROTATE_DEG_PER_S - 1u) / MOTION_ROTATE_DEG_PER_S);
}

void motion_bound_reset(motion_bound_t *b)
{
    memset(b, 0, sizeof(*b));
}

bool motion_bound_expired(motion_bound_t *b, uint32_t goal_seq, uint32_t bound_ms, uint32_t now_ms)
{
    if (!b->started || goal_seq != b->seq) {
        b->started = true;
        b->seq = goal_seq;
        b->start_ms = now_ms;
    }
    if (bound_ms == MOTION_BOUND_NONE) {
        return false;
    }
    return (uint32_t)(now_ms - b->start_ms) >= bound_ms;
}
