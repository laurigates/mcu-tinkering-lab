#include "status_color.h"

#include <math.h>

// Same values as balance_state_t (ipc.h), kept local so this file needs no
// SDK-adjacent header.
enum { ST_IDLE = 0, ST_RUN = 1, ST_FAULT = 2 };

static rgb_t scaled(uint8_t r, uint8_t g, uint8_t b, uint32_t level, uint32_t full)
{
    // level/full of STATUS_RGB_MAX, per channel given as 0..255 weights.
    if (level > full) {
        level = full;
    }
    const uint32_t k = STATUS_RGB_MAX * level;
    return (rgb_t){(uint8_t)(r * k / (255u * full)), (uint8_t)(g * k / (255u * full)),
                   (uint8_t)(b * k / (255u * full))};
}

rgb_t status_color(const status_inputs_t *in)
{
    if (in->starting) {
        const uint32_t phase = (in->ms / 20u) % 100u;  // 2 s triangle
        const uint32_t level = phase < 50u ? 10u + phase : 110u - phase;
        return scaled(0, 255, 255, level, 60u);
    }
    if (in->state == ST_FAULT) {
        return (in->ms / 100u) % 2u == 0u ? scaled(255, 0, 0, 1, 1) : (rgb_t){0, 0, 0};
    }
    if (!in->imu_ok) {
        return scaled(255, 0, 0, 1, 1);
    }
    if (in->state == ST_RUN) {
        return scaled(0, 255, 0, 1, 1);
    }
    if (in->arm_holdoff_ticks != 0u) {
        return scaled(180, 0, 255, 1, 1);
    }
    const float a = fabsf(in->pitch_deg);
    if (a < in->arm_window_deg) {
        // A quarter brightness on entering the window, full when it arms.
        const uint32_t need = in->gesture_needed ? in->gesture_needed : 1u;
        const uint32_t filled = in->gesture_ticks < need ? in->gesture_ticks : need;
        return scaled(0, 255, 0, need / 4u + (3u * filled) / 4u, need);
    }
    if (a < STATUS_NEAR_DEG) {
        return scaled(255, 160, 0, 1, 1);
    }
    return scaled(0, 0, 255, 1, 4);  // dim
}
