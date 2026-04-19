/**
 * @file led_pulse.c
 * @brief Pure-logic low-battery LED animation.
 *
 *   OK       -> LED off.
 *   LOW      -> Triangular red pulse at 1 Hz.  t=0ms off, t=500ms full,
 *               t=1000ms off (wraps).
 *   CRITICAL -> Solid red, full brightness.
 *
 * The LOW pulse is intentionally triangular rather than sinusoidal so the
 * shape is exactly reproducible from integer math — easier to test, no
 * libm dependency.
 */

#include "thinkpack_power.h"

#define PULSE_PERIOD_MS 1000u

static void assign(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t rv, uint8_t gv, uint8_t bv)
{
    if (r)
        *r = rv;
    if (g)
        *g = gv;
    if (b)
        *b = bv;
}

void thinkpack_power_low_battery_led_tick(thinkpack_power_battery_state_t state, uint32_t tick_ms,
                                          uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (state == THINKPACK_POWER_OK) {
        assign(r, g, b, 0, 0, 0);
        return;
    }

    if (state == THINKPACK_POWER_CRITICAL) {
        assign(r, g, b, 255, 0, 0);
        return;
    }

    /* THINKPACK_POWER_LOW — 1 Hz triangular pulse */
    uint32_t phase = tick_ms % PULSE_PERIOD_MS;
    uint32_t half = PULSE_PERIOD_MS / 2u;
    uint32_t level;
    if (phase <= half) {
        /* 0 -> 255 over 0..500ms */
        level = (phase * 255u) / half;
    } else {
        /* 255 -> 0 over 500..1000ms */
        level = ((PULSE_PERIOD_MS - phase) * 255u) / half;
    }
    if (level > 255u) {
        level = 255u;
    }
    assign(r, g, b, (uint8_t)level, 0, 0);
}
