/**
 * @file power_core.c
 * @brief Pack cutoff and power-button decisions (see power_core.h).
 */

#include "power_core.h"

uint32_t power_pack_mv_from_pin_mv(uint32_t pin_mv)
{
    const uint64_t num = (uint64_t)pin_mv * (POWER_DIVIDER_TOP_OHM + POWER_DIVIDER_BOTTOM_OHM);
    return (uint32_t)((num + POWER_DIVIDER_BOTTOM_OHM / 2) / POWER_DIVIDER_BOTTOM_OHM);
}

/* ------------------------------------------------------------------------
 * Pack cutoff
 * ------------------------------------------------------------------------ */

void power_pack_init(power_pack_t *p, const power_pack_config_t *cfg)
{
    *p = (power_pack_t){.cfg = *cfg, .state = POWER_PACK_NO_SAMPLE};
}

/** Move the filter 1/POWER_PACK_FILTER_DIV of the way to @p mv; the first
 *  sample after a gap seeds it. */
static void filter_in(power_pack_t *p, uint32_t mv)
{
    if (!p->have_filter) {
        p->filtered_mv = mv;
        p->have_filter = true;
        return;
    }
    if (mv >= p->filtered_mv) {
        p->filtered_mv += (mv - p->filtered_mv) / POWER_PACK_FILTER_DIV;
    } else {
        p->filtered_mv -= (p->filtered_mv - mv) / POWER_PACK_FILTER_DIV;
    }
}

power_pack_state_t power_pack_update(power_pack_t *p, uint32_t now_ms, uint32_t pack_mv)
{
    if (p->state == POWER_PACK_TRIPPED) {
        return p->state;
    }

    if (pack_mv < p->cfg.absent_mv) {
        /* No pack: forget everything, so the next one starts from its own
         * voltage and no earlier dip counts toward it. */
        p->have_filter = false;
        p->filtered_mv = 0;
        p->below = false;
        p->state = POWER_PACK_ABSENT;
        return p->state;
    }

    filter_in(p, pack_mv);

    if (p->filtered_mv < p->cfg.low_mv) {
        if (!p->below) {
            p->below = true;
            p->below_since_ms = now_ms;
        }
    } else if (p->filtered_mv >= p->cfg.clear_mv) {
        p->below = false;
    }
    /* Between low_mv and clear_mv `below` keeps its value. */

    if (!p->below) {
        p->state = POWER_PACK_OK;
    } else if ((uint32_t)(now_ms - p->below_since_ms) >= p->cfg.min_low_ms) {
        p->state = POWER_PACK_TRIPPED;
    } else {
        p->state = POWER_PACK_LOW;
    }
    return p->state;
}

void power_pack_read_failed(power_pack_t *p)
{
    if (p->state == POWER_PACK_TRIPPED) {
        return;
    }
    p->below = false;
    if (p->state == POWER_PACK_LOW) {
        p->state = POWER_PACK_OK;
    }
}

uint32_t power_pack_filtered_mv(const power_pack_t *p)
{
    return p->have_filter ? p->filtered_mv : 0;
}

const char *power_pack_state_name(power_pack_state_t s)
{
    switch (s) {
        case POWER_PACK_NO_SAMPLE:
            return "none";
        case POWER_PACK_ABSENT:
            return "absent";
        case POWER_PACK_OK:
            return "ok";
        case POWER_PACK_LOW:
            return "low";
        case POWER_PACK_TRIPPED:
            return "tripped";
    }
    return "?";
}

/* ------------------------------------------------------------------------
 * Power button
 * ------------------------------------------------------------------------ */

void power_button_init(power_button_t *b, uint32_t long_press_ms)
{
    *b = (power_button_t){.long_press_ms = long_press_ms, .state = POWER_BUTTON_ARMING};
}

power_button_event_t power_button_update(power_button_t *b, uint32_t now_ms, bool valid,
                                         bool pressed)
{
    if (!valid) {
        /* Unknown level: cancel a press in progress, arm nothing. */
        if (b->state == POWER_BUTTON_HELD) {
            b->state = POWER_BUTTON_IDLE;
        }
        return POWER_BUTTON_NONE;
    }

    switch (b->state) {
        case POWER_BUTTON_ARMING:
        case POWER_BUTTON_FIRED:
            if (!pressed) {
                b->state = POWER_BUTTON_IDLE;
            }
            break;
        case POWER_BUTTON_IDLE:
            if (pressed) {
                b->state = POWER_BUTTON_HELD;
                b->pressed_since_ms = now_ms;
            }
            break;
        case POWER_BUTTON_HELD:
            if (!pressed) {
                b->state = POWER_BUTTON_IDLE;
            } else if ((uint32_t)(now_ms - b->pressed_since_ms) >= b->long_press_ms) {
                b->state = POWER_BUTTON_FIRED;
                return POWER_BUTTON_LONG_PRESS;
            }
            break;
    }
    return POWER_BUTTON_NONE;
}
