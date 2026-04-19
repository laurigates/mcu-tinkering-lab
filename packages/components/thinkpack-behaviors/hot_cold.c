/**
 * @file hot_cold.c
 * @brief RSSI smoothing + blue→red colour mapping for Finderbox seek mode.
 *
 * Pure integer math; no ESP-IDF or hardware dependencies — fully host
 * testable.  See hot_cold.h for API documentation.
 */

#include "hot_cold.h"

#include <limits.h>
#include <stddef.h>

/* Exponential moving average factor: α = 1/4.
 * new_q8 = old_q8 + (sample_q8 - old_q8) >> 2. */
#define HOT_COLD_EMA_SHIFT 2

/* Q8.8 helpers — keep one bit of headroom for signed right-shift of
 * the difference (absolute value well below 2^15 for realistic RSSI). */
#define HOT_COLD_Q8(x) ((int16_t)((int32_t)(x) * 256))
#define HOT_COLD_FROM_Q8(x) ((int16_t)((x) / 256))

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

void hot_cold_reset(hot_cold_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->smoothed_q8 = 0;
    state->primed = false;
}

void hot_cold_update(hot_cold_state_t *state, int8_t rssi_dbm)
{
    if (state == NULL) {
        return;
    }

    int16_t sample_q8 = HOT_COLD_Q8(rssi_dbm);

    if (!state->primed) {
        state->smoothed_q8 = sample_q8;
        state->primed = true;
        return;
    }

    /* new = old + (sample - old) / 4  — integer EMA. */
    int32_t diff = (int32_t)sample_q8 - (int32_t)state->smoothed_q8;
    int32_t next = (int32_t)state->smoothed_q8 + (diff >> HOT_COLD_EMA_SHIFT);

    /* Clamp to int16_t range before storing; real RSSI values keep this
     * comfortably inside bounds but be defensive. */
    if (next > INT16_MAX) {
        next = INT16_MAX;
    } else if (next < INT16_MIN) {
        next = INT16_MIN;
    }
    state->smoothed_q8 = (int16_t)next;
}

int8_t hot_cold_smoothed_dbm(const hot_cold_state_t *state)
{
    if (state == NULL || !state->primed) {
        return INT8_MIN;
    }
    int16_t dbm = HOT_COLD_FROM_Q8(state->smoothed_q8);
    if (dbm > INT8_MAX) {
        return INT8_MAX;
    }
    if (dbm < INT8_MIN) {
        return INT8_MIN;
    }
    return (int8_t)dbm;
}

/* ------------------------------------------------------------------ */
/* Colour mapping                                                      */
/* ------------------------------------------------------------------ */

/* Map a smoothed dBm value to a 0..255 "heat" intensity.  Values at or
 * below HOT_COLD_RSSI_MIN produce 0 (cold / blue); values at or above
 * HOT_COLD_RSSI_MAX produce 255 (hot / red). */
static uint8_t heat_intensity(int dbm)
{
    if (dbm <= HOT_COLD_RSSI_MIN) {
        return 0;
    }
    if (dbm >= HOT_COLD_RSSI_MAX) {
        return 255;
    }
    int span = HOT_COLD_RSSI_MAX - HOT_COLD_RSSI_MIN;
    int numer = (dbm - HOT_COLD_RSSI_MIN) * 255;
    return (uint8_t)(numer / span);
}

void hot_cold_get_color(const hot_cold_state_t *state, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 255;

    if (state != NULL && state->primed) {
        int dbm = HOT_COLD_FROM_Q8(state->smoothed_q8);
        uint8_t heat = heat_intensity(dbm);
        red = heat;
        blue = (uint8_t)(255 - heat);
    }

    if (r != NULL) {
        *r = red;
    }
    if (g != NULL) {
        *g = green;
    }
    if (b != NULL) {
        *b = blue;
    }
}
