/**
 * @file hot_cold.h
 * @brief RSSI-driven "hot-cold" indicator logic for Finderbox group mode.
 *
 * Maintains an exponential moving average of RSSI samples (α = 1/4, pure
 * integer math — no floating point or ESP-IDF dependencies) and converts
 * the smoothed value into an RGB gradient running from cold blue
 * (weak signal, distant target) to hot red (strong signal, target close).
 *
 * All logic lives in the `thinkpack-behaviors` component so the math is
 * host-testable; Finderbox's group_mode.c is the only caller under ESP-IDF.
 *
 * Typical usage:
 * @code
 *   hot_cold_state_t state;
 *   hot_cold_reset(&state);
 *   hot_cold_update(&state, -60);
 *   uint8_t r, g, b;
 *   hot_cold_get_color(&state, &r, &g, &b);
 *   led_ring_fill(r, g, b);
 *   led_ring_show();
 * @endcode
 */

#ifndef HOT_COLD_H
#define HOT_COLD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Lower bound (weakest signal, coldest) of the mapped RSSI range, dBm. */
#define HOT_COLD_RSSI_MIN (-90)

/** Upper bound (strongest signal, hottest) of the mapped RSSI range, dBm. */
#define HOT_COLD_RSSI_MAX (-30)

/**
 * @brief Running hot-cold filter state.
 *
 * Store one instance per target.  Use ::hot_cold_reset before the first
 * ::hot_cold_update call so the filter warms up from the first sample
 * instead of jumping toward zero.
 */
typedef struct {
    int16_t smoothed_q8; /**< Smoothed RSSI scaled ×256 (Q8.8 fixed point) */
    bool primed;         /**< true once at least one sample has been taken */
} hot_cold_state_t;

/**
 * @brief Reset @p state so the next sample seeds the filter directly.
 *
 * Safe to call on a NULL pointer.
 */
void hot_cold_reset(hot_cold_state_t *state);

/**
 * @brief Fold @p rssi_dbm into the running EMA.
 *
 * On the first (primed=false) call the value is adopted verbatim so the
 * indicator doesn't need multiple samples to leave the -128 dBm default.
 * Subsequent calls apply an α = 1/4 EMA: new = old + (sample - old) / 4.
 *
 * @p state must not be NULL.
 */
void hot_cold_update(hot_cold_state_t *state, int8_t rssi_dbm);

/**
 * @brief Current smoothed RSSI in dBm (integer), or INT8_MIN if never primed.
 */
int8_t hot_cold_smoothed_dbm(const hot_cold_state_t *state);

/**
 * @brief Map the current smoothed RSSI to an RGB gradient.
 *
 * The gradient is a straight linear blend from blue (r=0, g=0, b=255) at
 * HOT_COLD_RSSI_MIN to red (r=255, g=0, b=0) at HOT_COLD_RSSI_MAX.  The
 * green channel stays at 0 to keep the "cool→warm" semantic obvious.
 * Output is safe to drop into led_ring_fill() directly — the Finderbox
 * LED ring applies its own brightness cap.
 *
 * If @p state has never been primed the function outputs pure blue.
 * Any of @p r/@p g/@p b may be NULL to skip that channel.
 */
void hot_cold_get_color(const hot_cold_state_t *state, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif

#endif /* HOT_COLD_H */
