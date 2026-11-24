/**
 * @file power_core.h
 * @brief Pack-voltage cutoff and power-button decisions, with no ESP-IDF
 *        dependency (ADR-038, #1271).
 *
 * The socketed carrier switches the pack with a P-MOSFET behind a push-button
 * soft latch (docs/reference/blocks/soft-latch.toml) and puts the pack on an
 * ADC1 pin through a divider. This module holds the decisions; power_manager.c
 * owns the ADC, the button read, the HOLD pin and the shutdown sequence.
 *
 * Pack cutoff. The firmware is the first line and the pack's BMS the backup,
 * so the threshold sits above the BMS's cell cutoff. A motor start sags the
 * pack for a moment (#991: the chip browned out within milliseconds of a goal
 * that starts the wheels), so a sample below the threshold does nothing until
 * the *filtered* voltage has stayed below it for a minimum time, and it
 * clears only above a higher voltage, so a pack hovering at the threshold
 * does not restart the timer on every tick.
 *
 * A reading under `absent_mv` is not a pack: the carrier on USB power alone,
 * or a robot with no carrier fitted, reads near zero or noise there. It never
 * trips the cutoff, because shutting down a robot that has no pack to protect
 * would only stop a bench session.
 *
 * Power button. SENSE reads low while the button is down. The button that
 * switched the robot on is still down when the firmware starts, so a press
 * counts only after the first confirmed release, otherwise every boot would
 * end in a shutdown two seconds later.
 *
 * Shared-core pattern (.claude/rules/testing.md): the firmware and
 * test/test_power_core.c compile this file unmodified. Times are uint32
 * milliseconds compared by difference, so the day-49 wrap does not matter.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------
 * Divider
 * ------------------------------------------------------------------------ */

/** The carrier's divider: R17 from the switched pack rail to the pin, R18 from
 *  the pin to GND (hardware.toml; 0.1803, the same as balancebot rev3). */
#define POWER_DIVIDER_TOP_OHM 100000U
#define POWER_DIVIDER_BOTTOM_OHM 22000U

/** Pack millivolts for a calibrated pin reading in millivolts, rounded. */
uint32_t power_pack_mv_from_pin_mv(uint32_t pin_mv);

/* ------------------------------------------------------------------------
 * Pack cutoff
 * ------------------------------------------------------------------------ */

/** Cutoff threshold: 3.3 V per cell on 2S, the example in #1271. The BMS's own
 *  cutoff is not recorded in this repo, and the value has not been settled from
 *  a discharge log: change it here once that log exists. */
#define POWER_PACK_LOW_MV 6600U

/** The cutoff clears above this: 0.2 V per cell above POWER_PACK_LOW_MV, wider
 *  than the pack's recovery when the wheels stop. */
#define POWER_PACK_CLEAR_MV 7000U

/** How long the filtered pack must stay below POWER_PACK_LOW_MV. Long against
 *  a motor-start sag (milliseconds), short against the minutes a pack takes to
 *  run down. */
#define POWER_PACK_MIN_LOW_MS 5000U

/** Below this there is no pack to protect. A 2S pack is never this low while
 *  it still runs a robot. */
#define POWER_PACK_ABSENT_MV 4000U

/** The filter takes 1/N of the distance to each new sample. At the monitor's
 *  10 Hz a 200 ms sag of 2 V moves the filtered value by under 0.8 V. */
#define POWER_PACK_FILTER_DIV 4

typedef struct {
    uint32_t low_mv;
    uint32_t clear_mv;
    uint32_t min_low_ms;
    uint32_t absent_mv;
} power_pack_config_t;

#define POWER_PACK_CONFIG_DEFAULT         \
    {.low_mv = POWER_PACK_LOW_MV,         \
     .clear_mv = POWER_PACK_CLEAR_MV,     \
     .min_low_ms = POWER_PACK_MIN_LOW_MS, \
     .absent_mv = POWER_PACK_ABSENT_MV}

typedef enum {
    POWER_PACK_NO_SAMPLE = 0, /**< nothing valid read yet */
    POWER_PACK_ABSENT,        /**< under absent_mv: no pack, never trips */
    POWER_PACK_OK,            /**< above the threshold, or recovering */
    POWER_PACK_LOW,           /**< below the threshold, timer running */
    POWER_PACK_TRIPPED,       /**< below it for min_low_ms: shut down */
} power_pack_state_t;

typedef struct {
    power_pack_config_t cfg;
    power_pack_state_t state;
    bool have_filter;
    bool below; /**< filtered value is on the low side of the band */
    uint32_t filtered_mv;
    uint32_t below_since_ms;
} power_pack_t;

/** Start with no sample. */
void power_pack_init(power_pack_t *p, const power_pack_config_t *cfg);

/**
 * @brief Feed one pack reading in millivolts and return the state.
 *
 * TRIPPED latches: it is returned on every later call, and the caller acts on
 * the first one. Between low_mv and clear_mv the previous side holds, so a pack
 * that has been low stays low until it clears.
 */
power_pack_state_t power_pack_update(power_pack_t *p, uint32_t now_ms, uint32_t pack_mv);

/** A read failed. Restarts the timer, so bad data never adds up to a trip; a
 *  tripped monitor stays tripped. */
void power_pack_read_failed(power_pack_t *p);

/** The filtered pack voltage, 0 before the first valid sample or while absent. */
uint32_t power_pack_filtered_mv(const power_pack_t *p);

/** Short name for a state, for the log ("ok", "low", "tripped", ...). */
const char *power_pack_state_name(power_pack_state_t s);

/* ------------------------------------------------------------------------
 * Power button
 * ------------------------------------------------------------------------ */

/** How long the button is held to switch off. Longer than the press that
 *  switches on, so the two are not confused. Unmeasured on hardware. */
#define POWER_BUTTON_LONG_PRESS_MS 2000U

typedef enum {
    POWER_BUTTON_NONE = 0,
    POWER_BUTTON_LONG_PRESS, /**< held for long_press_ms: returned once per press */
} power_button_event_t;

typedef enum {
    POWER_BUTTON_ARMING = 0, /**< waiting for the release after power-on */
    POWER_BUTTON_IDLE,
    POWER_BUTTON_HELD,
    POWER_BUTTON_FIRED, /**< event returned; waiting for the release */
} power_button_state_t;

typedef struct {
    uint32_t long_press_ms;
    power_button_state_t state;
    uint32_t pressed_since_ms;
} power_button_t;

/** Start armed-after-release: a button already down is the power-on press. */
void power_button_init(power_button_t *b, uint32_t long_press_ms);

/**
 * @brief Feed one sample.
 *
 * @param valid    False when the read failed (no expander, bus fault). A
 *                 failed read never completes a press, cancels a press in
 *                 progress, and does not count as the release that arms.
 * @param pressed  SENSE read low.
 */
power_button_event_t power_button_update(power_button_t *b, uint32_t now_ms, bool valid,
                                         bool pressed);

#ifdef __cplusplus
}
#endif
