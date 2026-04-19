/**
 * @file thinkpack_power.h
 * @brief Battery voltage monitoring, adaptive beacon rates, and low-battery UX.
 *
 * Three concerns are bundled into this component:
 *
 *  1. Classification of the Vbat ADC reading (pure logic, host-testable).
 *     Thresholds are chosen for a typical single-cell LiPo (4.2 V fully
 *     charged, 3.3 V knee). A 2-tap safety margin is applied so the box
 *     enters CRITICAL before the BMS cutoff actually fires.
 *
 *        OK       >= 3600 mV
 *        LOW      3300-3599 mV
 *        CRITICAL <  3300 mV
 *
 *  2. Mapping from battery state to the mesh beacon interval (pure logic,
 *     host-testable). Rationale: when battery is low, broadcasting less
 *     frequently saves radio energy at the cost of a slower peer-discovery
 *     time.
 *
 *        OK       ->  500 ms (2 Hz — matches beacon default)
 *        LOW      -> 2000 ms (0.5 Hz)
 *        CRITICAL -> 5000 ms (0.2 Hz)
 *
 *  3. Low-battery LED feedback: a 1 Hz red pulse in LOW, solid red in
 *     CRITICAL. Pure logic, host-testable.
 *
 * The @ref thinkpack_power_init routine starts the ADC oneshot unit and
 * schedules a periodic tick via esp_timer. The orchestration glue (actually
 * poking thinkpack-mesh to change beacon rate, and driving LEDs) lives in
 * power_mgr.c. The pure-logic routines can be unit-tested on the host.
 */

#ifndef THINKPACK_POWER_H
#define THINKPACK_POWER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_err.h"
#else
typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#endif
#ifndef ESP_ERR_INVALID_ARG
#define ESP_ERR_INVALID_ARG (-1)
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Thresholds (documented — keep in sync with classify.c)              */
/* ------------------------------------------------------------------ */

/** Minimum voltage (mV) to still be considered OK. */
#define THINKPACK_POWER_OK_MV 3600u

/** Minimum voltage (mV) to still be considered LOW (below this is CRITICAL). */
#define THINKPACK_POWER_LOW_MV 3300u

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    THINKPACK_POWER_OK = 0,
    THINKPACK_POWER_LOW = 1,
    THINKPACK_POWER_CRITICAL = 2,
} thinkpack_power_battery_state_t;

typedef struct {
    int adc_gpio;               /**< GPIO number to read Vbat from (ADC1 channel). */
    uint32_t tick_interval_ms;  /**< Periodic classifier tick (default 5000). */
    uint16_t divider_ratio_x10; /**< Vbat / Vadc * 10 (e.g. 20 for a 1:2 divider). 0 => 20. */
} power_config_t;

/* ------------------------------------------------------------------ */
/* Pure logic (host-testable)                                          */
/* ------------------------------------------------------------------ */

/**
 * Classify a millivolt reading into a battery state.
 *
 * Thresholds (see THINKPACK_POWER_OK_MV, THINKPACK_POWER_LOW_MV):
 *   mv >= 3600      -> OK
 *   3300 <= mv <3600 -> LOW
 *   mv <  3300      -> CRITICAL
 */
thinkpack_power_battery_state_t thinkpack_power_classify(uint16_t mv);

/**
 * Map a battery state to the adaptive ESP-NOW beacon interval.
 *
 *   OK       ->  500 ms
 *   LOW      -> 2000 ms
 *   CRITICAL -> 5000 ms
 *
 * Unknown states clamp to 5000 ms to fail-safe toward lower radio activity.
 */
uint16_t thinkpack_power_adaptive_beacon_interval_ms(thinkpack_power_battery_state_t state);

/**
 * Compute the low-battery LED color at a given monotonic millisecond tick.
 *
 * OK       -> (0,0,0)          LED off, no battery alert.
 * LOW      -> 1 Hz red pulse   sinusoidal-ish red; off at t=0, full at t=500,
 *                              off again at t=1000.
 * CRITICAL -> (255,0,0)        solid red at full brightness.
 *
 * Outputs are written to @p r, @p g, @p b. NULL arguments are ignored.
 */
void thinkpack_power_low_battery_led_tick(thinkpack_power_battery_state_t state, uint32_t tick_ms,
                                          uint8_t *r, uint8_t *g, uint8_t *b);

/* ------------------------------------------------------------------ */
/* Platform glue (ESP-IDF only)                                        */
/* ------------------------------------------------------------------ */

/**
 * Initialise the power monitor. Starts ADC oneshot and a periodic tick that
 * re-classifies the battery and broadcasts state changes.
 *
 * Calling this on a non-ESP platform is a no-op that returns ESP_OK so host
 * tests of firmware main() files still link.
 */
esp_err_t thinkpack_power_init(const power_config_t *config);

/**
 * Return the most recent Vbat reading in millivolts.
 *
 * On host builds this always returns 0.
 */
uint16_t thinkpack_power_read_voltage_mv(void);

/**
 * Enter deep sleep for the requested number of milliseconds.
 *
 * On host builds this is a no-op. Wraps esp_sleep_enable_timer_wakeup +
 * esp_deep_sleep_start on the device.
 */
void thinkpack_power_deep_sleep(uint32_t wake_ms);

#ifdef __cplusplus
}
#endif

#endif /* THINKPACK_POWER_H */
