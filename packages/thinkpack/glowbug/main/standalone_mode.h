/**
 * @file standalone_mode.h
 * @brief Sensor-driven animation selector for glowbug operating without peers.
 *
 * Reads the LDR and IMU every tick and selects the appropriate animation
 * mode.  A button-press cycle overrides the auto selection.
 *
 * See https://github.com/laurigates/mcu-tinkering-lab/issues/195
 */

#ifndef STANDALONE_MODE_H
#define STANDALONE_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise standalone mode state.
 *
 * Must be called after imu_init(), light_sensor_init(), and animations_init().
 */
void standalone_mode_init(void);

/**
 * @brief Advance the standalone mode state machine by one tick.
 *
 * Reads sensors, applies the auto/override logic, and calls
 * animations_set_mode() as needed.  Call every 33 ms from the animation task.
 *
 * @param now_ms  Current monotonic time in milliseconds.
 */
void standalone_mode_tick(uint32_t now_ms);

/**
 * @brief Cycle the forced-mode override.
 *
 * Called from the button callback.  Sequences through:
 *   BREATHE → RAINBOW_CHASE → SPARKLE_BURST → AUTO → BREATHE …
 */
void standalone_mode_cycle_override(void);

/**
 * @brief Soft factory reset: clear override + persistent state.
 *
 * Invoked on a 2 s long-press.  Clears any override, erases the
 * "thinkpack" NVS namespace (best-effort — missing namespace is not
 * an error), and flashes the LED ring red for ~300 ms as visual
 * feedback.  Returns after the flash completes.
 */
void standalone_mode_factory_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* STANDALONE_MODE_H */
