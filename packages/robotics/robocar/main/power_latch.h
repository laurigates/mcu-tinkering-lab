/**
 * @file power_latch.h
 * @brief The carrier's soft-latch HOLD output (ADR-038, #1271).
 *
 * The button that switches the robot on has to stay down until HOLD is high,
 * so the line is driven as early as software can: a constructor runs before
 * app_main, and power_latch_init() asserts it again from app_main's first
 * lines with the GPIO driver. HOLD low switches the robot off once the button
 * is released (docs/reference/blocks/soft-latch.toml).
 *
 * Both do nothing unless FEATURE_POWER_LATCH_ENABLED is set. Linking:
 * app_main must call power_latch_init(), which is also what keeps this object
 * (and its constructor) in the image.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Claim POWER_HOLD_PIN as an output and drive it high.
 *
 * Idempotent, and no glitch when the constructor already did it. The first
 * line of app_main, ahead of the log mirror and everything else. Makes no
 * logging calls.
 *
 * @return ESP_OK, or ESP_ERR_NOT_SUPPORTED when the feature is off.
 */
esp_err_t power_latch_init(void);

/** Drive HOLD low: the carrier switches off once the button is up. */
void power_latch_release(void);

/** Whether the constructor drove HOLD before app_main ran. */
bool power_latch_asserted_early(void);

#ifdef __cplusplus
}
#endif
