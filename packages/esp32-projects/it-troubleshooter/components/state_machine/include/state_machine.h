/**
 * @file state_machine.h
 * @brief NVS-persisted device state machine for IT Troubleshooter.
 *
 * Stores operational phase and workflow step in NVS namespace "it-ts" so
 * multi-step workflows survive power cycles. State is loaded on init and
 * written synchronously on every update (NVS flash commit overhead is
 * acceptable given the infrequent update rate).
 *
 * Phase values (sm_phase_t):
 *   SM_PHASE_USB_ONLY      — device booted, USB composite active, no WiFi
 *   SM_PHASE_WIFI_CONNECTED — WiFi hotspot link is up
 *   SM_PHASE_CMD_RUNNING   — command passthrough task is active
 *
 * Step is a free-running counter (0–255) that callers increment as they
 * complete logical steps within a workflow. Reset to 0 on phase change or
 * explicit sm_reset().
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/** Operational phases stored in NVS. */
typedef enum {
    SM_PHASE_USB_ONLY = 1,       /**< USB mounted, no WiFi. */
    SM_PHASE_WIFI_CONNECTED = 2, /**< WiFi hotspot connected. */
    SM_PHASE_CMD_RUNNING = 3,    /**< Command passthrough active (Phase 2). */
    SM_PHASE_AI_RUNNING = 4,     /**< Claude API diagnostic loop active (Phase 3). */
} sm_phase_t;

/**
 * @brief Initialize the state machine and load persisted state from NVS.
 *
 * Must be called after nvs_flash_init(). If no state is found in NVS the
 * machine starts at SM_PHASE_USB_ONLY, step 0.
 *
 * @return ESP_OK on success, or an NVS error code.
 */
esp_err_t sm_init(void);

/**
 * @brief Return the current phase.
 */
sm_phase_t sm_get_phase(void);

/**
 * @brief Return the current step within the active phase.
 */
uint8_t sm_get_step(void);

/**
 * @brief Advance to a new phase, resetting step to 0, and persist.
 *
 * No-op if phase == current phase.
 *
 * @param phase  Target phase.
 * @return ESP_OK, or an NVS error code.
 */
esp_err_t sm_set_phase(sm_phase_t phase);

/**
 * @brief Update the step counter and persist.
 *
 * @param step  New step value (0–255).
 * @return ESP_OK, or an NVS error code.
 */
esp_err_t sm_set_step(uint8_t step);

/**
 * @brief Reset state to SM_PHASE_USB_ONLY, step 0, and persist.
 *
 * @return ESP_OK, or an NVS error code.
 */
esp_err_t sm_reset(void);
