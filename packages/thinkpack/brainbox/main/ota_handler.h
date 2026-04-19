/**
 * @file ota_handler.h
 * @brief Brainbox OTA handler — downloads firmware from GitHub Releases
 *        and broadcasts it over the ESP-NOW mesh.
 *
 * Owns the `ota-github` component and the MSG_OTA_MANIFEST /
 * MSG_OTA_CHUNK / MSG_OTA_COMPLETE broadcast sequence.
 *
 * Trigger mechanism (hardware-verification helper): the handler exposes
 * @ref ota_handler_check_and_push_all so the ThinkPack Brainbox can
 * start a relay pass from application code. The `just ota-push-all`
 * recipe documents the expected serial trigger for bring-up testing.
 */

#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Sentinel meaning "broadcast to every follower box type". */
#define OTA_HANDLER_TARGET_ALL 0xFFu

/**
 * @brief One-shot init. Safe to call after mesh is up.
 *
 * Wires the underlying `ota_github` component in TRIGGERED mode so it
 * only downloads when this Brainbox asks it to. The actual relay over
 * ESP-NOW is performed in ota_handler_check_and_push_all.
 */
esp_err_t ota_handler_init(void);

/**
 * @brief Download the latest release asset and broadcast it to the mesh.
 *
 * Blocks until the broadcast sequence finishes or fails. Uses the
 * default broadcast-rate limiter documented in thinkpack_ota.h
 * (≈10 ms per chunk ⇒ ~18 kB/s).
 *
 * @param target_box_mask Bitmask over thinkpack_box_type_t values, or
 *                        OTA_HANDLER_TARGET_ALL.
 */
esp_err_t ota_handler_check_and_push_all(uint8_t target_box_mask);

#ifdef __cplusplus
}
#endif

#endif /* OTA_HANDLER_H */
