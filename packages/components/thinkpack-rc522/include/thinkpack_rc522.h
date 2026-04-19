/**
 * @file thinkpack_rc522.h
 * @brief MFRC522 NFC reader driver (SPI) shared across ThinkPack firmware.
 *
 * Ported from packages/games/nfc-scavenger-hunt/main/rc522_driver.{c,h}.
 * The SPI bus setup and timing are unchanged from the reference driver, which
 * is known-good on hardware. Board-specific pin selection is done at compile
 * time via the BOARD_SUPERMINI / XIAO_ESP32S3 defines.
 *
 * No dependencies on project-specific tag config headers — this driver only
 * reads UIDs and leaves mapping to behaviours to the application.
 */

#ifndef THINKPACK_RC522_H
#define THINKPACK_RC522_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum UID length supported (MIFARE 7-byte cascaded UID). */
#define RC522_MAX_UID_LEN 7

/**
 * @brief Initialise the RC522 NFC reader over SPI.
 *
 * Configures the reset pin, brings up the SPI bus, performs a hardware +
 * software reset, and verifies that the RC522 reports a plausible firmware
 * version.
 *
 * @return ESP_OK on success; ESP_ERR_NOT_FOUND if the chip does not respond.
 */
esp_err_t thinkpack_rc522_init(void);

/**
 * @brief Poll once for an NFC tag.
 *
 * Non-blocking: returns false immediately if no tag is present.
 *
 * @param uid      Output buffer, at least @ref RC522_MAX_UID_LEN bytes.
 * @param uid_len  Output: actual UID length (4 or 7 bytes).
 * @return true if a tag was detected and its UID was read.
 */
bool thinkpack_rc522_poll_tag(uint8_t *uid, uint8_t *uid_len);

#ifdef __cplusplus
}
#endif

#endif /* THINKPACK_RC522_H */
