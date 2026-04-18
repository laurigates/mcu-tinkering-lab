#ifndef RC522_DRIVER_H
#define RC522_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "tag_config.h"

/**
 * Initialize the RC522 NFC reader over SPI.
 */
esp_err_t rc522_init(void);

/**
 * Poll for an NFC tag. Returns true if a tag is present and its UID was read.
 *
 * @param uid      Output buffer for the tag UID (at least MAX_UID_LEN bytes).
 * @param uid_len  Output: actual UID length in bytes.
 */
bool rc522_poll_tag(uint8_t *uid, uint8_t *uid_len);

#endif  // RC522_DRIVER_H
