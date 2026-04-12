/**
 * @file ota_github_events.h
 * @brief esp_event bus declarations for the ota_github component.
 *
 * Observers register with the default esp_event loop:
 *
 * @code
 *   esp_event_handler_register(OTA_GITHUB_EVENTS, ESP_EVENT_ANY_ID,
 *                              &my_handler, NULL);
 * @endcode
 *
 * Every posted event carries a pointer to an @ref ota_github_event_payload_t
 * with details appropriate to the event id.
 */

#ifndef OTA_GITHUB_EVENTS_H
#define OTA_GITHUB_EVENTS_H

#include <stdint.h>
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief esp_event base used by the component. */
ESP_EVENT_DECLARE_BASE(OTA_GITHUB_EVENTS);

/** @brief Event ids posted on the @ref OTA_GITHUB_EVENTS base. */
typedef enum {
    /** A newer release has been detected. `.version` is set. */
    OTA_GITHUB_EVENT_UPDATE_AVAILABLE = 0,
    /** No update needed — the current firmware is up to date. */
    OTA_GITHUB_EVENT_NO_UPDATE = 1,
    /** Download/flash in progress. `.progress` is 0..100. */
    OTA_GITHUB_EVENT_PROGRESS = 2,
    /** Download + flash + verification succeeded. Reboot imminent. */
    OTA_GITHUB_EVENT_SUCCESS = 3,
    /** Update failed. `.error_code` matches @ref ota_github_get_error_code. */
    OTA_GITHUB_EVENT_FAILED = 4,
    /** Rollback was cancelled after the stability window elapsed. */
    OTA_GITHUB_EVENT_VALID_CONFIRMED = 5,
} ota_github_event_id_t;

/** @brief Payload posted with each event. Fields unset for a given event
 *         are zero. */
typedef struct {
    uint8_t progress;        /**< 0..100, valid for PROGRESS events. */
    uint8_t error_code;      /**< Non-zero for FAILED events. */
    char version[32];        /**< New version, for UPDATE_AVAILABLE events. */
} ota_github_event_payload_t;

#ifdef __cplusplus
}
#endif

#endif  // OTA_GITHUB_EVENTS_H
