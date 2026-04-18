/**
 * @file ota_github_internal.h
 * @brief Private state shared between the component's translation units.
 *
 * Not part of the public API. Do not include from outside the component.
 */

#ifndef OTA_GITHUB_INTERNAL_H
#define OTA_GITHUB_INTERNAL_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "mqtt_client.h"
#include "ota_github.h"
#include "ota_github_events.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialized;
    ota_github_config_t cfg;

    /* State (guarded by mutex) */
    SemaphoreHandle_t mutex;
    ota_github_status_t status;
    uint8_t progress;
    uint8_t error_code;
    bool firmware_valid_confirmed;

    /* Rollback stability timer */
    TimerHandle_t stability_timer;
} ota_github_state_t;

/** @brief Single global component instance. */
extern ota_github_state_t g_ota_github;

/* ---- Common helpers (ota_github.c) ---- */
void ota_github_set_state(ota_github_status_t status, uint8_t progress, uint8_t error_code);
void ota_github_set_progress(uint8_t progress);
void ota_github_post_event(ota_github_event_id_t id, const ota_github_event_payload_t *payload);

/* ---- Mode-specific initializers ---- */
esp_err_t ota_github_pull_init(void);
esp_err_t ota_github_pull_check_now(void);

/* ---- Direct download driver (shared by TRIGGERED mode and PULL's
 *      manual-URL path). Runs the esp_https_ota state machine with
 *      progress/error bookkeeping + SHA256 verification. Blocks caller. */
esp_err_t ota_github_direct_run(const char *url, const uint8_t sha256_prefix[4]);

/* ---- Optional MQTT push-notify integration ---- */
esp_err_t ota_github_mqtt_init(void);

#ifdef __cplusplus
}
#endif

#endif  // OTA_GITHUB_INTERNAL_H
