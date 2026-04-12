/**
 * @file ota_github.h
 * @brief Reusable GitHub-Releases OTA update component for ESP-IDF.
 *
 * `ota_github` is a project-agnostic wrapper around `esp_ghota` and
 * `esp_https_ota` that provides:
 *
 *  - Periodic polling of GitHub Releases (PULL mode)
 *  - Externally-triggered URL download (TRIGGERED mode, e.g. from I2C/UART)
 *  - Optional MQTT push-notify to short-circuit the polling interval
 *  - SHA256 verification of the downloaded app-ELF digest
 *  - Automatic rollback-cancellation after a configurable stability window
 *  - Thread-safe progress / status reporting
 *  - An esp_event bus for non-polling observers
 *
 * The component deliberately has no knowledge of the hosting project's
 * topology (WiFi bring-up, MQTT broker URI, dual-controller choreography,
 * serial fallback, etc.). Those concerns are handled by the caller via
 * the @ref ota_github_hooks_t callback table and the optional MQTT client
 * handle injection in @ref ota_github_config_t.
 *
 * ### Minimal usage (PULL mode)
 * @code
 *   ota_github_config_t cfg = OTA_GITHUB_CONFIG_DEFAULT();
 *   cfg.mode                    = OTA_GITHUB_MODE_PULL;
 *   cfg.github_org              = "laurigates";
 *   cfg.github_repo              = "mcu-tinkering-lab";
 *   cfg.firmware_filename_match = "my-project";
 *   cfg.poll_interval_min       = 360;  // 6 hours
 *   ESP_ERROR_CHECK(ota_github_init(&cfg));
 * @endcode
 *
 * ### Minimal usage (TRIGGERED mode)
 * @code
 *   ota_github_config_t cfg = OTA_GITHUB_CONFIG_DEFAULT();
 *   cfg.mode        = OTA_GITHUB_MODE_TRIGGERED;
 *   cfg.github_org  = "laurigates";
 *   cfg.github_repo = "mcu-tinkering-lab";
 *   ESP_ERROR_CHECK(ota_github_init(&cfg));
 *
 *   // Later, when an external signal arrives with a tag + 4-byte SHA256 prefix:
 *   ota_github_trigger_tag("my-project", "v1.2.3", sha_prefix);
 * @endcode
 */

#ifndef OTA_GITHUB_H
#define OTA_GITHUB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Operating mode of the OTA component.
 *
 * A single device uses exactly one mode for the lifetime of a boot. To run
 * both behaviors on the same device (e.g. periodic poll AND external
 * trigger), use PULL mode and call @ref ota_github_trigger_url when the
 * external signal fires — PULL mode supports manual triggers too.
 */
typedef enum {
    /** Periodic polling of GitHub releases via esp_ghota. */
    OTA_GITHUB_MODE_PULL = 0,
    /** Wait for explicit @ref ota_github_trigger_url / _trigger_tag calls. */
    OTA_GITHUB_MODE_TRIGGERED = 1,
} ota_github_mode_t;

/**
 * @brief High-level state reported by @ref ota_github_get_status.
 *
 * Values intentionally match the robocar I2C protocol's `ota_status_t` so
 * they can be forwarded unchanged across the bus. The numeric values are
 * part of the ABI — do not renumber.
 */
typedef enum {
    OTA_GITHUB_STATUS_IDLE = 0x00,
    OTA_GITHUB_STATUS_IN_PROGRESS = 0x01,
    OTA_GITHUB_STATUS_SUCCESS = 0x02,
    OTA_GITHUB_STATUS_FAILED = 0x03,
    OTA_GITHUB_STATUS_MAINTENANCE_MODE = 0x04,
} ota_github_status_t;

/**
 * @brief Optional callback hooks for project-specific behavior.
 *
 * All hooks are optional — set to NULL to skip. They run on the OTA worker
 * task unless documented otherwise, so keep them brief or dispatch work.
 */
typedef struct {
    /**
     * @brief Invoked once a newer release is detected, before the download
     *        begins. Use it to publish status, dim the LCD, etc.
     */
    void (*on_update_available)(const char *new_version, void *user_ctx);

    /**
     * @brief Invoked immediately before the device will reboot into the
     *        new firmware. Use it to quiesce peripherals (stop motors,
     *        detach servos, notify peers over I2C, …).
     *
     * @return ESP_OK to proceed with reboot. Any other value aborts the
     *         reboot and leaves the update pending-verify.
     */
    esp_err_t (*on_update_ready_to_reboot)(void *user_ctx);

    /**
     * @brief Invoked before any HTTP traffic is issued. Use it to lazily
     *        bring up WiFi from NVS credentials, connect to a VPN, etc.
     *
     * @return ESP_OK to proceed with download. Any other value aborts.
     */
    esp_err_t (*pre_download_hook)(void *user_ctx);

    /**
     * @brief Invoked on progress ticks. `progress` is 0..100.
     */
    void (*on_progress)(uint8_t progress, void *user_ctx);
} ota_github_hooks_t;

/**
 * @brief Full configuration passed to @ref ota_github_init.
 *
 * Prefer initializing with @ref OTA_GITHUB_CONFIG_DEFAULT() and overriding
 * the fields you care about — the defaults are tuned for the robocar
 * reference implementation (6-hour poll, 60-second rollback window,
 * 30-second HTTP timeout) and are sensible for most projects.
 */
typedef struct {
    /* --- Required --- */
    ota_github_mode_t mode;
    const char *github_org;   /**< GitHub org/user (e.g. "laurigates"). */
    const char *github_repo;  /**< GitHub repo name. */

    /* --- PULL mode --- */
    /** Substring that release asset names must contain (e.g. "robocar-camera").
     *  Matches esp_ghota's `filenamematch` semantics. Ignored in TRIGGERED mode. */
    const char *firmware_filename_match;
    /** Poll interval in minutes. Default 360 (6 hours). 0 disables polling
     *  (useful when you only want MQTT-triggered checks). */
    uint32_t poll_interval_min;

    /* --- TRIGGERED mode --- */
    /** Asset filename (without path) that will be appended to
     *  `https://github.com/{org}/{repo}/releases/download/{tag}/{filename}`
     *  when @ref ota_github_trigger_tag is used. If left NULL the caller
     *  must use @ref ota_github_trigger_url and supply a full URL. */
    const char *triggered_asset_filename;

    /* --- Common --- */
    /** Milliseconds to wait after a successful boot before calling
     *  `esp_ota_mark_app_valid_cancel_rollback`. Default 60000 (60s). */
    uint32_t stability_timeout_ms;
    /** HTTP timeout for the firmware download. Default 30000 (30s). */
    uint32_t http_timeout_ms;
    /** Stack size of the internal OTA worker task. Default 8192. */
    uint32_t task_stack_size;
    /** FreeRTOS priority of the internal OTA worker task. Default 5. */
    uint8_t task_priority;

    /* --- Optional MQTT push-notify --- */
    /** If true and `mqtt_client` is set, the component subscribes to
     *  `mqtt_notify_topic`; any inbound message triggers an immediate
     *  update check (rate-limited to at most once per 60 seconds). */
    bool mqtt_enabled;
    /** Caller-owned MQTT client. The component never creates or destroys
     *  MQTT clients — it only registers event handlers. */
    esp_mqtt_client_handle_t mqtt_client;
    /** Topic to subscribe for push notifications. e.g. "myproject/ota/notify" */
    const char *mqtt_notify_topic;
    /** Optional topic for publishing status JSON strings. May be NULL. */
    const char *mqtt_status_topic;

    /* --- Optional hooks --- */
    ota_github_hooks_t hooks;
    void *hooks_user_ctx;
} ota_github_config_t;

/**
 * @brief Sensible defaults. Equivalent to `{0}` plus the tuned timing
 *        constants used in production by the robocar reference project.
 */
#define OTA_GITHUB_CONFIG_DEFAULT()               \
    ((ota_github_config_t){                       \
        .mode = OTA_GITHUB_MODE_PULL,             \
        .poll_interval_min = 360,                 \
        .stability_timeout_ms = 60000,            \
        .http_timeout_ms = 30000,                 \
        .task_stack_size = 8192,                  \
        .task_priority = 5,                       \
    })

/**
 * @brief Initialize the OTA component with the supplied configuration.
 *
 * Must be called exactly once per boot, after NVS and (for PULL mode or
 * MQTT-enabled TRIGGERED mode) after the network stack is ready.
 *
 * @return
 *  - ESP_OK on success
 *  - ESP_ERR_INVALID_ARG if required fields (org, repo) are missing
 *  - ESP_ERR_INVALID_STATE if called twice
 *  - Propagated error from esp_ghota / esp_event_handler_register
 */
esp_err_t ota_github_init(const ota_github_config_t *cfg);

/**
 * @brief Trigger an immediate update check. PULL mode only.
 *
 * Equivalent to `ghota_check()`. In TRIGGERED mode this returns
 * ESP_ERR_NOT_SUPPORTED — use @ref ota_github_trigger_url instead.
 */
esp_err_t ota_github_check_now(void);

/**
 * @brief Start a firmware download from an explicit HTTPS URL.
 *
 * Available in both modes. The URL must be HTTPS and reachable with the
 * embedded ESP-IDF certificate bundle (any public CA works). The optional
 * `sha256_prefix` is compared against the first 4 bytes of the downloaded
 * firmware's embedded app description SHA256; mismatch aborts and marks
 * the partition invalid so the bootloader rolls back on next boot.
 *
 * @param url            Full HTTPS download URL.
 * @param sha256_prefix  First 4 bytes of expected SHA256, or NULL to skip.
 */
esp_err_t ota_github_trigger_url(const char *url, const uint8_t sha256_prefix[4]);

/**
 * @brief Convenience wrapper: build the URL from tag + `triggered_asset_filename`.
 *
 * URL format:
 *   `https://github.com/{org}/{repo}/releases/download/{tag}/{asset}`
 *
 * @param asset_override   If non-NULL, overrides `triggered_asset_filename`.
 * @param tag              Release tag (e.g. "v1.2.3" or "my-project@v1.2.3").
 * @param sha256_prefix    First 4 bytes of expected SHA256, or NULL.
 */
esp_err_t ota_github_trigger_tag(const char *asset_override, const char *tag,
                                 const uint8_t sha256_prefix[4]);

/** @brief Current progress 0..100. Returns 0 when idle. */
uint8_t ota_github_get_progress(void);

/** @brief Current status. Thread-safe. */
ota_github_status_t ota_github_get_status(void);

/**
 * @brief Error code from the most recent OTA attempt. Zero if none.
 *
 * Values:
 *   1 = internal error (task creation, etc.)
 *   2 = pre-download hook failed (WiFi unavailable, etc.)
 *   3 = esp_https_ota_begin failed
 *   4 = download / flash error
 *   5 = esp_https_ota_finish failed
 *   6 = SHA256 mismatch
 */
uint8_t ota_github_get_error_code(void);

/** @brief Current app version string, from `esp_app_get_description()`. */
const char *ota_github_get_version(void);

/* --------------------------------------------------------------------------
 * Escape hatch: expose the underlying esp_ghota client for callers that
 * need to call esp_ghota APIs directly (e.g. `ghota_get_latest_version`
 * for peer-orchestration). The handle is only valid in PULL mode.
 *
 * Declared with `void *` to avoid forcing every consumer to pull in
 * esp_ghota headers. In the caller:
 *
 *     #include "esp_ghota.h"
 *     ghota_client_handle_t *c = ota_github_pull_get_client_handle();
 *
 * Returns NULL if PULL mode is not active.
 * -------------------------------------------------------------------------- */
void *ota_github_pull_get_client_handle(void);

/**
 * @brief Manually cancel rollback and mark this firmware valid.
 *
 * Called automatically once `stability_timeout_ms` elapses without any
 * panic/reboot. Call it earlier from app code if you have stronger
 * evidence that the new firmware is healthy.
 */
esp_err_t ota_github_confirm_valid(void);

#ifdef __cplusplus
}
#endif

#endif  // OTA_GITHUB_H
