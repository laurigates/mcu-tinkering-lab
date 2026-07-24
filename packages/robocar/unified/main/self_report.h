/**
 * @file self_report.h
 * @brief Spoken self-introduction and status self-diagnostic.
 *
 * The robot has a voice (MAX98357A + Gemini TTS, ADR-019) but until now only
 * spoke when the vision planner emitted a `speak`. This module makes it
 * announce itself the moment it becomes *able to speak*, and re-announce when a
 * subsystem's health changes — so a silent, part-populated board can tell you
 * what came up and what didn't.
 *
 * Design (see the plan / ADR-019):
 *   - Facts are gathered deterministically on-device (never hallucinated).
 *   - A lightweight Gemini *text* call phrases them into one spoken line
 *     (gemini_backend_narrate); on failure an on-device template speaks instead.
 *   - The line is posted to the existing speech_queue — the same path the
 *     planner's `speak` uses, so the two never collide (drop-newest queue).
 *   - Cadence: once when first voice-able, then on health change (debounced +
 *     rate-limited). The facts are always logged (and MQTT-published when up)
 *     so the diagnostic is observable even while the robot cannot talk.
 *
 * The monitor is independent of the camera/planner, so it can report
 * "camera not responding" even when the planner cannot run.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "credentials_loader.h"  // MAX_SSID_LENGTH
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Subsystems whose boot result is recorded by main.c's init phases. */
typedef enum {
    SELF_REPORT_SUBSYS_CAMERA = 0,
    SELF_REPORT_SUBSYS_I2C_BUS,  //!< mux + PCA9685: motors, servos, LEDs
    SELF_REPORT_SUBSYS_AUDIO,
} self_report_subsystem_t;

/**
 * @brief A snapshot of the robot's subsystem health.
 *
 * Booleans plus a little context. Each is rendered into the facts string as
 * OK / not-responding (expected) / absent (optional) so a genuine fault reads
 * differently from optional hardware that simply isn't fitted.
 */
typedef struct {
    bool wifi_up;                //!< STA connected
    char ssid[MAX_SSID_LENGTH];  //!< SSID when known, else ""
    bool camera_ok;              //!< OV2640 init succeeded (boot-recorded)
    bool i2c_bus_ok;             //!< TCA9548A + PCA9685 up (motors/servos/LEDs)
    bool mcp23017_present;       //!< optional GPIO expander detected
    bool audio_ok;               //!< I2S player task + ring ready
    bool key_present;            //!< Gemini API key available
    char version[32];            //!< firmware version
} robocar_status_t;

/** Longest facts string produced by self_report_format_facts(), incl. NUL. */
#define SELF_REPORT_FACTS_MAX 256

/**
 * @brief Record a subsystem's boot result. Called from main.c init phases.
 *
 * Only the fields without a live runtime accessor (currently the camera) are
 * read back from here; the rest are refreshed live in self_report_collect().
 * Recording all three still feeds the boot-time summary log.
 */
void self_report_note_init(self_report_subsystem_t subsystem, bool ok);

/**
 * @brief Fill @p out with the current health snapshot.
 *
 * Refreshes the dynamic bits from live accessors (wifi_is_connected,
 * i2c_bus_is_ready, audio_player_is_ready, gpio_expander_available,
 * get_gemini_api_key) and folds in the boot-recorded camera result.
 */
void self_report_collect(robocar_status_t *out);

/**
 * @brief Whether the robot can currently speak: WiFi + API key + audio.
 */
bool self_report_voice_able(const robocar_status_t *status);

/**
 * @brief Render the snapshot as a compact `key=state` facts line.
 *
 * Used for BOTH the Gemini narration prompt and the on-device template
 * fallback, and for the serial/MQTT diagnostic log.
 *
 * @return strlen of the written string (may be truncated to @p len - 1).
 */
size_t self_report_format_facts(const robocar_status_t *status, char *buf, size_t len);

/**
 * @brief Start the status monitor task (Core 1).
 *
 * Polls ~every 3 s: logs (and MQTT-publishes) the facts, and — when voice-able
 * and either announcing for the first time or the health signature changed —
 * narrates a spoken line and posts it to the speech_queue. Idempotent.
 *
 * @return ESP_OK, or ESP_FAIL if the task could not be created.
 */
esp_err_t self_report_start(void);

#ifdef __cplusplus
}
#endif
