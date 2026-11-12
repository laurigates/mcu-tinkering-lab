/**
 * @file ota_announce.h
 * @brief What the robot says before an OTA install, and the window in which
 *        the owner can postpone it: pure C, host-tested.
 *
 * The sequence, driven by ota_manager.c when a newer version is found:
 *
 *   1. a line saying an update is available and what it changes, phrased by
 *      gemini_backend_announce_update() from ota_announce_facts(), or
 *      ota_announce_fallback() when that request fails;
 *   2. OTA_ANNOUNCE_PUN, spoken verbatim;
 *   3. a cancel window of OTA_ANNOUNCE_WINDOW_MS in which `ota postpone`
 *      (console or MQTT) defers the install to the next check;
 *   4. the park (ota_park.h), then the install.
 *
 * The notes come from the manifest's "notes" array (tools/changelog-notes.py):
 * English "scope: summary" lines. They go to the model as facts, never to the
 * speaker directly.
 */

#ifndef OTA_ANNOUNCE_H
#define OTA_ANNOUNCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Spoken verbatim after the announcement. The pun is intended: päivittyä
 *  (to update) and päivettyä (to get a tan). */
#define OTA_ANNOUNCE_PUN "Menen tuonne toisaalle vähän päivettymään."

/** Spoken when the install fails and the robot carries on with the running
 *  image. */
#define OTA_ANNOUNCE_FAILED_FI "Päivitys ei onnistunut, jatkan vanhalla versiolla."
#define OTA_ANNOUNCE_FAILED_EN "The update did not work, so I am carrying on as I was."

/** How long `ota postpone` is accepted after the pun. */
#define OTA_ANNOUNCE_WINDOW_MS 60000U

/** Notes read from a manifest; more are ignored. Matches changelog-notes.py. */
#define OTA_ANNOUNCE_MAX_NOTES 8

/**
 * @brief Build the facts string the announcement prompt is given.
 *
 * "Firmware version <v> is available (running <cur>). Changes: <n1>; <n2>."
 * Notes are added whole or not at all, in order, so the string never ends in
 * half a note or half a UTF-8 sequence. With no notes that fit, the changes
 * clause is "Changes: not listed."
 *
 * @return Length written, excluding the NUL; 0 if a required argument is NULL
 *         or @p out_len is too small for the version clause.
 */
size_t ota_announce_facts(const char *new_version, const char *running_version,
                          const char *const *notes, size_t n_notes, char *out, size_t out_len);

/**
 * @brief The line spoken when the announcement request fails.
 *
 * Finnish when @p finnish, English otherwise; names the version only.
 * @return Length written, excluding the NUL.
 */
size_t ota_announce_fallback(const char *new_version, bool finnish, char *out, size_t out_len);

typedef enum {
    OTA_WINDOW_OPEN = 0,  /**< still accepting a postpone */
    OTA_WINDOW_POSTPONED, /**< the owner postponed; skip this install */
    OTA_WINDOW_EXPIRED,   /**< no postpone in time; go ahead */
} ota_window_state_t;

typedef struct {
    uint32_t opened_ms;
    uint32_t length_ms;
    bool postponed;
    bool open;
} ota_window_t;

/** Open the window at @p now_ms for @p length_ms. */
void ota_window_open(ota_window_t *w, uint32_t now_ms, uint32_t length_ms);

/** Record a postpone. Returns true if the window was open to take it. */
bool ota_window_postpone(ota_window_t *w, uint32_t now_ms);

/** The window's state at @p now_ms (uint32 wrap-safe). */
ota_window_state_t ota_window_state(const ota_window_t *w, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* OTA_ANNOUNCE_H */
