/**
 * @file ota_update_decision.h
 * @brief Whether a fetched OTA manifest may be installed, given the running
 *        firmware's version and build commit SHA (issue #627).
 *
 * Pure C, no ESP-IDF or FreeRTOS dependency — see .claude/rules/testing.md's
 * shared-core pattern. Like ota_version_compare.h, this sits between a
 * network-fetched string and a call to esp_https_ota(), and none of its
 * branches can be staged on a bench, so it is host-tested.
 *
 * Why a SHA as well as a version: build-firmware.yml rebuilds EVERY
 * flasher-enabled project at the commit of whichever release fired it. When
 * another project releases, robocar-unified's binary on Pages is rebuilt from
 * a later commit while version.txt still names the last robocar-unified
 * release — the same version string over different contents. The version
 * alone cannot see that; the build SHA the manifest now carries
 * (tools/generate-flasher-manifests.sh, "buildSha") and the one compiled into
 * the firmware (ROBOCAR_BUILD_SHA, from git at CMake configure) can.
 *
 * The SHA never authorises an install on its own. Only a strictly newer
 * version does; the SHA decides how a non-install is reported, and a manifest
 * without a valid one is refused outright (fail closed).
 */

#ifndef OTA_UPDATE_DECISION_H
#define OTA_UPDATE_DECISION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Length of a full git commit SHA-1 in hex, without the terminator. */
#define OTA_BUILD_SHA_HEX_LEN 40

typedef enum {
    /** The manifest's version is strictly newer — install it. The only
     *  outcome that may lead to esp_https_ota(). */
    OTA_DECISION_UPDATE = 0,
    /** Same version and same build SHA: this exact build is already running. */
    OTA_DECISION_UP_TO_DATE,
    /** The manifest's version is older than the running one. No downgrades. */
    OTA_DECISION_OLDER_REMOTE,
    /** Same version, different build SHA: the published binary was built from
     *  a different commit under the same version string. Refused and
     *  reported — installing it would put unreleased commits on the robot
     *  under a released version's name. */
    OTA_DECISION_REFUSE_SHA_MISMATCH,
    /** Same version, but the running firmware carries no usable build SHA (a
     *  local build from a dirty tree, or one where git was unavailable at
     *  configure time). Nothing to compare against, so nothing is installed. */
    OTA_DECISION_RUNNING_SHA_UNKNOWN,
    /** The manifest has no "buildSha", or one that is not a full lowercase
     *  hex SHA — an older generator, or a broken one. Refused regardless of
     *  version (fail closed). */
    OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING,
    /** Either version string failed to parse. Fails closed, as
     *  ota_version_compare() does. */
    OTA_DECISION_INVALID_VERSION,
} ota_update_decision_t;

/**
 * True iff @p sha is exactly OTA_BUILD_SHA_HEX_LEN lowercase hex digits —
 * the form `git rev-parse HEAD` prints. NULL, empty, abbreviated, uppercase,
 * "unknown" and a "-dirty" suffix are all invalid.
 */
bool ota_build_sha_valid(const char *sha);

/**
 * Decide what to do with a fetched manifest.
 *
 * @param current_version esp_app_get_description()->version.
 * @param current_sha     The build SHA compiled into the running firmware;
 *                        may be NULL or invalid ("unknown", "...-dirty").
 * @param remote_version  The manifest's "version" field.
 * @param remote_sha      The manifest's "buildSha" field; NULL when absent.
 *
 * Precedence: an unparseable version first, then a missing/invalid manifest
 * SHA, then the version ordering, then — only for equal versions — the SHA
 * comparison.
 */
ota_update_decision_t ota_update_decide(const char *current_version, const char *current_sha,
                                        const char *remote_version, const char *remote_sha);

/** Stable lowercase token for logs and the MQTT status payload. */
const char *ota_update_decision_name(ota_update_decision_t decision);

#ifdef __cplusplus
}
#endif

#endif  // OTA_UPDATE_DECISION_H
