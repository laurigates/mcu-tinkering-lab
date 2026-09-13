/**
 * @file ota_version_compare.h
 * @brief Dotted-numeric (semver-shaped) version comparison for the OTA
 *        manifest poll.
 *
 * Pure C, no ESP-IDF or FreeRTOS dependency — see
 * .claude/rules/testing.md's shared-core pattern. This is the one part of
 * `ota_manager.c` a bench cannot exercise (a manifest with a bad or absent
 * "version" field is a network-fetched string, not something you can stage on
 * a robot), and it decides whether an update is allowed to start — so it is
 * host-tested rather than trusted to a device boot.
 */

#ifndef OTA_VERSION_COMPARE_H
#define OTA_VERSION_COMPARE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Result of comparing a remote manifest version against the running one. */
typedef enum {
    /** Either string failed to parse as a dotted-numeric version (NULL,
     *  empty, non-digit characters, an empty component, or more components
     *  than this parser supports). Fails CLOSED: the caller must treat this
     *  exactly like "no update", never guess. */
    OTA_VERSION_COMPARE_INVALID = -1,
    /** remote <= current, component-wise and numerically — including an
     *  exact match and a remote that is OLDER than current (no downgrade). */
    OTA_VERSION_COMPARE_NO_UPDATE = 0,
    /** remote > current, component-wise and numerically. */
    OTA_VERSION_COMPARE_UPDATE_AVAILABLE = 1,
} ota_version_compare_result_t;

/**
 * Compare @p remote_version against @p current_version as dotted-numeric
 * strings (e.g. "0.2.10"), component by component, NUMERICALLY rather than
 * lexically — "0.2.10" is newer than "0.2.9", even though it sorts earlier as
 * a string. Missing trailing components compare as 0 ("1.2" == "1.2.0").
 *
 * Fails CLOSED: a NULL, empty, or malformed string on EITHER side returns
 * OTA_VERSION_COMPARE_INVALID. The caller must log this and treat it as "no
 * update" — never start a flash on an unparsed comparison.
 *
 * @param current_version The version currently running (from
 *        esp_app_get_description()->version).
 * @param remote_version   The "version" field read out of the OTA manifest.
 */
ota_version_compare_result_t ota_version_compare(const char *current_version,
                                                 const char *remote_version);

#ifdef __cplusplus
}
#endif

#endif  // OTA_VERSION_COMPARE_H
