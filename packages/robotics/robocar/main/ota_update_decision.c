/**
 * @file ota_update_decision.c
 * @brief See ota_update_decision.h.
 */

#include "ota_update_decision.h"

#include <stddef.h>
#include <string.h>

#include "ota_version_compare.h"

bool ota_build_sha_valid(const char *sha)
{
    if (sha == NULL) {
        return false;
    }
    size_t i = 0;
    for (; sha[i] != '\0'; i++) {
        if (i >= OTA_BUILD_SHA_HEX_LEN) {
            return false; /* too long — includes a "-dirty" suffix */
        }
        const char c = sha[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return i == OTA_BUILD_SHA_HEX_LEN;
}

ota_update_decision_t ota_update_decide(const char *current_version, const char *current_sha,
                                        const char *remote_version, const char *remote_sha)
{
    const ota_version_compare_result_t forward =
        ota_version_compare(current_version, remote_version);
    if (forward == OTA_VERSION_COMPARE_INVALID) {
        return OTA_DECISION_INVALID_VERSION;
    }

    if (!ota_build_sha_valid(remote_sha)) {
        return OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING;
    }

    if (forward == OTA_VERSION_COMPARE_UPDATE_AVAILABLE) {
        return OTA_DECISION_UPDATE;
    }

    /* ota_version_compare() reports both "equal" and "remote older" as
     * NO_UPDATE. Asking the reverse question separates them: the running
     * version is newer than the remote exactly when the swapped comparison
     * says an update is available. */
    if (ota_version_compare(remote_version, current_version) ==
        OTA_VERSION_COMPARE_UPDATE_AVAILABLE) {
        return OTA_DECISION_OLDER_REMOTE;
    }

    if (!ota_build_sha_valid(current_sha)) {
        return OTA_DECISION_RUNNING_SHA_UNKNOWN;
    }
    if (strcmp(current_sha, remote_sha) != 0) {
        return OTA_DECISION_REFUSE_SHA_MISMATCH;
    }
    return OTA_DECISION_UP_TO_DATE;
}

const char *ota_update_decision_name(ota_update_decision_t decision)
{
    switch (decision) {
        case OTA_DECISION_UPDATE:
            return "update";
        case OTA_DECISION_UP_TO_DATE:
            return "up_to_date";
        case OTA_DECISION_OLDER_REMOTE:
            return "older_remote";
        case OTA_DECISION_REFUSE_SHA_MISMATCH:
            return "sha_mismatch";
        case OTA_DECISION_RUNNING_SHA_UNKNOWN:
            return "running_sha_unknown";
        case OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING:
            return "manifest_sha_missing";
        case OTA_DECISION_INVALID_VERSION:
            return "invalid_version";
    }
    return "unknown";
}
