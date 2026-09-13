/**
 * @file ota_version_compare.c
 * @brief See ota_version_compare.h.
 */

#include "ota_version_compare.h"

#include <stdbool.h>
#include <stddef.h>

/** "0.2.10" has three; leave headroom for a build/patch component nobody
 *  currently uses without silently truncating a longer string into a match. */
#define OTA_VERSION_MAX_COMPONENTS 4

/**
 * Parse a dotted-numeric version string into up to
 * OTA_VERSION_MAX_COMPONENTS unsigned components, zero-filling any that are
 * absent (so "1.2" parses as {1, 2, 0, 0}).
 *
 * Returns true on success. Returns false — and leaves @p out untouched by
 * the caller's contract, since the caller must not act on it — for: NULL or
 * empty input, any character that is not a digit or '.', an empty component
 * (leading '.', trailing '.', or ".."), or more than OTA_VERSION_MAX_COMPONENTS
 * components. This is the fail-closed boundary: anything this function cannot
 * confidently parse is treated as malformed, not as "0".
 */
static bool parse_version(const char *s, unsigned long out[OTA_VERSION_MAX_COMPONENTS])
{
    if (s == NULL || *s == '\0') {
        return false;
    }

    for (int i = 0; i < OTA_VERSION_MAX_COMPONENTS; i++) {
        out[i] = 0;
    }

    int component = 0;
    const char *p = s;
    while (*p != '\0') {
        if (component >= OTA_VERSION_MAX_COMPONENTS) {
            return false; /* more components than this parser supports */
        }
        if (*p < '0' || *p > '9') {
            return false; /* stray character, or an empty component */
        }

        unsigned long value = 0;
        while (*p >= '0' && *p <= '9') {
            value = (value * 10) + (unsigned long)(*p - '0');
            p++;
        }
        out[component++] = value;

        if (*p == '.') {
            p++;
            if (*p == '\0') {
                return false; /* trailing dot */
            }
        } else if (*p != '\0') {
            return false; /* something other than a digit or a dot */
        }
    }

    return true;
}

ota_version_compare_result_t ota_version_compare(const char *current_version,
                                                 const char *remote_version)
{
    unsigned long current[OTA_VERSION_MAX_COMPONENTS];
    unsigned long remote[OTA_VERSION_MAX_COMPONENTS];

    if (!parse_version(current_version, current) || !parse_version(remote_version, remote)) {
        return OTA_VERSION_COMPARE_INVALID;
    }

    for (int i = 0; i < OTA_VERSION_MAX_COMPONENTS; i++) {
        if (remote[i] > current[i]) {
            return OTA_VERSION_COMPARE_UPDATE_AVAILABLE;
        }
        if (remote[i] < current[i]) {
            return OTA_VERSION_COMPARE_NO_UPDATE;
        }
    }

    return OTA_VERSION_COMPARE_NO_UPDATE; /* exactly equal */
}
