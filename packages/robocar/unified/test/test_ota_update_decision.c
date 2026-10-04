/**
 * @file test_ota_update_decision.c
 * @brief Host tests for the OTA install decision (issue #627).
 *
 * The property under test: esp_https_ota() may be reached ONLY through
 * OTA_DECISION_UPDATE, and that outcome requires a cleanly parsed, strictly
 * newer version AND a manifest carrying a valid build SHA. Every other input
 * — equal, older, unparseable, SHA missing or malformed — must land on a
 * non-install outcome, and an equal version must be reported by what the SHA
 * comparison found rather than collapsed into one "no update".
 */

#include "ota_update_decision.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

static void test_assert(int cond, const char *file, int line, const char *expr)
{
    if (!cond) {
        printf("FAIL: %s:%d assertion failed: %s\n", file, line, expr);
        assert(cond);
    }
}

#define ASSERT(cond) test_assert((cond), __FILE__, __LINE__, #cond)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] Running: %s...\n", test_count, name);
    fflush(stdout);
    fn();
    test_pass++;
    printf("     PASS\n");
}

/* SHAs are built from a repeated character so their length is exact by
 * construction rather than by counting a hand-typed literal. */
static char sha_a[OTA_BUILD_SHA_HEX_LEN + 1];
static char sha_b[OTA_BUILD_SHA_HEX_LEN + 1];

static void fill(char *buf, size_t len, char c)
{
    memset(buf, c, len);
    buf[len] = '\0';
}

static void setup_shas(void)
{
    fill(sha_a, OTA_BUILD_SHA_HEX_LEN, 'a');
    fill(sha_b, OTA_BUILD_SHA_HEX_LEN, '0');
    sha_b[OTA_BUILD_SHA_HEX_LEN - 1] = 'f'; /* mixed digits and letters */
}

static void test_sha_valid(void)
{
    char short_sha[OTA_BUILD_SHA_HEX_LEN];
    char long_sha[OTA_BUILD_SHA_HEX_LEN + 2];
    char upper[OTA_BUILD_SHA_HEX_LEN + 1];
    char non_hex[OTA_BUILD_SHA_HEX_LEN + 1];
    char dirty[OTA_BUILD_SHA_HEX_LEN + 7];

    fill(short_sha, OTA_BUILD_SHA_HEX_LEN - 1, 'a');
    fill(long_sha, OTA_BUILD_SHA_HEX_LEN + 1, 'a');
    fill(upper, OTA_BUILD_SHA_HEX_LEN, 'A');
    fill(non_hex, OTA_BUILD_SHA_HEX_LEN, 'a');
    non_hex[7] = 'g';
    snprintf(dirty, sizeof(dirty), "%s-dirty", sha_a);

    ASSERT(ota_build_sha_valid(sha_a));
    ASSERT(ota_build_sha_valid(sha_b));
    ASSERT(!ota_build_sha_valid(NULL));
    ASSERT(!ota_build_sha_valid(""));
    ASSERT(!ota_build_sha_valid("unknown"));
    ASSERT(!ota_build_sha_valid(short_sha)); /* abbreviated */
    ASSERT(!ota_build_sha_valid(long_sha));
    ASSERT(!ota_build_sha_valid(upper));
    ASSERT(!ota_build_sha_valid(non_hex));
    ASSERT(!ota_build_sha_valid(dirty)); /* a dirty build is not that commit */
}

/* A strictly newer version installs, whatever the running build's SHA. */
static void test_newer_version_updates(void)
{
    ASSERT(ota_update_decide("0.2.9", sha_a, "0.2.10", sha_b) == OTA_DECISION_UPDATE);
    ASSERT(ota_update_decide("0.2.9", sha_a, "0.2.10", sha_a) == OTA_DECISION_UPDATE);
    /* A dev build with no usable SHA still takes a real release. */
    ASSERT(ota_update_decide("0.2.9", "unknown", "0.2.10", sha_b) == OTA_DECISION_UPDATE);
    ASSERT(ota_update_decide("0.2.9", NULL, "0.2.10", sha_b) == OTA_DECISION_UPDATE);
}

static void test_same_version_same_sha_up_to_date(void)
{
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.10", sha_a) == OTA_DECISION_UP_TO_DATE);
    /* "1.2" and "1.2.0" are the same version to ota_version_compare(). */
    ASSERT(ota_update_decide("1.2", sha_a, "1.2.0", sha_a) == OTA_DECISION_UP_TO_DATE);
}

/* The case the issue is about: a binary rebuilt from another commit under
 * the same version string. Refused, and reported as such. */
static void test_same_version_different_sha_refused(void)
{
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.10", sha_b) == OTA_DECISION_REFUSE_SHA_MISMATCH);
    ASSERT(ota_update_decide("1.2.0", sha_b, "1.2", sha_a) == OTA_DECISION_REFUSE_SHA_MISMATCH);
}

static void test_same_version_running_sha_unknown(void)
{
    char dirty[OTA_BUILD_SHA_HEX_LEN + 7];
    snprintf(dirty, sizeof(dirty), "%s-dirty", sha_a);

    ASSERT(ota_update_decide("0.2.10", "unknown", "0.2.10", sha_a) ==
           OTA_DECISION_RUNNING_SHA_UNKNOWN);
    ASSERT(ota_update_decide("0.2.10", NULL, "0.2.10", sha_a) == OTA_DECISION_RUNNING_SHA_UNKNOWN);
    /* A dirty build at the published commit must not read as up to date:
     * its contents are not that commit. */
    ASSERT(ota_update_decide("0.2.10", dirty, "0.2.10", sha_a) == OTA_DECISION_RUNNING_SHA_UNKNOWN);
}

/* No downgrades, and an older remote is reported as older even when its SHA
 * differs — the SHA only qualifies an equal version. */
static void test_older_remote(void)
{
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.9", sha_b) == OTA_DECISION_OLDER_REMOTE);
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.9", sha_a) == OTA_DECISION_OLDER_REMOTE);
    ASSERT(ota_update_decide("0.2.10", "unknown", "0.2.9", sha_a) == OTA_DECISION_OLDER_REMOTE);
}

/* A manifest from an older generator has no buildSha. Fail closed — even
 * when its version is newer, which is the case that would otherwise flash. */
static void test_manifest_sha_missing_fails_closed(void)
{
    char short_sha[OTA_BUILD_SHA_HEX_LEN];
    fill(short_sha, OTA_BUILD_SHA_HEX_LEN - 1, 'a');

    ASSERT(ota_update_decide("0.2.9", sha_a, "0.2.10", NULL) ==
           OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING);
    ASSERT(ota_update_decide("0.2.9", sha_a, "0.2.10", "") ==
           OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING);
    ASSERT(ota_update_decide("0.2.9", sha_a, "0.2.10", short_sha) ==
           OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING);
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.10", NULL) ==
           OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING);
    ASSERT(ota_update_decide("0.2.10", sha_a, "0.2.9", NULL) ==
           OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING);
}

static void test_invalid_version_fails_closed(void)
{
    ASSERT(ota_update_decide(NULL, sha_a, "0.2.10", sha_b) == OTA_DECISION_INVALID_VERSION);
    ASSERT(ota_update_decide("0.2.9", sha_a, "v0.2.10", sha_b) == OTA_DECISION_INVALID_VERSION);
    /* An unparseable version outranks a missing SHA: both refuse, and the
     * version is the more basic defect to report. */
    ASSERT(ota_update_decide("0.2.9", sha_a, "", NULL) == OTA_DECISION_INVALID_VERSION);
}

/* Exhaustive guard on the property that matters most: across every
 * combination of the inputs above, UPDATE appears only for a newer version
 * with a valid manifest SHA. */
static void test_update_only_on_newer_with_valid_sha(void)
{
    const char *versions[] = {"0.2.9", "0.2.10", "0.3", "", "x", NULL};
    const char *shas[] = {sha_a, sha_b, "unknown", "", NULL};
    const size_t nv = sizeof(versions) / sizeof(versions[0]);
    const size_t ns = sizeof(shas) / sizeof(shas[0]);

    for (size_t cv = 0; cv < nv; cv++) {
        for (size_t rv = 0; rv < nv; rv++) {
            for (size_t cs = 0; cs < ns; cs++) {
                for (size_t rs = 0; rs < ns; rs++) {
                    ota_update_decision_t d =
                        ota_update_decide(versions[cv], shas[cs], versions[rv], shas[rs]);
                    if (d != OTA_DECISION_UPDATE) {
                        continue;
                    }
                    ASSERT(ota_build_sha_valid(shas[rs]));
                    /* cv < rv in this list's order exactly when rv is newer
                     * and both parse (indices 0..2 are valid, ascending). */
                    ASSERT(cv < rv && rv <= 2);
                }
            }
        }
    }
}

static void test_decision_names(void)
{
    ASSERT(strcmp(ota_update_decision_name(OTA_DECISION_UPDATE), "update") == 0);
    ASSERT(strcmp(ota_update_decision_name(OTA_DECISION_REFUSE_SHA_MISMATCH), "sha_mismatch") == 0);
    ASSERT(strcmp(ota_update_decision_name(OTA_DECISION_REFUSE_MANIFEST_SHA_MISSING),
                  "manifest_sha_missing") == 0);
    ASSERT(strcmp(ota_update_decision_name((ota_update_decision_t)99), "unknown") == 0);
}

int main(void)
{
    printf("=== OTA update decision host tests ===\n\n");
    setup_shas();

    test_run("sha_valid", test_sha_valid);
    test_run("newer_version_updates", test_newer_version_updates);
    test_run("same_version_same_sha_up_to_date", test_same_version_same_sha_up_to_date);
    test_run("same_version_different_sha_refused", test_same_version_different_sha_refused);
    test_run("same_version_running_sha_unknown", test_same_version_running_sha_unknown);
    test_run("older_remote", test_older_remote);
    test_run("manifest_sha_missing_fails_closed", test_manifest_sha_missing_fails_closed);
    test_run("invalid_version_fails_closed", test_invalid_version_fails_closed);
    test_run("update_only_on_newer_with_valid_sha", test_update_only_on_newer_with_valid_sha);
    test_run("decision_names", test_decision_names);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
