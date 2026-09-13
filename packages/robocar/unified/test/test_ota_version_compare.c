/**
 * @file test_ota_version_compare.c
 * @brief Host tests for the OTA manifest version comparison (issue #539).
 *
 * The property under test is the fail-closed contract: an update may start
 * ONLY on a numeric remote > numeric current comparison that both sides
 * parsed cleanly. Everything else — equal, older, or unparseable on either
 * side — must read as "no update", because the caller hands the result
 * straight to esp_https_ota with no second check.
 */

#include "ota_version_compare.h"

#include <assert.h>
#include <stdio.h>

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

/* "0.2.10" is newer than "0.2.9" NUMERICALLY. A lexical strcmp() gets this
 * backwards ('1' < '9'), which is exactly the bug this module exists to
 * avoid. */
static void test_numeric_not_lexical(void)
{
    ASSERT(ota_version_compare("0.2.9", "0.2.10") == OTA_VERSION_COMPARE_UPDATE_AVAILABLE);
    ASSERT(ota_version_compare("0.2.9", "0.2.100") == OTA_VERSION_COMPARE_UPDATE_AVAILABLE);
    ASSERT(ota_version_compare("1.9.0", "1.10.0") == OTA_VERSION_COMPARE_UPDATE_AVAILABLE);
}

/* Identical versions never trigger an update. */
static void test_equal_versions_no_update(void)
{
    ASSERT(ota_version_compare("0.2.10", "0.2.10") == OTA_VERSION_COMPARE_NO_UPDATE);
    ASSERT(ota_version_compare("1.0.0", "1.0.0") == OTA_VERSION_COMPARE_NO_UPDATE);
    /* Missing trailing components compare as zero. */
    ASSERT(ota_version_compare("1.2", "1.2.0") == OTA_VERSION_COMPARE_NO_UPDATE);
    ASSERT(ota_version_compare("1.2.0", "1.2") == OTA_VERSION_COMPARE_NO_UPDATE);
}

/* A remote version older than what is already running must never be treated
 * as an update — no downgrades. */
static void test_older_remote_no_downgrade(void)
{
    ASSERT(ota_version_compare("0.2.10", "0.2.9") == OTA_VERSION_COMPARE_NO_UPDATE);
    ASSERT(ota_version_compare("1.10.0", "1.9.0") == OTA_VERSION_COMPARE_NO_UPDATE);
    ASSERT(ota_version_compare("2.0.0", "1.99.99") == OTA_VERSION_COMPARE_NO_UPDATE);
}

/* Malformed or empty strings fail CLOSED: never an update, on either side. */
static void test_malformed_fails_closed(void)
{
    ASSERT(ota_version_compare(NULL, "0.2.10") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", NULL) == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("", "0.2.10") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "v0.2.11") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "0.2.11-beta") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "0..11") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", ".2.11") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "0.2.11.") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("0.2.10", "0.2.11.0.0") == OTA_VERSION_COMPARE_INVALID);
    ASSERT(ota_version_compare("not-a-version", "0.2.10") == OTA_VERSION_COMPARE_INVALID);
}

int main(void)
{
    printf("=== OTA version comparison host tests ===\n\n");

    test_run("numeric_not_lexical", test_numeric_not_lexical);
    test_run("equal_versions_no_update", test_equal_versions_no_update);
    test_run("older_remote_no_downgrade", test_older_remote_no_downgrade);
    test_run("malformed_fails_closed", test_malformed_fails_closed);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
