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

/* A component with more digits than an unsigned long accumulator can safely
 * hold (>9 digits) must fail closed rather than silently wrap — on either
 * side of the comparison. A 9-digit component is the boundary and must
 * still parse. */
static void test_overlong_component_fails_closed(void)
{
    ASSERT(ota_version_compare("0.2.10", "0.9999999999.0") ==
           OTA_VERSION_COMPARE_INVALID); /* 10 digits */
    ASSERT(ota_version_compare("0.9999999999.0", "0.2.10") ==
           OTA_VERSION_COMPARE_INVALID); /* 10 digits, current side */
    ASSERT(ota_version_compare("0.0.0", "0.999999999.0") ==
           OTA_VERSION_COMPARE_UPDATE_AVAILABLE); /* 9 digits: still valid */
}

/* ota_version_fits() gates the manifest "version" string against the
 * caller's fixed buffer BEFORE it is copied — a string that would need
 * truncation must be rejected outright, not silently shortened.
 *
 * Test strings are built from a fixed-length '9' run rather than hand-typed
 * digit literals, so the boundary is exact by construction instead of by
 * counting characters in a string literal. */
static void test_version_fits(void)
{
    char thirty_one[32]; /* 31 '9's + NUL */
    char thirty_two[33]; /* 32 '9's + NUL */
    for (int i = 0; i < 31; i++) {
        thirty_one[i] = '9';
    }
    thirty_one[31] = '\0';
    for (int i = 0; i < 32; i++) {
        thirty_two[i] = '9';
    }
    thirty_two[32] = '\0';

    ASSERT(ota_version_fits("0.2.10", 32) == true); /* fits with room to spare */
    ASSERT(ota_version_fits("", 1) == true);        /* empty string + NUL fits exactly */
    ASSERT(ota_version_fits(NULL, 32) == false);
    ASSERT(ota_version_fits("0.2.10", 0) == false);

    /* Exact boundary against a 32-byte buffer: a 31-char string (+ NUL == 32)
     * fits; a 32-char string (+ NUL == 33) does not. */
    ASSERT(ota_version_fits(thirty_one, 32) == true);
    ASSERT(ota_version_fits(thirty_two, 32) == false);
}

int main(void)
{
    printf("=== OTA version comparison host tests ===\n\n");

    test_run("numeric_not_lexical", test_numeric_not_lexical);
    test_run("equal_versions_no_update", test_equal_versions_no_update);
    test_run("older_remote_no_downgrade", test_older_remote_no_downgrade);
    test_run("malformed_fails_closed", test_malformed_fails_closed);
    test_run("overlong_component_fails_closed", test_overlong_component_fails_closed);
    test_run("version_fits", test_version_fits);

    printf("\n=== Results ===\n");
    printf("Passed: %d / %d\n", test_pass, test_count);
    return (test_pass == test_count) ? 0 : 1;
}
