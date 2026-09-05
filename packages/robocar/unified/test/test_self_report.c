/**
 * @file test_self_report.c
 * @brief Per-peripheral health reporting (issue #500).
 *
 * The hardware phase used to abort the boot when any one of the PCA9685-backed
 * peripherals failed to initialise, so "which peripheral is missing?" was a
 * question the firmware never had to answer — the board just reboot-looped.
 * Now it keeps booting, which makes the self-report the only thing standing
 * between a half-populated board and a silent, unexplained loss of function.
 *
 * The cases below are the ones a bench cannot stage: a working I2C bus with a
 * servo that refuses to initialise, or a buzzer whose GPIO config failed. Both
 * need a fault injected at exactly one peripheral while the rest come up.
 *
 * self_report.c is compiled UNMODIFIED against the shims in test/include and
 * the recording stubs in self_report_stubs.c, so the code under test is the
 * code the device runs.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "self_report.h"
#include "self_report_stubs.h"

static int g_checks = 0;

#define CHECK(cond)                                                         \
    do {                                                                    \
        ++g_checks;                                                         \
        if (!(cond)) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                       \
        }                                                                   \
    } while (0)

/** Collect a snapshot and render it, returning the facts buffer. */
static void facts_now(char *buf, size_t len)
{
    robocar_status_t status;
    self_report_collect(&status);
    self_report_format_facts(&status, buf, len);
}

/* -------------------------------------------------------------------------- */

/** A fully populated, fully healthy board names no degraded peripheral. */
static int test_healthy_board_reports_ok(void)
{
    stub_reset_healthy();

    char facts[SELF_REPORT_FACTS_MAX];
    facts_now(facts, sizeof(facts));

    CHECK(strstr(facts, "i2c_peripherals=ok") != NULL);
    CHECK(strstr(facts, "buzzer=ok") != NULL);
    CHECK(strstr(facts, "degraded") == NULL);
    return 0;
}

/**
 * A bare board (no I2C hardware at all) must still read exactly
 * "not-responding". The persona's spoken fault line keys off i2c_bus_ok, and
 * this is the wording the bare-board path has always produced — a peripheral
 * breakdown must not change what a missing bus looks like.
 */
static int test_absent_bus_still_reads_not_responding(void)
{
    stub_reset_healthy();
    g_stub.i2c_bus_ready = false;
    /* With no bus, nothing behind it can have initialised either. */
    g_stub.motors_initialized = false;
    g_stub.leds_initialized = false;
    g_stub.servos_initialized = false;

    char facts[SELF_REPORT_FACTS_MAX];
    facts_now(facts, sizeof(facts));

    CHECK(strstr(facts, "i2c_peripherals=not-responding") != NULL);
    /* The buzzer is on a dedicated GPIO, so a dead bus must not implicate it. */
    CHECK(strstr(facts, "buzzer=ok") != NULL);
    return 0;
}

/**
 * The case issue #500 is about: the bus answers, one peripheral does not, and
 * the board keeps booting. The report has to name the peripheral — otherwise
 * the only difference from a healthy robot is that the pan/tilt head never
 * moves, with nothing anywhere saying why.
 */
static int test_single_failed_peripheral_is_named(void)
{
    stub_reset_healthy();
    g_stub.servos_initialized = false;

    char facts[SELF_REPORT_FACTS_MAX];
    facts_now(facts, sizeof(facts));

    CHECK(strstr(facts, "i2c_peripherals=degraded(servos)") != NULL);
    /* Still a live bus: it must not be reported as absent. */
    CHECK(strstr(facts, "not-responding") == NULL ||
          strstr(facts, "i2c_peripherals=not-responding") == NULL);
    return 0;
}

/** Several failures list in a stable order, so the facts line is diffable. */
static int test_multiple_failures_all_listed(void)
{
    stub_reset_healthy();
    g_stub.motors_initialized = false;
    g_stub.leds_initialized = false;
    g_stub.servos_initialized = false;

    char facts[SELF_REPORT_FACTS_MAX];
    facts_now(facts, sizeof(facts));

    CHECK(strstr(facts, "i2c_peripherals=degraded(motors,leds,servos)") != NULL);
    return 0;
}

/**
 * The buzzer is a GPIO peripheral, not an I2C one. A failed buzzer must be
 * reported without implying anything about the bus behind the PCA9685.
 */
static int test_buzzer_reported_separately(void)
{
    stub_reset_healthy();
    g_stub.buzzer_initialized = false;

    char facts[SELF_REPORT_FACTS_MAX];
    facts_now(facts, sizeof(facts));

    CHECK(strstr(facts, "buzzer=not-responding") != NULL);
    CHECK(strstr(facts, "i2c_peripherals=ok") != NULL);
    return 0;
}

/**
 * The facts line feeds the narration prompt and the MQTT status topic, and it
 * is assembled with snprintf into a fixed buffer — so the worst case (every
 * subsystem down, a maximum-length SSID and version) has to fit, or the tail
 * keys are silently truncated away exactly when they carry the most.
 */
static int test_worst_case_fits_the_buffer(void)
{
    stub_reset_healthy();
    /* Bus UP with every peripheral behind it down. Counter-intuitive, but this
     * is the longest line: "degraded(motors,leds,servos)" is fourteen
     * characters longer than the "not-responding" a dead bus renders, so
     * staging the *more* broken board (no bus at all) measures the SHORTER
     * string and passes on a buffer that overflows in the field. */
    g_stub.i2c_bus_ready = true;
    g_stub.motors_initialized = false;
    g_stub.leds_initialized = false;
    g_stub.servos_initialized = false;
    g_stub.buzzer_initialized = false;
    g_stub.audio_ready = false;
    g_stub.wifi_connected = false;
    /* Absent, not present: "absent(optional)" is nine characters longer than
     * "present", so the expander being FITTED is the shorter line. Getting this
     * backwards is what makes a worst-case test pass on a buffer that overflows
     * in the field. */
    g_stub.expander_available = false;
    g_stub.api_key = "";
    /* MAX_SSID_LENGTH - 1 characters, the longest strlcpy will keep. */
    g_stub.ssid = "abcdefghijklmnopqrstuvwxyz01234";
    g_stub.version = "1234567890.1234567890.12345678";

    robocar_status_t status;
    self_report_collect(&status);

    char facts[SELF_REPORT_FACTS_MAX];
    const size_t n = self_report_format_facts(&status, facts, sizeof(facts));

    /* strlcpy-style truncation would report len-1 and lose the tail. */
    CHECK(n < SELF_REPORT_FACTS_MAX - 1);
    /* The tail keys are the ones snprintf drops first, and they are exactly
     * the ones a degraded board most needs to report. */
    CHECK(strstr(facts, "i2c_peripherals=degraded(motors,leds,servos)") != NULL);
    CHECK(strstr(facts, "gemini_key=absent") != NULL);
    CHECK(strstr(facts, "buzzer=not-responding") != NULL);
    return 0;
}

/** collect() must read the live accessors, not a cached boot result. */
static int test_collect_reads_live_state(void)
{
    stub_reset_healthy();

    robocar_status_t status;
    self_report_collect(&status);
    CHECK(status.motors_ok);
    CHECK(status.leds_ok);
    CHECK(status.servos_ok);
    CHECK(status.buzzer_ok);

    g_stub.motors_initialized = false;
    self_report_collect(&status);
    CHECK(!status.motors_ok);
    CHECK(status.leds_ok);
    return 0;
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    struct {
        const char *name;
        int (*fn)(void);
    } tests[] = {
        {"healthy_board_reports_ok", test_healthy_board_reports_ok},
        {"absent_bus_still_reads_not_responding", test_absent_bus_still_reads_not_responding},
        {"single_failed_peripheral_is_named", test_single_failed_peripheral_is_named},
        {"multiple_failures_all_listed", test_multiple_failures_all_listed},
        {"buzzer_reported_separately", test_buzzer_reported_separately},
        {"worst_case_fits_the_buffer", test_worst_case_fits_the_buffer},
        {"collect_reads_live_state", test_collect_reads_live_state},
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        if (tests[i].fn() != 0) {
            fprintf(stderr, "test_self_report: %s FAILED\n", tests[i].name);
            return 1;
        }
        printf("  ok  %s\n", tests[i].name);
    }

    printf("test_self_report: all tests passed (%d checks)\n", g_checks);
    return 0;
}
