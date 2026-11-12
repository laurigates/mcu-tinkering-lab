/**
 * @file test_oled_status_core.c
 * @brief Host tests for the OLED status screen's row formatter.
 *
 * Pins what the bench cannot stage on demand: a tripped planner fuse, one
 * rangefinder failing beside a working one, a dead I2C bus with every
 * peripheral flag false, and the widest value in every field. The last group
 * matters because the panel holds 21 characters and a row that overruns is
 * clipped silently, so every row must fit and use only characters the font
 * carries (oled.c renders anything else as `?`).
 */

#include <stdio.h>
#include <string.h>

#include "oled_status_core.h"

static int g_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                            \
        }                                                            \
    } while (0)

#define ASSERT_ROW(rows, i, expected)                                                       \
    do {                                                                                    \
        if (strcmp((rows)[i], (expected)) != 0) {                                           \
            printf("  FAIL %s:%d: row %d = \"%s\", want \"%s\"\n", __FILE__, __LINE__, (i), \
                   (rows)[i], (expected));                                                  \
            g_failures++;                                                                   \
        }                                                                                   \
    } while (0)

static char g_rows[OLED_STATUS_ROWS][OLED_STATUS_COLS + 1];

/** A healthy, connected, dormant robot with both rangefinders reading. */
static oled_status_t healthy(void)
{
    oled_status_t s = {
        .wifi_up = true,
        .ip = {192, 168, 0, 42},
        .i2c_bus_ok = true,
        .motors_ok = true,
        .servos_ok = true,
        .leds_ok = true,
        .plan_gate_on = true,
        .plan_dormant = true,
        .plan_requests = 12,
        .plan_max_requests = 500,
        .sonar_known = true,
        .front_cm = 123,
        .rear_cm = 45,
    };
    return s;
}

/** The font: A-Z, 0-9, space and `- _ . / :` (oled.c glyph()). */
static int row_is_renderable(const char *row)
{
    for (const char *p = row; *p; p++) {
        const char c = *p;
        const int ok = (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ' || c == '-' ||
                       c == '_' || c == '.' || c == '/' || c == ':';
        if (!ok) {
            return 0;
        }
    }
    return 1;
}

static void assert_all_rows_fit_and_render(void)
{
    for (int i = 0; i < OLED_STATUS_ROWS; i++) {
        ASSERT(strlen(g_rows[i]) <= OLED_STATUS_COLS);
        ASSERT(row_is_renderable(g_rows[i]));
    }
}

static void test_healthy_robot(void)
{
    const oled_status_t s = healthy();
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 0, "IP 192.168.0.42");
    ASSERT_ROW(g_rows, 1, "HW OK");
    ASSERT_ROW(g_rows, 2, "PLAN SLEEP 12/500");
    ASSERT_ROW(g_rows, 3, "SONAR F 123 R 045");
}

static void test_no_wifi(void)
{
    oled_status_t s = healthy();
    s.wifi_up = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 0, "WIFI DOWN");
}

static void test_dead_bus_outranks_peripheral_flags(void)
{
    oled_status_t s = healthy();
    s.i2c_bus_ok = false;
    s.motors_ok = s.servos_ok = s.leds_ok = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 1, "HW NO I2C BUS");
}

static void test_each_missing_peripheral_is_named(void)
{
    oled_status_t s = healthy();
    s.motors_ok = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 1, "HW -MOT");

    s.servos_ok = false;
    s.leds_ok = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 1, "HW -MOT -SRV -LED");
}

static void test_planner_states(void)
{
    oled_status_t s = healthy();
    s.plan_dormant = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 2, "PLAN AWAKE 12/500");

    s.plan_gate_on = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 2, "PLAN NOGATE 12/500");

    /* A tripped fuse outranks the gate state: it is the one that needs action. */
    s.plan_fused = true;
    s.plan_requests = 500;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 2, "PLAN FUSED 500/500");
}

static void test_no_request_ceiling(void)
{
    oled_status_t s = healthy();
    s.plan_max_requests = 0;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 2, "PLAN SLEEP 12/-");
}

static void test_sonar_failures_and_reflex(void)
{
    oled_status_t s = healthy();
    s.rear_failed = true;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 3, "SONAR F 123 R ERR");

    s.front_failed = true;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 3, "SONAR F ERR R ERR");

    s = healthy();
    s.reflex_active = true;
    s.front_cm = 7;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 3, "STOP  F 007 R 045");

    /* UINT16_MAX is ULTRASONIC_DIST_ERROR: never print it as a distance. */
    s = healthy();
    s.rear_cm = 65535;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 3, "SONAR F 123 R ERR");
}

static void test_no_telemetry_yet(void)
{
    oled_status_t s = healthy();
    s.sonar_known = false;
    oled_status_format(&s, g_rows);
    ASSERT_ROW(g_rows, 3, "SONAR --");
}

static void test_widest_values_fit_and_render(void)
{
    oled_status_t s = healthy();
    s.ip[0] = s.ip[1] = s.ip[2] = s.ip[3] = 255;
    s.motors_ok = s.servos_ok = s.leds_ok = false;
    s.plan_gate_on = false;
    s.plan_dormant = false;
    s.plan_requests = 4294967295u;
    s.plan_max_requests = 4294967295u;
    s.reflex_active = true;
    s.front_failed = true;
    s.rear_failed = true;
    oled_status_format(&s, g_rows);
    assert_all_rows_fit_and_render();

    const oled_status_t h = healthy();
    oled_status_format(&h, g_rows);
    assert_all_rows_fit_and_render();
}

int main(void)
{
    test_healthy_robot();
    test_no_wifi();
    test_dead_bus_outranks_peripheral_flags();
    test_each_missing_peripheral_is_named();
    test_planner_states();
    test_no_request_ceiling();
    test_sonar_failures_and_reflex();
    test_no_telemetry_yet();
    test_widest_values_fit_and_render();

    if (g_failures) {
        printf("oled_status_core: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("oled_status_core: all tests passed\n");
    return 0;
}
