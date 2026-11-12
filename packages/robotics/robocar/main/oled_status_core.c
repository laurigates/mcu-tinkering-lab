/**
 * @file oled_status_core.c
 * @brief Pure formatter for the OLED status screen (see oled_status_core.h).
 */

#include "oled_status_core.h"

#include <stdio.h>
#include <string.h>

/* Rows are formatted into a scratch buffer wide enough for any value, then
 * clipped to the panel. Formatting straight into a 22-byte row would make GCC
 * prove every field fits (-Wformat-truncation, an error in the ESP-IDF build),
 * and a 10-digit request count does not. */
#define SCRATCH_BYTES 64
#define ROW_BYTES SCRATCH_BYTES

/** Distances print as three digits; anything past 999 is not a reading. */
#define SONAR_MAX_PRINTABLE_CM 999u

static void row_network(const oled_status_t *s, char *out)
{
    if (!s->wifi_up) {
        snprintf(out, ROW_BYTES, "WIFI DOWN");
        return;
    }
    snprintf(out, ROW_BYTES, "IP %u.%u.%u.%u", (unsigned)s->ip[0], (unsigned)s->ip[1],
             (unsigned)s->ip[2], (unsigned)s->ip[3]);
}

static void row_hardware(const oled_status_t *s, char *out)
{
    if (!s->i2c_bus_ok) {
        snprintf(out, ROW_BYTES, "HW NO I2C BUS");
        return;
    }
    if (s->motors_ok && s->servos_ok && s->leds_ok) {
        snprintf(out, ROW_BYTES, "HW OK");
        return;
    }
    snprintf(out, ROW_BYTES, "HW%s%s%s", s->motors_ok ? "" : " -MOT", s->servos_ok ? "" : " -SRV",
             s->leds_ok ? "" : " -LED");
}

static void row_planner(const oled_status_t *s, char *out)
{
    const char *state;
    if (s->plan_fused) {
        state = "FUSED";
    } else if (!s->plan_gate_on) {
        state = "NOGATE";
    } else if (s->plan_dormant) {
        state = "SLEEP";
    } else {
        state = "AWAKE";
    }

    if (s->plan_max_requests == 0) {
        snprintf(out, ROW_BYTES, "PLAN %s %lu/-", state, (unsigned long)s->plan_requests);
    } else {
        snprintf(out, ROW_BYTES, "PLAN %s %lu/%lu", state, (unsigned long)s->plan_requests,
                 (unsigned long)s->plan_max_requests);
    }
}

/** Three characters: a zero-padded distance, or ERR for a failed sensor. */
static void sonar_field(bool failed, uint16_t cm, char out[4])
{
    if (failed || cm > SONAR_MAX_PRINTABLE_CM) {
        memcpy(out, "ERR", 4);
        return;
    }
    snprintf(out, 4, "%03u", (unsigned)cm);
}

static void row_sonar(const oled_status_t *s, char *out)
{
    if (!s->sonar_known) {
        snprintf(out, ROW_BYTES, "SONAR --");
        return;
    }
    char front[4];
    char rear[4];
    sonar_field(s->front_failed, s->front_cm, front);
    sonar_field(s->rear_failed, s->rear_cm, rear);
    snprintf(out, ROW_BYTES, "%s F %s R %s", s->reflex_active ? "STOP " : "SONAR", front, rear);
}

/** Copy @p scratch into @p row, clipped to the panel width. */
static void clip_row(char row[OLED_STATUS_COLS + 1], const char *scratch)
{
    size_t n = 0;
    while (n < OLED_STATUS_COLS && scratch[n] != '\0') {
        row[n] = scratch[n];
        n++;
    }
    row[n] = '\0';
}

void oled_status_format(const oled_status_t *s, char rows[OLED_STATUS_ROWS][OLED_STATUS_COLS + 1])
{
    static void (*const k_rows[OLED_STATUS_ROWS])(const oled_status_t *, char *) = {
        row_network,
        row_hardware,
        row_planner,
        row_sonar,
    };
    for (int r = 0; r < OLED_STATUS_ROWS; r++) {
        char scratch[SCRATCH_BYTES];
        k_rows[r](s, scratch);
        clip_row(rows[r], scratch);
    }
}
