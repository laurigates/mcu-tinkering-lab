// Host unit tests for pack_sense.c: the divider scale, the raw window to
// volts, the absent / clipped judgement, and the warning and cutoff monitor
// (no Pico SDK). Raw counts are derived here from the circuit, not from the
// code's own scale: counts = V_pack / ratio / 3.3 V * 4096.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "pack_sense.h"

static bool near(float a, float b, float tol)
{
    return fabsf(a - b) <= tol;
}

// Counts a pack of `v` volts gives through a 100k / r_low divider.
static uint16_t counts_for(float v, float r_low)
{
    const float pin = v * r_low / (100.0f + r_low);
    return (uint16_t)lroundf(pin / 3.3f * 4096.0f);
}

static pack_reading_t read_flat(uint16_t c, float r_low)
{
    uint16_t raw[PACK_SAMPLES];
    for (unsigned i = 0; i < PACK_SAMPLES; i++) {
        raw[i] = c;
    }
    return pack_reading_from_counts(raw, PACK_SAMPLES, r_low);
}

static pack_reading_t volts(float v)
{
    pack_reading_t r = {.present = true, .clipped = false, .volts = v};
    return r;
}

static void test_scale(void)
{
    assert(near(pack_scale(27.0f), 127.0f / 27.0f, 1e-5f));
    assert(near(pack_scale(22.0f), 122.0f / 22.0f, 1e-5f));
}

static void test_validators(void)
{
    assert(pack_r_low_valid(27.0f) && pack_r_low_valid(22.0f));
    assert(!pack_r_low_valid(0.0f) && !pack_r_low_valid(-5.0f));
    assert(!pack_r_low_valid(NAN) && !pack_r_low_valid(INFINITY));
    assert(!pack_r_low_valid(1001.0f));
    assert(pack_cells_valid(3.0f) && pack_cells_valid(4.0f));
    assert(!pack_cells_valid(3.5f) && !pack_cells_valid(1.0f) && !pack_cells_valid(7.0f));
    assert(!pack_cells_valid(NAN));
}

static void test_reading_scales_by_board(void)
{
    // The same 12.6 V pack reads 2.68 V on D0 of a delivered board and 2.27 V
    // on rev3; each board's own setting recovers 12.6 V, the other's does not.
    pack_reading_t a = read_flat(counts_for(12.6f, 27.0f), 27.0f);
    assert(a.present && !a.clipped && near(a.volts, 12.6f, 0.05f));
    pack_reading_t b = read_flat(counts_for(12.6f, 22.0f), 22.0f);
    assert(b.present && !b.clipped && near(b.volts, 12.6f, 0.05f));
    pack_reading_t wrong = read_flat(counts_for(12.6f, 27.0f), 22.0f);
    assert(wrong.volts > 14.0f);  // ~18 % high, the error the setting exists to prevent
}

static void test_absent_when_floating(void)
{
    // Pin held at ground, or just above it.
    assert(!read_flat(0, 27.0f).present);
    assert(read_flat(0, 27.0f).volts == 0.0f);
    assert(!read_flat(300, 27.0f).present);  // 0.24 V
    // A floating pin wandering: spread over PACK_NOISE_PIN_V (62 counts).
    const uint16_t wander[PACK_SAMPLES] = {2000, 2100, 1950, 2200, 2050, 2150, 1900, 2250};
    assert(!pack_reading_from_counts(wander, PACK_SAMPLES, 27.0f).present);
    // No samples is not a reading.
    assert(!pack_reading_from_counts(wander, 0, 27.0f).present);
    // A connected pack with a few counts of noise stays present.
    const uint16_t quiet[PACK_SAMPLES] = {3325, 3327, 3324, 3326, 3325, 3328, 3323, 3326};
    assert(pack_reading_from_counts(quiet, PACK_SAMPLES, 27.0f).present);
}

static void test_clipped(void)
{
    // 16.8 V (4S full) on a 27k board: 3.57 V, past the 3.3 V reference.
    pack_reading_t r = read_flat(4095, 27.0f);
    assert(r.present && r.clipped);
    // 12.6 V is not clipped.
    assert(!read_flat(counts_for(12.6f, 27.0f), 27.0f).clipped);
}

static void test_centivolts(void)
{
    assert(pack_reading_centivolts(&(pack_reading_t){.present = true, .volts = 12.6f}) == 1260u);
    assert(pack_reading_centivolts(&(pack_reading_t){.present = true, .volts = 10.504f}) == 1050u);
    assert(pack_reading_centivolts(&(pack_reading_t){.present = false}) == 0u);
    assert(pack_reading_centivolts(&(pack_reading_t){.present = true, .volts = 9999.0f}) == 65535u);
}

static void test_thresholds(void)
{
    assert(near(pack_warn_volts(3.0f), 10.5f, 1e-4f));
    assert(near(pack_warn_volts(4.0f), 14.0f, 1e-4f));
    assert(near(pack_fault_volts(3.0f), 9.0f, 1e-4f));
}

// Feed `v` for `seconds` in 100 ms steps.
static void feed(pack_monitor_t *m, float v, float cells, float seconds)
{
    pack_reading_t r = volts(v);
    for (int i = 0; i < (int)lroundf(seconds * 10.0f); i++) {
        pack_monitor_update(m, &r, cells, 0.1f);
    }
}

static void test_warning_and_hysteresis(void)
{
    pack_monitor_t m;
    pack_monitor_init(&m);
    feed(&m, 12.0f, 3.0f, 0.1f);
    assert(m.level == PACK_OK && !m.fault);
    feed(&m, 10.5f, 3.0f, 0.1f);  // exactly at the threshold: not below it
    assert(m.level == PACK_OK);
    feed(&m, 10.4f, 3.0f, 0.1f);
    assert(m.level == PACK_WARN && !m.fault);
    feed(&m, 10.6f, 3.0f, 0.1f);  // inside the hysteresis band (clears at 10.65 V)
    assert(m.level == PACK_WARN);
    feed(&m, 10.7f, 3.0f, 0.1f);
    assert(m.level == PACK_OK);
}

static void test_cutoff_needs_duration(void)
{
    pack_monitor_t m;
    pack_monitor_init(&m);
    feed(&m, 8.9f, 3.0f, 0.9f);
    assert(!m.fault && m.level == PACK_WARN);  // below the cutoff, not for long enough
    feed(&m, 8.9f, 3.0f, 0.1f);
    assert(m.fault && m.level == PACK_LOW);

    // A sag that recovers resets the clock.
    pack_monitor_init(&m);
    feed(&m, 8.9f, 3.0f, 0.9f);
    feed(&m, 9.5f, 3.0f, 0.1f);
    feed(&m, 8.9f, 3.0f, 0.9f);
    assert(!m.fault);
}

static void test_cutoff_latch_and_clear(void)
{
    pack_monitor_t m;
    pack_monitor_init(&m);
    feed(&m, 8.5f, 3.0f, 1.0f);
    assert(m.fault);
    // The unloaded pack rebounds above the cutoff but not the clear level
    // (9.0 + 0.9 = 9.9 V): still latched.
    feed(&m, 9.5f, 3.0f, 2.0f);
    assert(m.fault && m.level == PACK_LOW);
    feed(&m, 9.95f, 3.0f, 0.1f);
    assert(!m.fault);
    assert(m.level == PACK_WARN);  // clear of the latch, still under the warning
}

static void test_cells_scale_the_thresholds(void)
{
    // 12.5 V is a healthy 3S pack and a flat 4S one.
    pack_monitor_t m3;
    pack_monitor_t m4;
    pack_monitor_init(&m3);
    pack_monitor_init(&m4);
    feed(&m3, 12.5f, 3.0f, 2.0f);
    feed(&m4, 12.5f, 4.0f, 2.0f);
    assert(m3.level == PACK_OK);
    assert(m4.level == PACK_WARN && !m4.fault);
    feed(&m4, 11.9f, 4.0f, 1.0f);
    assert(m4.fault);
}

static void test_absent_clears_the_monitor(void)
{
    pack_monitor_t m;
    pack_monitor_init(&m);
    feed(&m, 8.5f, 3.0f, 1.0f);
    assert(m.fault);
    pack_reading_t none = {.present = false};
    pack_monitor_update(&m, &none, 3.0f, 0.1f);
    assert(m.level == PACK_ABSENT && !m.fault && m.below_s == 0.0f);
    // An absent pin never starts the clock.
    pack_monitor_init(&m);
    for (int i = 0; i < 50; i++) {
        pack_monitor_update(&m, &none, 3.0f, 0.1f);
    }
    assert(!m.fault);
}

int main(void)
{
    test_scale();
    test_validators();
    test_reading_scales_by_board();
    test_absent_when_floating();
    test_clipped();
    test_centivolts();
    test_thresholds();
    test_warning_and_hysteresis();
    test_cutoff_needs_duration();
    test_cutoff_latch_and_clear();
    test_cells_scale_the_thresholds();
    test_absent_clears_the_monitor();
    printf("pack_sense: all tests passed\n");
    return 0;
}
