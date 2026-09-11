/*
 * Host unit tests for audio_core — the hardware-agnostic logic shared by the
 * firmware and the simulator. Plain C asserts, no framework. Build and run:
 *
 *     just test          (from the project dir)
 * or  cc -I main -o /tmp/t test/test_audio_core.c main/audio_core.c -lm && /tmp/t
 *
 * These pin down the bound, clamp, and modulation behaviour so a future ADC
 * driver migration (or a tweak to audio_core) can't silently change it.
 */
#include <math.h>
#include <stdio.h>

#include "audio_core.h"

static int failures = 0;

#define CHECK_NEAR(actual, expected, tol, msg)                                       \
    do {                                                                             \
        float _a = (actual), _e = (expected);                                        \
        if (fabsf(_a - _e) > (tol)) {                                                \
            printf("  FAIL: %s (got %.3f, expected %.3f +/- %.3f)\n", (msg), _a, _e, \
                   (float)(tol));                                                    \
            failures++;                                                              \
        }                                                                            \
    } while (0)

static void test_map_bounds(void)
{
    printf("map_adc_to_range bounds:\n");
    CHECK_NEAR(audio_map_adc_to_range(0, 100.0f, 2000.0f), 100.0f, 0.01f, "adc 0 -> lo");
    CHECK_NEAR(audio_map_adc_to_range(4095, 100.0f, 2000.0f), 2000.0f, 0.01f, "adc max -> hi");
    CHECK_NEAR(audio_map_adc_to_range(2048, 100.0f, 2000.0f), 1050.5f, 1.0f, "adc mid -> midpoint");
    // Out-of-range readings clamp to the high end rather than overshooting.
    CHECK_NEAR(audio_map_adc_to_range(9000, 100.0f, 2000.0f), 2000.0f, 0.01f, "adc > max clamps");
}

static void test_params_from_adc(void)
{
    printf("audio_params_from_adc:\n");
    audio_params_t p;
    audio_params_from_adc(0, 0, 0, &p);
    CHECK_NEAR(p.pitch_hz, 100.0f, 0.01f, "pitch min");
    CHECK_NEAR(p.duration_ms, 50.0f, 0.01f, "duration min");
    CHECK_NEAR(p.interval_ms, 100.0f, 0.01f, "interval min");

    audio_params_from_adc(4095, 4095, 4095, &p);
    CHECK_NEAR(p.pitch_hz, 2000.0f, 0.01f, "pitch max");
    CHECK_NEAR(p.duration_ms, 1000.0f, 0.01f, "duration max");
    CHECK_NEAR(p.interval_ms, 2000.0f, 0.01f, "interval max");
}

static void test_modulation_bipolar_mapping(void)
{
    printf("modulation bipolar mapping:\n");
    // One un-smoothed step from 0: result is 0.2 * raw (since smoothing=0.8).
    // raw at adc 0 is -200, at adc 4095 is +200, at mid is ~0.
    CHECK_NEAR(audio_update_modulation(0.0f, 0), 0.2f * -200.0f, 0.5f, "adc 0 -> negative");
    CHECK_NEAR(audio_update_modulation(0.0f, 4095), 0.2f * 200.0f, 0.5f, "adc max -> positive");
    CHECK_NEAR(audio_update_modulation(0.0f, 2048), 0.0f, 0.5f, "adc mid -> ~zero");
}

static void test_modulation_converges_and_clamps(void)
{
    printf("modulation convergence + clamp:\n");
    // Feeding a constant max input repeatedly converges toward +MOD_DEPTH_MAX.
    float m = 0.0f;
    for (int i = 0; i < 200; i++) {
        m = audio_update_modulation(m, 4095);
    }
    CHECK_NEAR(m, AUDIO_MOD_DEPTH_MAX, 1.0f, "converges to +max");

    // The smoothed value never exceeds the clamp even from an extreme prior.
    float clamped = audio_update_modulation(1000.0f, 4095);
    if (clamped > AUDIO_MOD_DEPTH_MAX) {
        printf("  FAIL: clamp high (got %.3f)\n", clamped);
        failures++;
    }
    clamped = audio_update_modulation(-1000.0f, 0);
    if (clamped < -AUDIO_MOD_DEPTH_MAX) {
        printf("  FAIL: clamp low (got %.3f)\n", clamped);
        failures++;
    }
}

static void test_modulated_freq_clamps(void)
{
    printf("modulated_freq clamp:\n");
    CHECK_NEAR(audio_modulated_freq(440.0f, 60.0f), 500.0f, 0.01f, "in-range adds");
    CHECK_NEAR(audio_modulated_freq(2000.0f, 200.0f), 2000.0f, 0.01f, "clamps to max");
    CHECK_NEAR(audio_modulated_freq(100.0f, -200.0f), 100.0f, 0.01f, "clamps to min");
}

int main(void)
{
    printf("audio_core host tests\n=====================\n");
    test_map_bounds();
    test_params_from_adc();
    test_modulation_bipolar_mapping();
    test_modulation_converges_and_clamps();
    test_modulated_freq_clamps();

    if (failures == 0) {
        printf("\nAll audio_core tests passed.\n");
        return 0;
    }
    printf("\n%d check(s) FAILED.\n", failures);
    return 1;
}
