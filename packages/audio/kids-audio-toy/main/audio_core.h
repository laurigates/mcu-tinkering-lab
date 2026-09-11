/*
 * audio_core — hardware-agnostic logic for the Kids Audio Toy.
 *
 * Pure functions, no ESP-IDF dependencies, so the exact same code runs in
 * three places:
 *   - the firmware (main.c wraps these with ADC reads + LEDC output),
 *   - the host unit tests (test/test_audio_core.c),
 *   - the host simulator (sim/toy_sim.py, via ctypes).
 *
 * This is the logic worth iterating on: how the 0-4095 ADC readings map to
 * pitch / duration / interval, how the 555-timer modulation is smoothed and
 * clamped, and how modulation is folded into the played frequency. Keep it
 * free of platform headers.
 */
#ifndef AUDIO_CORE_H
#define AUDIO_CORE_H

#include <stdint.h>

/* ADC full-scale (12-bit). Readings are clamped to this. */
#define AUDIO_ADC_MAX 4095u

/* Parameter ranges — single source of truth, shared by firmware and sim. */
#define AUDIO_MIN_FREQ_HZ 100.0f
#define AUDIO_MAX_FREQ_HZ 2000.0f
#define AUDIO_MIN_DURATION_MS 50.0f
#define AUDIO_MAX_DURATION_MS 1000.0f
#define AUDIO_MIN_INTERVAL_MS 100.0f
#define AUDIO_MAX_INTERVAL_MS 2000.0f

/* Modulation: max frequency deviation (Hz) and EMA smoothing factor. */
#define AUDIO_MOD_DEPTH_MAX 200.0f
#define AUDIO_MOD_SMOOTHING 0.8f

/* Derived audio parameters for one beep. */
typedef struct {
    float pitch_hz;
    float duration_ms;
    float interval_ms;
} audio_params_t;

/* Linearly map an ADC reading (0..AUDIO_ADC_MAX, clamped) onto [lo, hi]. */
float audio_map_adc_to_range(uint32_t adc_value, float lo, float hi);

/* Map the three control ADC readings onto pitch/duration/interval in *out. */
void audio_params_from_adc(uint32_t pitch_adc, uint32_t duration_adc, uint32_t interval_adc,
                           audio_params_t *out);

/*
 * Advance the smoothed modulation value given the previous value and a fresh
 * 555-timer ADC reading. Applies an exponential moving average, then clamps
 * to [-AUDIO_MOD_DEPTH_MAX, +AUDIO_MOD_DEPTH_MAX]. Returns the new value.
 *
 * Note the input is unipolar (0..AUDIO_ADC_MAX) but the output is bipolar:
 * a reading at mid-scale (~2048) yields ~0 Hz deviation, which is why the
 * 555 is expected to idle near mid-rail.
 */
float audio_update_modulation(float prev_modulation, uint32_t mod_adc);

/* Fold modulation into the base pitch, clamped to [MIN_FREQ, MAX_FREQ]. */
float audio_modulated_freq(float pitch_hz, float modulation);

#endif /* AUDIO_CORE_H */
