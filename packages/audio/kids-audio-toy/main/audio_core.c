/*
 * audio_core — implementation. See audio_core.h. Pure C, no platform headers.
 */
#include "audio_core.h"

float audio_map_adc_to_range(uint32_t adc_value, float lo, float hi)
{
    if (adc_value > AUDIO_ADC_MAX) {
        adc_value = AUDIO_ADC_MAX;
    }
    return lo + (hi - lo) * ((float)adc_value / (float)AUDIO_ADC_MAX);
}

void audio_params_from_adc(uint32_t pitch_adc, uint32_t duration_adc, uint32_t interval_adc,
                           audio_params_t *out)
{
    out->pitch_hz = audio_map_adc_to_range(pitch_adc, AUDIO_MIN_FREQ_HZ, AUDIO_MAX_FREQ_HZ);
    out->duration_ms =
        audio_map_adc_to_range(duration_adc, AUDIO_MIN_DURATION_MS, AUDIO_MAX_DURATION_MS);
    out->interval_ms =
        audio_map_adc_to_range(interval_adc, AUDIO_MIN_INTERVAL_MS, AUDIO_MAX_INTERVAL_MS);
}

float audio_update_modulation(float prev_modulation, uint32_t mod_adc)
{
    float raw = audio_map_adc_to_range(mod_adc, -AUDIO_MOD_DEPTH_MAX, AUDIO_MOD_DEPTH_MAX);

    float smoothed = (AUDIO_MOD_SMOOTHING * prev_modulation) + ((1.0f - AUDIO_MOD_SMOOTHING) * raw);

    if (smoothed < -AUDIO_MOD_DEPTH_MAX) {
        smoothed = -AUDIO_MOD_DEPTH_MAX;
    }
    if (smoothed > AUDIO_MOD_DEPTH_MAX) {
        smoothed = AUDIO_MOD_DEPTH_MAX;
    }
    return smoothed;
}

float audio_modulated_freq(float pitch_hz, float modulation)
{
    float freq = pitch_hz + modulation;

    if (freq < AUDIO_MIN_FREQ_HZ) {
        freq = AUDIO_MIN_FREQ_HZ;
    }
    if (freq > AUDIO_MAX_FREQ_HZ) {
        freq = AUDIO_MAX_FREQ_HZ;
    }
    return freq;
}
