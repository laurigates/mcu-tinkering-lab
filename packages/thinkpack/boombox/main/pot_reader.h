/**
 * @file pot_reader.h
 * @brief ADC oneshot potentiometer reader with exponential smoothing.
 *
 * Two channels on ADC1:
 *   POT_TEMPO  — GPIO 1 (ADC1_CH0) — maps to BPM range [60, 200]
 *   POT_PITCH  — GPIO 2 (ADC1_CH1) — maps to semitone shift [-12, +12]
 *
 * Uses ESP-IDF v5.4 oneshot ADC API (not the deprecated legacy API).
 * Exponential smoothing alpha = 0.2 reduces ADC jitter.
 */

#ifndef POT_READER_H
#define POT_READER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Potentiometer identifiers. */
typedef enum {
    POT_TEMPO = 0, /**< GPIO 1, ADC1_CH0 — tempo control */
    POT_PITCH = 1, /**< GPIO 2, ADC1_CH1 — pitch shift   */
    POT_COUNT,
} pot_id_t;

/**
 * @brief Initialise ADC1 oneshot unit and configure both pot channels.
 *
 * @return ESP_OK on success.
 */
int pot_reader_init(void);

/**
 * @brief Sample both pots and apply exponential smoothing.
 *
 * Call at a regular interval (e.g., every 50 ms in the tone task).
 *
 * @param now_ms  Current time in milliseconds (used for rate limiting).
 */
void pot_reader_tick(uint32_t now_ms);

/**
 * @brief Return the latest smoothed ADC raw value for a pot.
 *
 * @param pot  Which potentiometer.
 * @return     Smoothed value in range [0, 4095].
 */
uint16_t pot_reader_get_raw(pot_id_t pot);

/**
 * @brief Map tempo pot raw value to BPM.
 *
 * @param raw  Raw ADC value [0, 4095].
 * @return     BPM in [60, 200].
 */
uint16_t pot_tempo_to_bpm(uint16_t raw);

/**
 * @brief Map pitch pot raw value to semitone shift.
 *
 * @param raw  Raw ADC value [0, 4095].
 * @return     Semitone shift in [-12, +12].
 */
int8_t pot_pitch_to_semitones(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* POT_READER_H */
