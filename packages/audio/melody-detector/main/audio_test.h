#ifndef AUDIO_TEST_H
#define AUDIO_TEST_H

#include <stdint.h>

#include "esp_err.h"

esp_err_t audio_test_init(void);

// Plays a sine wave at `freq_hz` for `duration_ms` (blocking). Used for
// hardware bring-up — the FR-4.x synth replaces this in Track E.
esp_err_t audio_test_tone(uint32_t freq_hz, uint32_t duration_ms);

#endif  // AUDIO_TEST_H
