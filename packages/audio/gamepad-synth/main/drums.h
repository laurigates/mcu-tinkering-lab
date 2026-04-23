/**
 * @file drums.h
 * @brief Background drum engine for the gamepad-synth.
 *
 * Provides a 16-step sequencer with 4 patterns and three voices (kick, snare,
 * hi-hat). The sequencer runs on an esp_timer and triggers voices whose phase
 * and envelope state are advanced in drums_render_block(), which mixes (adds
 * with saturation) the summed voices in-place into a stereo int16 buffer.
 *
 * Integration contract:
 *   - drums_init()                    once, after init_sine_table()
 *   - drums_set_pattern(1..4)         start/switch pattern
 *   - drums_set_pattern(0)            stop the sequencer
 *   - drums_set_tempo(bpm)            clamped to [60, 200]
 *   - drums_set_volume(0.0 .. 1.0)    post-sum master scalar
 *   - drums_render_block(buf, N)      call AFTER the synth has filled `buf`;
 *                                     adds drum content with saturation
 */

#ifndef DRUMS_H
#define DRUMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void drums_init(void);
void drums_set_pattern(int idx);
void drums_set_tempo(int bpm);
void drums_set_volume(float v);
void drums_render_block(int16_t *stereo_buf, int block_size);

#ifdef __cplusplus
}
#endif

#endif /* DRUMS_H */
