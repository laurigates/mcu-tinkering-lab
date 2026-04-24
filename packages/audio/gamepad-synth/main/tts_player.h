/**
 * @file tts_player.h
 * @brief One-shot PCM clip player for voicing-announcement TTS clips.
 *
 * Source material is raw 16-bit LE PCM at 24 kHz mono (produced offline by
 * tools/tts/generate.py via the Gemini TTS API, embedded in the firmware via
 * EMBED_FILES). Playback upsamples to 44.1 kHz with linear interpolation
 * and overlay-mixes into the shared stereo buffer — same pattern as
 * drums_render_block().
 *
 * Contract:
 *   - tts_player_start(samples, num_samples) — non-blocking; stores a pointer
 *     and resets the fractional read position. Single-slot: a later call
 *     pre-empts any in-flight clip.
 *   - tts_player_render_block(stereo_buf, block_size) — called from the audio
 *     render task on every block. When a clip is active, saturating-adds into
 *     both L/R of each frame. When exhausted, becomes a no-op.
 *   - tts_player_is_playing() — true while a clip is still streaming.
 *
 * Samples must live at least until the clip finishes — EMBED_FILES data has
 * program lifetime, so embedded-clip callers are trivially safe.
 */

#ifndef TTS_PLAYER_H
#define TTS_PLAYER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tts_player_start(const int16_t *samples, int num_samples);
void tts_player_render_block(int16_t *stereo_buf, int block_size);
bool tts_player_is_playing(void);

#ifdef __cplusplus
}
#endif

#endif /* TTS_PLAYER_H */
