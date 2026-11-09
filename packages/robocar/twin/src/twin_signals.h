/**
 * @file twin_signals.h
 * @brief Deterministic synthetic sensor data for the behaviour twin.
 *
 * Phase 1 has no camera or microphone fixtures, so the twin makes the raw data
 * the firmware's own reducers consume: a luma thumbnail for
 * scene_fingerprint_from_luma() and PCM frames for ambient_fingerprint_from_pcm().
 * Everything is a pure function of its inputs, so a run is reproducible bit for
 * bit from the scenario and its seed (PRD-017 FR-2).
 */

#ifndef TWIN_SIGNALS_H
#define TWIN_SIGNALS_H

#include <stddef.h>
#include <stdint.h>

/** frame_stats_log() decodes a 320x240 frame at JPEG_IMAGE_SCALE_1_8 (frame_dump.c),
 *  and scene_change.h's block grid divides that 40x30 thumbnail exactly. */
#define TWIN_THUMB_W 40
#define TWIN_THUMB_H 30

/** RMS of the background room tone, in LSB (about 40 dB on ambient_audio's scale). */
#define TWIN_ROOM_RMS_LSB 100.0

/**
 * @brief A luma thumbnail that depends only on the scene name.
 *
 * Each 5x5 block gets one luma value drawn from a PRNG seeded by a hash of the
 * name, so the same name always yields the same fingerprint (distance 0) and two
 * different names land tens of luma units apart, far past both scene thresholds.
 */
void twin_scene_luma(const char *scene, uint8_t luma[TWIN_THUMB_W * TWIN_THUMB_H]);

/**
 * @brief One frame of steady room tone (white noise at TWIN_ROOM_RMS_LSB).
 *
 * The same frame is used every time, so the room is perfectly stationary: its
 * level sits on the adaptive floor and its shape never moves, and only scripted
 * sounds produce audio evidence.
 */
void twin_pcm_room(uint32_t seed, int16_t *pcm, size_t n);

/**
 * @brief One frame of a scripted sound: room tone plus a burst @p peak_db above it.
 *
 * @param frame_index  Frame number within the burst; varies the noise per frame.
 */
void twin_pcm_burst(uint32_t seed, uint32_t frame_index, int peak_db, int16_t *pcm, size_t n);

#endif /* TWIN_SIGNALS_H */
