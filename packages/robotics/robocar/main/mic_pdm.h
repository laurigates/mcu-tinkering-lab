/**
 * @file mic_pdm.h
 * @brief The onboard PDM microphone, as a thin I2S0 RX channel and nothing else.
 *
 * What this file is
 * -----------------
 * The camera.c of the audio input path: it owns a piece of hardware, hands out
 * buffers of samples, and performs no analysis whatsoever.  Deciding what a
 * frame of PCM *means* — is the room louder, has its shape changed, is that a
 * human talking — belongs to the modules above, which are pure and host-tested.
 * Nothing here computes a level, a threshold or a verdict.
 *
 * Why the allocation is subtle
 * ----------------------------
 * On the ESP32-S3, PDM RX exists ONLY on I2S0 (esp_driver_i2s/i2s_pdm.c rejects
 * any other controller outright), and audio_player.c has already taken I2S0 for
 * the MAX98357A at 24 kHz.  The mic and the amplifier therefore have no choice
 * but to share controller 0.  That is fine — ESP32-S3 is SOC_I2S_HW_VERSION_2,
 * so TX and RX have independent clock dividers and 24 kHz out / 16 kHz in
 * coexist happily — but ONLY as long as the driver does not decide the
 * controller is in full-duplex mode.  The full reasoning, and the one-line
 * "simplification" that would break it, lives at the allocation site in
 * mic_pdm.c.  Read it before touching channel setup.
 *
 * Single reader, enforced by a mutex
 * ----------------------------------
 * Two callers want this microphone and they must never interleave on one
 * channel:
 *
 *   - the ambient listener, which takes short frames continuously to watch the
 *     room, and
 *   - a voice turn, which needs several uninterrupted seconds of speech.
 *
 * Both go through mic_pdm_lock() / mic_pdm_unlock().  The listener takes the
 * lock per frame; a voice turn holds it for the whole recording window.  That
 * ordering is deliberate on both sides: a recording is never punctured by the
 * listener's reads, and — just as importantly — the listener's noise floor does
 * not learn from a conversation it was excluded from.
 *
 * Staleness is the thing to watch
 * -------------------------------
 * I2S RX is a descriptor FIFO that keeps filling whether or not anyone is
 * reading.  Whenever the reader is away — computing a fingerprint, dumping a
 * frame over the console, or descheduled — the DMA banks samples, and the next
 * mic_pdm_read() returns the OLDEST of them, not the newest.  So a returned
 * frame can describe a moment up to MIC_PDM_BACKLOG_MS in the past.
 *
 * That matters for exactly one thing: any caller gating capture on
 * audio_player_is_active() is testing NOW against samples from THEN, and a
 * frame containing the robot's own voice can arrive after the amplifier has
 * gone quiet.  Callers must therefore call mic_pdm_flush() when playback ends,
 * before trusting the next frame.  MIC_PDM_BACKLOG_MS is kept deliberately
 * small for the same reason: it is an upper bound on how stale an accepted
 * frame can be, and any hangover a caller applies has to exceed it.
 *
 * There is no hardware help available
 * -----------------------------------
 * SOC_I2S_SUPPORTS_PDM_RX_HP_FILTER is not defined for the ESP32-S3, so the
 * slot config has no high-pass filter, no cut-off, and no `amplify_num`.  DC
 * removal and any gain stage are software problems for the layers above; there
 * is no register to reach for if the amplitude comes back short.
 */

#ifndef MIC_PDM_H
#define MIC_PDM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "pin_config.h"  // MIC_PDM_CLK_PIN / MIC_PDM_DATA_PIN / MIC_SAMPLE_RATE_HZ

#ifdef __cplusplus
extern "C" {
#endif

/** Upper bound on how far behind real time a returned frame can be, in
 *  milliseconds — the RX DMA depth expressed as time at MIC_SAMPLE_RATE_HZ.
 *  Sized small on purpose; see the staleness note above. */
#define MIC_PDM_BACKLOG_MS 128

/**
 * @brief Allocate and start the I2S0 PDM RX channel.
 *
 * Safe to call after audio_player_init(); the order of the two does not matter,
 * because each allocates its own direction on the shared controller.
 *
 * Non-fatal by convention in this firmware: a failure here must leave the robot
 * driving, talking and reachable on the console.  Callers should log and carry
 * on rather than ESP_ERROR_CHECK.
 */
esp_err_t mic_pdm_init(void);

/** True once the RX channel exists and is enabled. */
bool mic_pdm_is_ready(void);

/**
 * @brief Take exclusive use of the microphone.
 *
 * @param timeout_ms  Bound on the wait.  A listener should pass something short
 *                    and simply skip its frame on failure — a voice turn is in
 *                    progress and its samples are the ones that matter.
 * @return ESP_OK, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_STATE before init.
 */
esp_err_t mic_pdm_lock(uint32_t timeout_ms);

/** Release the microphone.  Only the holder may call this. */
void mic_pdm_unlock(void);

/**
 * @brief Read up to @p max_samples 16-bit mono samples.
 *
 * The caller must hold the lock.  Returns short reads: @p out_samples is the
 * count actually delivered and may be less than requested even on ESP_OK.
 *
 * @param timeout_ms  A real millisecond bound.  Never portMAX_DELAY — the I2S
 *                    driver converts this through pdMS_TO_TICKS internally, so
 *                    the sentinel would silently become ~72 minutes rather than
 *                    "forever", and a wedged DMA would look like a hung task
 *                    instead of a logged error.
 */
esp_err_t mic_pdm_read(int16_t *dst, size_t max_samples, size_t *out_samples, uint32_t timeout_ms);

/**
 * @brief Discard whatever the RX DMA has banked, so the next read is fresh.
 *
 * Call this after any period during which nobody was reading — most importantly
 * the moment audio_player_is_active() goes false.  Without it the first frames
 * after playback are the robot's own voice, delivered late, and every gate
 * downstream is testing the present against the past.
 *
 * The caller must hold the lock.
 */
void mic_pdm_flush(void);

#ifdef __cplusplus
}
#endif

#endif  // MIC_PDM_H
