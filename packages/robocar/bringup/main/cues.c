/**
 * @file cues.c
 * @brief Buzzer cue vocabulary and the locally generated amplifier test tone.
 */

#include "cues.h"

#include <math.h>
#include <stddef.h>

#include "audio_player.h"
#include "buzzer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pin_config.h"

static const char *TAG = "cues";

/* -------------------------------------------------------------------------- */
/* Buzzer cues                                                                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint16_t hz;
    uint16_t ms;
} note_t;

/* Gaps are part of the pattern: two notes at the same pitch with no gap read as
 * one long note, which collapses FAIL into WARN by ear. */
#define CUE_GAP_MS 45

static void play_notes(const note_t *notes, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        buzzer_play_tone(notes[i].hz, notes[i].ms);
        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(CUE_GAP_MS));
        }
    }
}

void cues_play(cue_t cue)
{
    switch (cue) {
        case CUE_SWEEP_START: {
            static const note_t n[] = {{880, 90}, {1175, 120}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_PASS: {
            static const note_t n[] = {{1568, 60}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_WARN: {
            static const note_t n[] = {{784, 70}, {784, 70}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_SKIP: {
            /* Short, mid-low, single. Most of a build is spent with most things
             * unfitted, so this is the sound the sweep makes most often — it
             * has to be unobtrusive or the whole run reads as failure. */
            static const note_t n[] = {{523, 45}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_FAIL: {
            static const note_t n[] = {{220, 180}, {196, 220}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_MOTOR_ARMED: {
            /* Deliberately unlike every other cue: the next thing that happens
             * is that the wheels turn. A robot on a bench edge drives off it. */
            static const note_t n[] = {{1319, 70}, {1319, 70}, {1319, 140}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_ALL_PASS: {
            static const note_t n[] = {{523, 120}, {659, 120}, {784, 220}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
        case CUE_ANY_FAIL: {
            static const note_t n[] = {{392, 140}, {330, 140}, {262, 300}};
            play_notes(n, sizeof(n) / sizeof(n[0]));
            break;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Amplifier test tone                                                          */
/* -------------------------------------------------------------------------- */

/** Samples generated per audio_player_write(). 256 int16 = 512 bytes, which
 *  keeps the scratch buffer off a deep stack while still being far larger than
 *  the 3-byte granularity that once thrashed the ring (see robocar-unified's
 *  TTS_PCM_BATCH_BYTES). */
#define TONE_CHUNK_SAMPLES 256

/** Peak amplitude as a fraction of full scale. Held well below 1.0 on purpose:
 *  the point of this tone is to hear DISTORTION, so it must not be capable of
 *  clipping on its own. audio_player applies a further AUDIO_VOLUME_PCT. */
#define TONE_AMPLITUDE 0.5f

#define TONE_RING_TIMEOUT_MS 2000

#define TWO_PI 6.28318530717958647692f

/**
 * Append @p ms of a tone sweeping linearly from @p from_hz to @p to_hz.
 *
 * @p phase is carried in and out rather than reset per call. Restarting the
 * sine at zero between segments steps the waveform discontinuously, which the
 * amplifier reproduces as a click — and a click is exactly the artefact this
 * sequence exists to let someone listen for.
 */
static esp_err_t push_sweep(float from_hz, float to_hz, uint32_t ms, float *phase)
{
    const size_t total = (size_t)((uint64_t)AUDIO_SAMPLE_RATE_HZ * ms / 1000u);
    int16_t chunk[TONE_CHUNK_SAMPLES];
    size_t done = 0;

    while (done < total) {
        const size_t n = (total - done < TONE_CHUNK_SAMPLES) ? (total - done) : TONE_CHUNK_SAMPLES;

        for (size_t i = 0; i < n; i++) {
            const float t = (float)(done + i) / (float)total;
            const float hz = from_hz + (to_hz - from_hz) * t;
            chunk[i] = (int16_t)(TONE_AMPLITUDE * 32767.0f * sinf(*phase));
            *phase += TWO_PI * hz / (float)AUDIO_SAMPLE_RATE_HZ;
            if (*phase >= TWO_PI) {
                *phase -= TWO_PI;
            }
        }

        const esp_err_t err =
            audio_player_write((const uint8_t *)chunk, n * sizeof(chunk[0]), TONE_RING_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ring write failed: %s", esp_err_to_name(err));
            return err;
        }
        done += n;
    }
    return ESP_OK;
}

/** Silence of @p ms, so the segments are separable by ear. */
static esp_err_t push_silence(uint32_t ms)
{
    static const int16_t quiet[TONE_CHUNK_SAMPLES] = {0};
    const size_t total = (size_t)((uint64_t)AUDIO_SAMPLE_RATE_HZ * ms / 1000u);
    size_t done = 0;

    while (done < total) {
        const size_t n = (total - done < TONE_CHUNK_SAMPLES) ? (total - done) : TONE_CHUNK_SAMPLES;
        const esp_err_t err =
            audio_player_write((const uint8_t *)quiet, n * sizeof(quiet[0]), TONE_RING_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
        done += n;
    }
    return ESP_OK;
}

esp_err_t cues_amp_tone_sequence(void)
{
    if (!audio_player_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    float phase = 0.0f;
    esp_err_t err = ESP_OK;

    /* begin/end bracket the whole sequence, not each segment: audio_player's
     * preroll gate is a property of the ring, and re-opening it per segment
     * would let the second tone start with nothing banked. */
    audio_player_begin_utterance();

    if (err == ESP_OK) {
        err = push_sweep(440.0f, 440.0f, 400, &phase); /* A4, steady */
    }
    if (err == ESP_OK) {
        err = push_silence(120);
    }
    if (err == ESP_OK) {
        err = push_sweep(880.0f, 880.0f, 400, &phase); /* A5: an octave up */
    }
    if (err == ESP_OK) {
        err = push_silence(120);
    }
    if (err == ESP_OK) {
        /* The sweep is the informative part. A resonance, a loose ground or a
         * sagging rail shows up as a band that buzzes while the neighbouring
         * frequencies stay clean, which a single fixed tone cannot reveal. */
        err = push_sweep(200.0f, 2000.0f, 1200, &phase);
    }

    audio_player_end_utterance();

    /* Wait out playback so the sweep's next check does not run while the
     * amplifier is still sounding — the microphone check in particular would
     * otherwise measure this tone rather than the room. */
    for (int i = 0; i < 100 && audio_player_is_active(); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return err;
}
