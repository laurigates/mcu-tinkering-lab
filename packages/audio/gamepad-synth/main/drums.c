/**
 * @file drums.c
 * @brief Background drum engine — 4 patterns x 16 steps, 3 voices.
 *
 * Sequencer
 * ---------
 * An esp_timer fires at a 16th-note grid. Each tick advances a step counter
 * (0..15) and inspects the active pattern's kick/snare/hat bitmasks. Voices
 * whose bit is set on the current step get a retrigger (phase/env reset +
 * samples_remaining assignment).
 *
 * Voice synthesis
 * ---------------
 * Each voice holds a samples_remaining counter plus internal phase/envelope
 * state. drums_render_block() iterates block_size samples, accumulates each
 * voice's contribution (pre-volume) into a float mono accumulator, applies
 * the volume scalar, clips to int16, then MIXES (saturating add) the drum
 * sample into both L/R of stereo_buf[i*2], stereo_buf[i*2+1].
 *
 * This file keeps its own 256-entry sine LUT so it does not depend on
 * symbols in main.c.
 */

#include "drums.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

static const char *TAG = "drums";

/* ── Audio constants (must match main.c I2S setup) ───────── */

#define DRUMS_SAMPLE_RATE 44100

/* ── Sine LUT (local to drums.c) ──────────────────────────── */

#define DRUMS_SINE_TABLE_SIZE 256
static int16_t s_sine_lut[DRUMS_SINE_TABLE_SIZE];

static void drums_init_sine_lut(void)
{
    for (int i = 0; i < DRUMS_SINE_TABLE_SIZE; i++) {
        s_sine_lut[i] = (int16_t)(32000.0f * sinf(2.0f * 3.14159265f * (float)i /
                                                  (float)DRUMS_SINE_TABLE_SIZE));
    }
}

static inline int16_t clip_i16(float x)
{
    if (x > 32767.0f)
        return 32767;
    if (x < -32768.0f)
        return -32768;
    return (int16_t)x;
}

/* ── Patterns (bit N = voice fires on step N) ────────────── */

typedef struct {
    uint16_t kick;
    uint16_t snare;
    uint16_t hat;
} drum_pattern_t;

/* Step bit helpers for readability in pattern literals. */
#define S(n) ((uint16_t)(1u << (n)))

/* Pattern 1 — 4/4 rock:
 *   kick  on 0, 8
 *   snare on 4, 12
 *   hat   on every step (0..15)
 * Pattern 2 — half-time:
 *   kick  on 0
 *   snare on 8
 *   hat   on odd steps (1,3,5,7,9,11,13,15)
 * Pattern 3 — fast hat:
 *   kick  on 0, 8
 *   snare on 4, 12
 *   hat   on all 16
 * Pattern 4 — sparse:
 *   kick  on 0
 *   snare on 8
 *   hat   on 0, 4, 8, 12
 */
static const drum_pattern_t s_patterns[4] = {
    /* 1 — 4/4 rock */
    {
        .kick = S(0) | S(8),
        .snare = S(4) | S(12),
        .hat = 0xFFFFu,
    },
    /* 2 — half-time */
    {
        .kick = S(0),
        .snare = S(8),
        .hat = S(1) | S(3) | S(5) | S(7) | S(9) | S(11) | S(13) | S(15),
    },
    /* 3 — fast hat */
    {
        .kick = S(0) | S(8),
        .snare = S(4) | S(12),
        .hat = 0xFFFFu,
    },
    /* 4 — sparse */
    {
        .kick = S(0),
        .snare = S(8),
        .hat = S(0) | S(4) | S(8) | S(12),
    },
};

#undef S

/* ── Voice state ──────────────────────────────────────────── */

/* Kick: sine LUT, pitch env 80 Hz → 40 Hz linear over 120 ms,
 *       amp env AD: 5 ms linear attack, 115 ms linear decay. */
typedef struct {
    int samples_remaining;
    int samples_total;  /* for envelope normalization (120 ms) */
    int attack_samples; /* 5 ms */
    float phase;        /* [0, 1) */
    float freq_start;   /* 80 Hz */
    float freq_end;     /* 40 Hz */
    float peak_amp;     /* 12000 */
} kick_voice_t;

/* Snare: noise + 200 Hz triangle summed 50/50, one-pole HPF ~800 Hz,
 *        linear decay 80 ms, peak amp 10000. */
typedef struct {
    int samples_remaining;
    int samples_total;
    float tri_phase;
    float tri_freq;   /* 200 Hz */
    float hpf_x_prev; /* x[n-1] */
    float hpf_y_prev; /* y[n-1] */
    float hpf_coef;   /* ≈ 0.97 for ~800 Hz cutoff at 44.1k */
    float peak_amp;   /* 10000 */
} snare_voice_t;

/* Hi-hat: noise through one-pole HPF near 5 kHz, linear decay 30 ms,
 *         peak amp 6000. */
typedef struct {
    int samples_remaining;
    int samples_total;
    float hpf_x_prev;
    float hpf_y_prev;
    float hpf_coef; /* lower (≈ 0.55) for ~5 kHz cutoff */
    float peak_amp; /* 6000 */
} hat_voice_t;

static kick_voice_t s_kick;
static snare_voice_t s_snare;
static hat_voice_t s_hat;

/* ── Master / sequencer state ────────────────────────────── */

static volatile int s_pattern_idx = 0; /* 0 = silent, 1..4 = active */
static volatile int s_bpm = 110;
static volatile float s_drum_volume = 0.5f;
static volatile int s_step = 0;
static esp_timer_handle_t s_timer = NULL;
static bool s_timer_running = false;

/* ── Envelope / coefficient derivations ──────────────────── */

/* One-pole HPF approximation:
 *   y[n] = x[n] - x[n-1] + a * y[n-1]
 * For a 1st-order HPF at 44.1 kHz, the pole a = exp(-2π fc / fs).
 *   fc =   800 Hz → a ≈ 0.893  (spec says "0.97 as approximation" — use 0.97
 *                                per PRD-like spec for recognisable snare body)
 *   fc =  5000 Hz → a ≈ 0.492
 *
 * The spec for snare calls out 0.97 explicitly as an approximation — honour it
 * so the snare has the "bright crack + body" character the prompt describes.
 * For the hi-hat the spec asks for a different coefficient near 5 kHz; 0.55
 * is in that neighbourhood and gives a perceptually hatty timbre.
 */
#define SNARE_HPF_COEF 0.97f
#define HAT_HPF_COEF 0.55f

/* Voice durations in ms (converted to samples at runtime). */
#define KICK_DURATION_MS 120
#define KICK_ATTACK_MS 5
#define SNARE_DURATION_MS 80
#define HAT_DURATION_MS 30

/* ── Voice trigger helpers ───────────────────────────────── */

static inline int ms_to_samples(int ms)
{
    return (DRUMS_SAMPLE_RATE * ms) / 1000;
}

static void trigger_kick(void)
{
    s_kick.samples_total = ms_to_samples(KICK_DURATION_MS);
    s_kick.attack_samples = ms_to_samples(KICK_ATTACK_MS);
    s_kick.samples_remaining = s_kick.samples_total;
    s_kick.phase = 0.0f;
    s_kick.freq_start = 80.0f;
    s_kick.freq_end = 40.0f;
    s_kick.peak_amp = 12000.0f;
}

static void trigger_snare(void)
{
    s_snare.samples_total = ms_to_samples(SNARE_DURATION_MS);
    s_snare.samples_remaining = s_snare.samples_total;
    /* Keep tri_phase running across hits so successive snares aren't phase-locked. */
    s_snare.tri_freq = 200.0f;
    s_snare.hpf_x_prev = 0.0f;
    s_snare.hpf_y_prev = 0.0f;
    s_snare.hpf_coef = SNARE_HPF_COEF;
    s_snare.peak_amp = 10000.0f;
}

static void trigger_hat(void)
{
    s_hat.samples_total = ms_to_samples(HAT_DURATION_MS);
    s_hat.samples_remaining = s_hat.samples_total;
    s_hat.hpf_x_prev = 0.0f;
    s_hat.hpf_y_prev = 0.0f;
    s_hat.hpf_coef = HAT_HPF_COEF;
    s_hat.peak_amp = 6000.0f;
}

/* ── Sequencer ───────────────────────────────────────────── */

/* Tick interval in µs for a given BPM: 16th-note grid = bpm * 4 per minute. */
static int64_t tick_period_us(int bpm)
{
    if (bpm < 60)
        bpm = 60;
    if (bpm > 200)
        bpm = 200;
    return (int64_t)60 * 1000 * 1000 / ((int64_t)bpm * 4);
}

static void sequencer_tick_cb(void *arg)
{
    (void)arg;
    int idx = s_pattern_idx;
    if (idx < 1 || idx > 4) {
        return;
    }
    const drum_pattern_t *p = &s_patterns[idx - 1];
    int step = s_step;
    uint16_t mask = (uint16_t)(1u << step);

    if (p->kick & mask)
        trigger_kick();
    if (p->snare & mask)
        trigger_snare();
    if (p->hat & mask)
        trigger_hat();

    s_step = (step + 1) & 0x0F;
}

static void sequencer_start(void)
{
    if (s_timer == NULL) {
        return;
    }
    if (s_timer_running) {
        esp_timer_stop(s_timer);
        s_timer_running = false;
    }
    s_step = 0;
    esp_err_t err = esp_timer_start_periodic(s_timer, (uint64_t)tick_period_us(s_bpm));
    if (err == ESP_OK) {
        s_timer_running = true;
    } else {
        ESP_LOGW(TAG, "esp_timer_start_periodic failed: %d", err);
    }
}

static void sequencer_stop(void)
{
    if (s_timer != NULL && s_timer_running) {
        esp_timer_stop(s_timer);
        s_timer_running = false;
    }
    s_step = 0;
    /* Let any in-flight voice tails finish naturally — don't zero samples_remaining. */
}

/* ── Public API ──────────────────────────────────────────── */

void drums_init(void)
{
    drums_init_sine_lut();

    memset(&s_kick, 0, sizeof(s_kick));
    memset(&s_snare, 0, sizeof(s_snare));
    memset(&s_hat, 0, sizeof(s_hat));

    s_pattern_idx = 0;
    s_bpm = 110;
    s_drum_volume = 0.5f;
    s_step = 0;

    const esp_timer_create_args_t args = {
        .callback = sequencer_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "drum_seq",
        .skip_unhandled_events = true,
    };
    esp_err_t err = esp_timer_create(&args, &s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %d", err);
        s_timer = NULL;
    }
    ESP_LOGI(TAG, "drum engine initialised (bpm=%d vol=%.2f)", s_bpm, (double)s_drum_volume);
}

void drums_set_pattern(int idx)
{
    if (idx < 0)
        idx = 0;
    if (idx > 4)
        idx = 4;

    int prev = s_pattern_idx;
    s_pattern_idx = idx;

    if (idx == 0) {
        sequencer_stop();
    } else if (prev == 0 || prev != idx) {
        /* Restart so the downbeat aligns with the pattern switch. */
        sequencer_start();
    }
    ESP_LOGI(TAG, "pattern: %d", idx);
}

void drums_set_tempo(int bpm)
{
    if (bpm < 60)
        bpm = 60;
    if (bpm > 200)
        bpm = 200;
    if (bpm == s_bpm)
        return;
    s_bpm = bpm;

    /* If the sequencer is running, re-arm at the new period. */
    if (s_timer != NULL && s_timer_running) {
        esp_timer_stop(s_timer);
        esp_err_t err = esp_timer_start_periodic(s_timer, (uint64_t)tick_period_us(s_bpm));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_timer_start_periodic (retempo) failed: %d", err);
            s_timer_running = false;
        }
    }
}

void drums_set_volume(float v)
{
    if (v < 0.0f)
        v = 0.0f;
    if (v > 1.0f)
        v = 1.0f;
    s_drum_volume = v;
}

/* ── Per-sample voice rendering ──────────────────────────── */

static inline float kick_sample(void)
{
    if (s_kick.samples_remaining <= 0)
        return 0.0f;

    int pos = s_kick.samples_total - s_kick.samples_remaining;

    /* Linear pitch sweep 80 Hz → 40 Hz over the voice duration. */
    float t = (float)pos / (float)s_kick.samples_total;
    float freq = s_kick.freq_start + (s_kick.freq_end - s_kick.freq_start) * t;

    /* AD amp envelope (attack up then linear decay down). */
    float env;
    if (pos < s_kick.attack_samples) {
        env = (float)pos / (float)s_kick.attack_samples;
    } else {
        int decay_total = s_kick.samples_total - s_kick.attack_samples;
        int decay_pos = pos - s_kick.attack_samples;
        env = 1.0f - (float)decay_pos / (float)decay_total;
    }
    if (env < 0.0f)
        env = 0.0f;

    /* Advance phase with per-sample increment. */
    s_kick.phase += freq / (float)DRUMS_SAMPLE_RATE;
    if (s_kick.phase >= 1.0f)
        s_kick.phase -= (float)((int)s_kick.phase);

    int idx = (int)(s_kick.phase * DRUMS_SINE_TABLE_SIZE) & (DRUMS_SINE_TABLE_SIZE - 1);
    /* Normalise LUT to ±1 then scale by peak * env. */
    float osc = (float)s_sine_lut[idx] / 32000.0f;

    s_kick.samples_remaining--;
    return osc * env * s_kick.peak_amp;
}

static inline float snare_sample(void)
{
    if (s_snare.samples_remaining <= 0)
        return 0.0f;

    int pos = s_snare.samples_total - s_snare.samples_remaining;
    float env = 1.0f - (float)pos / (float)s_snare.samples_total;
    if (env < 0.0f)
        env = 0.0f;

    /* Noise component: signed [-1, 1]. */
    float noise = ((float)((int32_t)(esp_random() & 0xFFFFu) - 0x8000)) / 32768.0f;

    /* Triangle at 200 Hz in [-1, 1]. */
    s_snare.tri_phase += s_snare.tri_freq / (float)DRUMS_SAMPLE_RATE;
    if (s_snare.tri_phase >= 1.0f)
        s_snare.tri_phase -= (float)((int)s_snare.tri_phase);
    float tri = (s_snare.tri_phase < 0.5f) ? (4.0f * s_snare.tri_phase - 1.0f)
                                           : (3.0f - 4.0f * s_snare.tri_phase);

    /* 50/50 sum, then one-pole HPF. */
    float x = 0.5f * noise + 0.5f * tri;
    float y = x - s_snare.hpf_x_prev + s_snare.hpf_coef * s_snare.hpf_y_prev;
    s_snare.hpf_x_prev = x;
    s_snare.hpf_y_prev = y;

    s_snare.samples_remaining--;
    return y * env * s_snare.peak_amp;
}

static inline float hat_sample(void)
{
    if (s_hat.samples_remaining <= 0)
        return 0.0f;

    int pos = s_hat.samples_total - s_hat.samples_remaining;
    float env = 1.0f - (float)pos / (float)s_hat.samples_total;
    if (env < 0.0f)
        env = 0.0f;

    float noise = ((float)((int32_t)(esp_random() & 0xFFFFu) - 0x8000)) / 32768.0f;
    float y = noise - s_hat.hpf_x_prev + s_hat.hpf_coef * s_hat.hpf_y_prev;
    s_hat.hpf_x_prev = noise;
    s_hat.hpf_y_prev = y;

    s_hat.samples_remaining--;
    return y * env * s_hat.peak_amp;
}

/* ── Block render (mix into existing stereo_buf) ─────────── */

void drums_render_block(int16_t *stereo_buf, int block_size)
{
    if (stereo_buf == NULL || block_size <= 0)
        return;

    /* Fast-path: nothing to render if all voices are idle. */
    if (s_kick.samples_remaining <= 0 && s_snare.samples_remaining <= 0 &&
        s_hat.samples_remaining <= 0) {
        return;
    }

    float vol = s_drum_volume;

    for (int i = 0; i < block_size; i++) {
        float mono = 0.0f;
        mono += kick_sample();
        mono += snare_sample();
        mono += hat_sample();

        /* Pre-clip the drum mono sum so stacked voices can't wrap before the
         * master scalar; then apply volume. */
        int16_t drum = clip_i16(mono);
        float scaled = (float)drum * vol;

        /* Saturating add into both L and R. */
        int idx_l = i * 2;
        int idx_r = idx_l + 1;
        stereo_buf[idx_l] = clip_i16((float)stereo_buf[idx_l] + scaled);
        stereo_buf[idx_r] = clip_i16((float)stereo_buf[idx_r] + scaled);
    }
}
