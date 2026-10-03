/**
 * @file speech_trigger.c
 * @brief Speech-shaped VAD trigger. See speech_trigger.h for the rule, the
 *        defaults and what the sustain time does and does not reject.
 *
 * Pure C — no FreeRTOS, no ESP-IDF, only <math.h> — so
 * test/test_speech_trigger.c builds it on the host.
 */

#include "speech_trigger.h"

#include <math.h>
#include <string.h>

/** One biquad's coefficients, normalised so a0 == 1. */
typedef struct {
    float b0, b1, b2, a1, a2;
} biquad_t;

static biquad_t s_hp;
static biquad_t s_lp;
static bool s_coeffs_ready;

static uint8_t s_share_pct = SPEECH_TRIGGER_SHARE_PCT_DEFAULT;
static uint8_t s_margin_db = SPEECH_TRIGGER_MARGIN_DB_DEFAULT;
static uint32_t s_sustain_ms = SPEECH_TRIGGER_SUSTAIN_MS_DEFAULT;

static uint32_t s_run_ms;
static uint32_t s_last_note_ms;
static bool s_have_last_note;
static uint8_t s_last_share;
static uint32_t s_best_run_ms;
static uint8_t s_peak_share;

/** RBJ audio-EQ-cookbook 2nd-order Butterworth (Q = 1/sqrt 2). */
static biquad_t make_biquad(float fc_hz, bool highpass)
{
    const float w0 = 2.0f * 3.14159265358979f * fc_hz / (float)SPEECH_TRIGGER_SAMPLE_RATE_HZ;
    const float c = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    biquad_t q;
    if (highpass) {
        q.b0 = (1.0f + c) / 2.0f / a0;
        q.b1 = -(1.0f + c) / a0;
    } else {
        q.b0 = (1.0f - c) / 2.0f / a0;
        q.b1 = (1.0f - c) / a0;
    }
    q.b2 = q.b0;
    q.a1 = -2.0f * c / a0;
    q.a2 = (1.0f - alpha) / a0;
    return q;
}

static void ensure_coeffs(void)
{
    if (!s_coeffs_ready) {
        s_hp = make_biquad(SPEECH_TRIGGER_BAND_LO_HZ, true);
        s_lp = make_biquad(SPEECH_TRIGGER_BAND_HI_HZ, false);
        s_coeffs_ready = true;
    }
}

void speech_trigger_init(void)
{
    ensure_coeffs();
    s_share_pct = SPEECH_TRIGGER_SHARE_PCT_DEFAULT;
    s_margin_db = SPEECH_TRIGGER_MARGIN_DB_DEFAULT;
    s_sustain_ms = SPEECH_TRIGGER_SUSTAIN_MS_DEFAULT;
    s_run_ms = 0u;
    s_last_note_ms = 0u;
    s_have_last_note = false;
    s_last_share = 0u;
    s_best_run_ms = 0u;
    s_peak_share = 0u;
}

uint8_t speech_trigger_band_share_pct(const int16_t *pcm, size_t n)
{
    if (pcm == NULL || n == 0u) {
        return 0u;
    }
    ensure_coeffs();

    /* DC first, as in ambient_audio.c: the PDM mic's subsonic term would
     * otherwise count as out-of-band energy and depress every share. */
    int64_t sum = 0;
    for (size_t i = 0; i < n; ++i) {
        sum += pcm[i];
    }
    const float mean = (float)((double)sum / (double)n);

    /* Transposed direct form II, both filters starting from rest each frame. */
    float h1 = 0.0f, h2 = 0.0f; /* high-pass state */
    float l1 = 0.0f, l2 = 0.0f; /* low-pass state */
    float e_total = 0.0f;
    float e_band = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const float x = (float)pcm[i] - mean;
        e_total += x * x;

        const float hp = s_hp.b0 * x + h1;
        h1 = s_hp.b1 * x - s_hp.a1 * hp + h2;
        h2 = s_hp.b2 * x - s_hp.a2 * hp;

        const float bp = s_lp.b0 * hp + l1;
        l1 = s_lp.b1 * hp - s_lp.a1 * bp + l2;
        l2 = s_lp.b2 * hp - s_lp.a2 * bp;

        e_band += bp * bp;
    }

    /* Below one LSB RMS there is nothing to measure. */
    if (e_total < (float)n) {
        return 0u;
    }
    const float share = 100.0f * e_band / e_total;
    if (share >= 100.0f) {
        return 100u; /* filter ringing can overshoot slightly on a pure tone */
    }
    return (uint8_t)lroundf(share);
}

bool speech_trigger_note(const int16_t *pcm, size_t n, int16_t level_db, int16_t floor_db,
                         uint32_t now_ms)
{
    /* Unsigned difference: correct across the uint32 wrap. */
    if (s_have_last_note && (uint32_t)(now_ms - s_last_note_ms) > SPEECH_TRIGGER_MAX_GAP_MS) {
        s_run_ms = 0u;
    }
    s_last_note_ms = now_ms;
    s_have_last_note = true;

    const uint8_t share = speech_trigger_band_share_pct(pcm, n);
    s_last_share = share;

    const bool loud_enough =
        (s_margin_db == 0u) || (((int32_t)level_db - (int32_t)floor_db) >= (int32_t)s_margin_db);
    if (loud_enough && share > s_peak_share) {
        s_peak_share = share;
    }
    const bool shaped = (s_share_pct == 0u) || (share >= s_share_pct);

    if (!(loud_enough && shaped)) {
        s_run_ms = 0u;
        return false;
    }

    s_run_ms += (uint32_t)(((uint64_t)n * 1000u) / SPEECH_TRIGGER_SAMPLE_RATE_HZ);
    if (s_run_ms > s_best_run_ms) {
        s_best_run_ms = s_run_ms;
    }
    return s_run_ms >= s_sustain_ms;
}

void speech_trigger_reset_run(void)
{
    s_run_ms = 0u;
}

uint32_t speech_trigger_run_ms(void)
{
    return s_run_ms;
}

uint8_t speech_trigger_last_share_pct(void)
{
    return s_last_share;
}

void speech_trigger_best(uint32_t *run_ms, uint8_t *peak_share_pct, bool clear)
{
    if (run_ms) {
        *run_ms = s_best_run_ms;
    }
    if (peak_share_pct) {
        *peak_share_pct = s_peak_share;
    }
    if (clear) {
        s_best_run_ms = 0u;
        s_peak_share = 0u;
    }
}

void speech_trigger_set_share_pct(uint8_t pct)
{
    s_share_pct = (pct > 100u) ? 100u : pct;
}

void speech_trigger_set_margin_db(uint8_t db)
{
    s_margin_db = db;
}

void speech_trigger_set_sustain_ms(uint32_t ms)
{
    s_sustain_ms = ms;
}

uint8_t speech_trigger_share_pct(void)
{
    return s_share_pct;
}

uint8_t speech_trigger_margin_db(void)
{
    return s_margin_db;
}

uint32_t speech_trigger_sustain_ms(void)
{
    return s_sustain_ms;
}
