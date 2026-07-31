/**
 * @file ambient_audio.c
 * @brief The ambient-sound gate. See the header for the design, for why both
 *        sub-measures are level-shift invariant, and for why the scores are
 *        latched peaks rather than instantaneous readings.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF, only <math.h> — so
 * test/test_ambient_audio.c builds it on the host with no shims. Every piece of
 * state the module needs from the outside (the clock above all) is injected.
 */

#include "ambient_audio.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Fingerprint construction                                                     */
/* -------------------------------------------------------------------------- */

/** Band edges. Seven low-pass corners produce eight bands: the first low-pass
 *  output, six differences between successive outputs, and whatever is left
 *  above the last corner. Deliberately crude and overlapping — we measure
 *  CHANGE in shape, not spectrum. */
static const float k_band_cutoff_hz[AMBIENT_BANDS - 1] = {
    125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 3000.0f, 5000.0f,
};

/** RMS floor, in LSB. log10f(0) is -inf and would poison every later
 *  comparison, and digital silence is a legitimate reading that must produce a
 *  defined level rather than a NaN. One LSB is the smallest thing the ADC can
 *  actually distinguish, so it is the honest floor. */
#define AMBIENT_RMS_FLOOR 1.0f

/** Round to nearest, never truncate. Truncation biases every band level down by
 *  half a dB on average; the bias partially cancels when the mean is subtracted,
 *  which is exactly what would make it hard to notice. */
static int round_db(float v)
{
    return (int)roundf(v);
}

static int8_t clamp_i8(int v)
{
    if (v > 127) {
        return (int8_t)127;
    }
    if (v < -128) {
        return (int8_t)-128;
    }
    return (int8_t)v;
}

void ambient_fingerprint_from_pcm(const int16_t *pcm, size_t n, ambient_fingerprint_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (pcm == NULL || n < (size_t)AMBIENT_MIN_SAMPLES) {
        return; /* leaves valid = false */
    }

    /* DC first. A PDM microphone carries a large subsonic term; left in, it
     * lands entirely in band 0, swamps the other seven, and the shape
     * fingerprint degenerates to the same constant every frame. The sum is
     * exact in int64 so the mean does not itself become a source of drift. */
    int64_t sum = 0;
    for (size_t i = 0; i < n; ++i) {
        sum += pcm[i];
    }
    const double mean = (double)sum / (double)n;

    /* Parallel one-pole low-pass bank over the DC-removed signal. Each filter is
     * linear, so scaling the input by k scales every band signal by exactly k —
     * which is what makes the log-domain gain invariance below exact rather than
     * approximate. */
    float lp[AMBIENT_BANDS - 1];
    float coeff[AMBIENT_BANDS - 1];
    float energy[AMBIENT_BANDS];
    for (int b = 0; b < AMBIENT_BANDS - 1; ++b) {
        lp[b] = 0.0f;
        const float w =
            2.0f * 3.14159265358979f * k_band_cutoff_hz[b] / (float)AMBIENT_SAMPLE_RATE_HZ;
        coeff[b] = 1.0f - expf(-w);
    }
    for (int b = 0; b < AMBIENT_BANDS; ++b) {
        energy[b] = 0.0f;
    }

    float total_energy = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const float x = (float)((double)pcm[i] - mean);
        total_energy += x * x;

        float prev = 0.0f;
        for (int b = 0; b < AMBIENT_BANDS - 1; ++b) {
            lp[b] += coeff[b] * (x - lp[b]);
            const float band = lp[b] - prev;
            energy[b] += band * band;
            prev = lp[b];
        }
        const float top = x - prev; /* everything above the last corner */
        energy[AMBIENT_BANDS - 1] += top * top;
    }

    const float inv_n = 1.0f / (float)n;

    float band_db[AMBIENT_BANDS];
    float db_sum = 0.0f;
    for (int b = 0; b < AMBIENT_BANDS; ++b) {
        float rms = sqrtf(energy[b] * inv_n);
        if (rms < AMBIENT_RMS_FLOOR) {
            rms = AMBIENT_RMS_FLOOR;
        }
        band_db[b] = 20.0f * log10f(rms);
        db_sum += band_db[b];
    }

    /* Deviation from the mean of the bands. A uniform gain adds the same
     * constant to every band_db, and that constant cancels here exactly — the
     * audio analogue of scene_change subtracting the frame mean, and the whole
     * reason the mic's own AGC cannot masquerade as the room changing. */
    const float db_mean = db_sum / (float)AMBIENT_BANDS;
    for (int b = 0; b < AMBIENT_BANDS; ++b) {
        out->band[b] = clamp_i8(round_db(band_db[b] - db_mean));
    }

    float total_rms = sqrtf(total_energy * inv_n);
    if (total_rms < AMBIENT_RMS_FLOOR) {
        total_rms = AMBIENT_RMS_FLOOR;
    }
    out->level_db = (int16_t)round_db(20.0f * log10f(total_rms));

    out->valid = true;
}

unsigned ambient_fingerprint_distance(const ambient_fingerprint_t *a,
                                      const ambient_fingerprint_t *b)
{
    if (a == NULL || b == NULL || !a->valid || !b->valid) {
        return 0;
    }

    uint32_t total = 0;
    for (int i = 0; i < AMBIENT_BANDS; ++i) {
        const int d = (int)a->band[i] - (int)b->band[i];
        total += (uint32_t)((d < 0) ? -d : d);
    }
    return (unsigned)(total / AMBIENT_BANDS);
}

/* -------------------------------------------------------------------------- */
/* Gate state                                                                   */
/* -------------------------------------------------------------------------- */

/** Largest inter-frame gap the floor tracker will integrate, in ms. The
 *  listener can be starved for seconds (a voice turn holds the microphone), and
 *  crediting the floor with a 10 s rise on the first frame back would swallow
 *  the very transient that ended the silence. */
#define AMBIENT_FLOOR_MAX_DT_MS 1000u

static ambient_fingerprint_t s_current;   /**< Most recent measurable frame. */
static ambient_fingerprint_t s_reference; /**< The room the robot last spoke about. */

static float s_floor_db;        /**< Adapted noise floor. */
static bool s_floor_valid;      /**< False until the first frame seeds it. */
static uint32_t s_last_note_ms; /**< Clock at the previous accepted frame. */

static float s_loud_peak_level; /**< Absolute level of the latched peak. */
static float s_loud_peak_floor; /**< Floor as it stood when that peak was set. */
static bool s_loud_valid;       /**< A peak is latched. */
static uint32_t s_loud_ms;      /**< When it was set. */

static unsigned s_shape_peak; /**< Largest shape distance since mark_spoken. */
static bool s_shape_valid;
static uint32_t s_shape_ms;

static uint8_t s_loud_threshold = AMBIENT_LOUD_THRESHOLD_DB_DEFAULT;
static uint8_t s_shape_threshold = AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT;
static uint32_t s_latch_ttl_ms = AMBIENT_LATCH_TTL_MS_DEFAULT;

void ambient_audio_init(void)
{
    memset(&s_current, 0, sizeof(s_current));
    memset(&s_reference, 0, sizeof(s_reference));
    s_floor_db = 0.0f;
    s_floor_valid = false;
    s_last_note_ms = 0u;
    s_loud_peak_level = 0.0f;
    s_loud_peak_floor = 0.0f;
    s_loud_valid = false;
    s_loud_ms = 0u;
    s_shape_peak = 0u;
    s_shape_valid = false;
    s_shape_ms = 0u;
    s_loud_threshold = AMBIENT_LOUD_THRESHOLD_DB_DEFAULT;
    s_shape_threshold = AMBIENT_SHAPE_THRESHOLD_DB_DEFAULT;
    s_latch_ttl_ms = AMBIENT_LATCH_TTL_MS_DEFAULT;
}

void ambient_audio_note(const ambient_fingerprint_t *fp, uint32_t now_ms)
{
    if (fp == NULL || !fp->valid) {
        /* Keep the last frame we could actually measure. A frame that failed to
         * arrive says nothing about whether the room changed, and letting a run
         * of them overwrite the current fingerprint would read as a brand-new
         * soundscape and trigger speech about nothing. */
        return;
    }
    s_current = *fp;

    const float level = (float)fp->level_db;

    if (!s_floor_valid) {
        /* Seed the floor at the first thing we hear rather than at 0 dB —
         * otherwise the first minute of every boot is one long excursion. */
        s_floor_db = level;
        s_floor_valid = true;
        s_last_note_ms = now_ms;
    } else {
        uint32_t dt = now_ms - s_last_note_ms; /* unsigned: wrap-safe */
        if (dt > AMBIENT_FLOOR_MAX_DT_MS) {
            dt = AMBIENT_FLOOR_MAX_DT_MS;
        }
        s_last_note_ms = now_ms;

        if (level < s_floor_db) {
            /* Falls fast: the moment a sustained noise stops, the next real
             * transient must be measured against the quiet room again. */
            const float alpha = 1.0f - expf(-(float)dt / AMBIENT_FLOOR_FALL_TAU_MS);
            s_floor_db += alpha * (level - s_floor_db);
        } else {
            /* Rises slowly, and this rate is the entire definition of "steady":
             * anything that cannot outrun it stops being news. */
            s_floor_db += AMBIENT_FLOOR_RISE_DB_PER_S * (float)dt / 1000.0f;
            if (s_floor_db > level) {
                s_floor_db = level;
            }
        }
    }

    /* Latch the loudest level heard since the robot last spoke, together with
     * the floor at that instant. Storing the LEVEL (not the excursion) is what
     * lets the latch and the floor agree: see the header. */
    if (level > s_floor_db && (!s_loud_valid || level > s_loud_peak_level)) {
        s_loud_peak_level = level;
        s_loud_peak_floor = s_floor_db;
        s_loud_valid = true;
        s_loud_ms = now_ms;
    }

    if (s_reference.valid) {
        const unsigned d = ambient_fingerprint_distance(&s_current, &s_reference);
        if (d > 0u && (!s_shape_valid || d > s_shape_peak)) {
            s_shape_peak = d;
            s_shape_valid = true;
            s_shape_ms = now_ms;
        }
    }
}

unsigned ambient_audio_loud_score(void)
{
    if (!s_loud_valid) {
        return 0u;
    }
    /* Measure the peak against the floor as it stands NOW, but never below the
     * floor it was measured against. The max() is what makes a transient's score
     * survive the floor falling back to a quiet room, while a steady sound's
     * score is eaten by the floor rising through it. */
    const float ref = (s_floor_db > s_loud_peak_floor) ? s_floor_db : s_loud_peak_floor;
    const float score = s_loud_peak_level - ref;
    if (score <= 0.0f) {
        return 0u;
    }
    return (unsigned)round_db(score);
}

unsigned ambient_audio_shape_score(void)
{
    return s_shape_valid ? s_shape_peak : 0u;
}

int16_t ambient_audio_floor_db(void)
{
    return (int16_t)round_db(s_floor_db);
}

bool ambient_audio_novel(uint32_t now_ms)
{
    const bool loud_enabled = (s_loud_threshold != 0u);
    const bool shape_enabled = (s_shape_threshold != 0u);

    /* Both sub-gates disabled: this whole term drops out of the OR, which means
     * FALSE — the inverse of scene_change's disabled state, because the neutral
     * element follows the operator. See the header. */
    if (!loud_enabled && !shape_enabled) {
        return false;
    }

    if (!s_reference.valid) {
        return true; /* nothing spoken about yet — the first impression is new */
    }

    bool loud = false;
    if (loud_enabled && s_loud_valid) {
        /* Timestamp read before the value: a torn read then expires a live latch
         * rather than reviving a dead one, i.e. it fails toward silence. */
        const uint32_t age = now_ms - s_loud_ms;
        loud = (age < s_latch_ttl_ms) && (ambient_audio_loud_score() >= (unsigned)s_loud_threshold);
    }

    bool shape = false;
    if (shape_enabled && s_shape_valid) {
        const uint32_t age = now_ms - s_shape_ms;
        shape =
            (age < s_latch_ttl_ms) && (ambient_audio_shape_score() >= (unsigned)s_shape_threshold);
    }

    return loud || shape;
}

void ambient_audio_mark_spoken(void)
{
    /* Adopts the LAST NOTED frame. A caller that held the microphone through a
     * voice turn must note a fresh post-turn frame first — see the header. */
    s_reference = s_current;

    s_loud_valid = false;
    s_loud_peak_level = 0.0f;
    s_loud_peak_floor = 0.0f;
    s_loud_ms = 0u;

    s_shape_peak = 0u;
    s_shape_valid = false;
    s_shape_ms = 0u;
}

void ambient_audio_set_loud_threshold(uint8_t db)
{
    s_loud_threshold = db;
}

void ambient_audio_set_shape_threshold(uint8_t db)
{
    s_shape_threshold = db;
}

uint8_t ambient_audio_loud_threshold(void)
{
    return s_loud_threshold;
}

uint8_t ambient_audio_shape_threshold(void)
{
    return s_shape_threshold;
}

void ambient_audio_set_latch_ttl_ms(uint32_t ms)
{
    s_latch_ttl_ms = ms;
}

uint32_t ambient_audio_latch_ttl_ms(void)
{
    return s_latch_ttl_ms;
}

/* -------------------------------------------------------------------------- */
/* Capture policy                                                               */
/* -------------------------------------------------------------------------- */

bool ambient_capture_allowed(bool playback_active, uint32_t now_ms, uint32_t last_playback_end_ms,
                             uint32_t hangover_ms)
{
    if (playback_active) {
        return false;
    }
    if (hangover_ms == 0u) {
        return true;
    }
    /* Unsigned difference, never `now > end + hangover`: the latter overflows at
     * the uint32 wrap and deafens the robot for the rest of the 49-day cycle. */
    return (uint32_t)(now_ms - last_playback_end_ms) >= hangover_ms;
}
