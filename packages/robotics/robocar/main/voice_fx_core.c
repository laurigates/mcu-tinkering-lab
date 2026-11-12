/**
 * @file voice_fx_core.c
 * @brief Implementation of the retro-robot body resonance. See voice_fx_core.h.
 */

#include "voice_fx_core.h"

#include <math.h>
#include <string.h>

/** Full-scale magnitude for the int16 <-> float conversion.
 *
 *  32767 in BOTH directions, deliberately asymmetric with int16's true -32768
 *  floor: scaling by 32768 on the way out would let a float of exactly -1.0
 *  land on -32768, which is representable, but the matching +1.0 would need
 *  +32768, which is not — it wraps to -32768, a full-scale sign inversion.
 *  Using 32767 both ways costs one LSB and cannot wrap. Same hazard
 *  audio_player_set_volume_pct() guards on its own cast. */
#define VOICE_FX_FULL_SCALE 32767.0f

static size_t clamp_delay(uint32_t rate_hz, float ms)
{
    float samples = (ms / 1000.0f) * (float)rate_hz;
    if (samples < 1.0f) {
        samples = 1.0f;
    }
    size_t d = (size_t)samples;
    if (d >= VOICE_FX_MAX_DELAY_SAMPLES) {
        d = VOICE_FX_MAX_DELAY_SAMPLES - 1;
    }
    return d;
}

void voice_fx_init(voice_fx_t *fx, uint32_t sample_rate_hz)
{
    if (!fx) {
        return;
    }
    memset(fx, 0, sizeof(*fx));
    fx->rate_hz = sample_rate_hz ? sample_rate_hz : 24000u;
    fx->enabled = VOICE_FX_DEFAULT_ENABLED;
    fx->feedback = VOICE_FX_DEFAULT_FEEDBACK;
    fx->body_ms = VOICE_FX_DEFAULT_BODY_MS;
    fx->delay_samples = clamp_delay(fx->rate_hz, VOICE_FX_DEFAULT_BODY_MS);
    fx->drive = VOICE_FX_DEFAULT_DRIVE;
    fx->drive_norm = tanhf(VOICE_FX_DEFAULT_DRIVE);
    fx->write_idx = 0;
}

void voice_fx_reset(voice_fx_t *fx)
{
    if (!fx) {
        return;
    }
    memset(fx->line, 0, sizeof(fx->line));
    fx->write_idx = 0;
}

bool voice_fx_set_body_ms(voice_fx_t *fx, float ms)
{
    if (!fx || !(ms >= VOICE_FX_BODY_MS_MIN) || !(ms <= VOICE_FX_BODY_MS_MAX)) {
        return false;  // the !(>=) form also rejects NaN, which a plain < would pass
    }
    fx->body_ms = ms;
    fx->delay_samples = clamp_delay(fx->rate_hz, ms);
    /* Changing the delay leaves stale samples at indices the new length now
     * reads, which rings as a click. Cheaper to clear than to interpolate, and
     * this is a console-speed operation. */
    voice_fx_reset(fx);
    return true;
}

bool voice_fx_set_feedback(voice_fx_t *fx, float g)
{
    if (!fx || !(g >= VOICE_FX_FEEDBACK_MIN) || !(g <= VOICE_FX_FEEDBACK_MAX)) {
        return false;
    }
    fx->feedback = g;
    return true;
}

bool voice_fx_set_drive(voice_fx_t *fx, float drive)
{
    if (!fx || !(drive >= VOICE_FX_DRIVE_MIN) || !(drive <= VOICE_FX_DRIVE_MAX)) {
        return false;
    }
    fx->drive = drive;
    fx->drive_norm = tanhf(drive);
    return true;
}

void voice_fx_set_enabled(voice_fx_t *fx, bool enabled)
{
    if (!fx) {
        return;
    }
    /* Clear on the OFF->ON edge so switching the effect in mid-stream does not
     * ring through whatever was left in the line when it was switched off. */
    if (enabled && !fx->enabled) {
        voice_fx_reset(fx);
    }
    fx->enabled = enabled;
}

void voice_fx_apply(voice_fx_t *fx, int16_t *samples, size_t count)
{
    if (!fx || !samples || !fx->enabled || fx->delay_samples == 0) {
        return;
    }

    const float g = fx->feedback;
    const float dry = 1.0f - g;
    const float drive = fx->drive;
    /* tanhf(drive) is never 0 for drive >= VOICE_FX_DRIVE_MIN, but a struct
     * zeroed by something other than voice_fx_init() would divide by zero and
     * turn the voice into NaN — silence at best, full-scale noise at worst. */
    const float norm = (fx->drive_norm > 0.0f) ? fx->drive_norm : 1.0f;
    const size_t d = fx->delay_samples;

    for (size_t i = 0; i < count; i++) {
        const float x = (float)samples[i] / VOICE_FX_FULL_SCALE;

        /* Feedback comb. Reading and writing the SAME index is what makes this
         * a D-sample delay: the slot holds y[n-D] until the store replaces it
         * with y[n]. A separate read pointer at (write + MAX - D) % MAX would
         * be a delay of D only while MAX happens to equal the buffer length. */
        const float delayed = fx->line[fx->write_idx];
        float y = dry * x + g * delayed;
        fx->line[fx->write_idx] = y;
        fx->write_idx = (fx->write_idx + 1u) % d;

        /* Soft clip. Normalised by tanh(drive) so `drive` changes character
         * rather than level — otherwise every drive change reads as a volume
         * change and the A/B is worthless. */
        y = tanhf(y * drive) / norm;

        /* The comb bounds |y| <= 1 given |x| <= 1 (see the header), and the
         * normalised tanh maps [-1,1] onto itself, so this clamp should never
         * fire. It is here because "should never" and "cannot" differ by one
         * float rounding step, and the failure mode of the cast below is a
         * wrap to the opposite rail — full-scale noise, not a quiet glitch. */
        if (y > 1.0f) {
            y = 1.0f;
        } else if (y < -1.0f) {
            y = -1.0f;
        }
        samples[i] = (int16_t)(y * VOICE_FX_FULL_SCALE);
    }
}
