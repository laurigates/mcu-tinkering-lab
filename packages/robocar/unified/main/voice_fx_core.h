/**
 * @file voice_fx_core.h
 * @brief Retro-robot body resonance applied to the voice on its way to I2S.
 *
 * Teuvo is a 1950s retrofuturist robot — Robby the Robot rather than a Dalek —
 * and the half of that character the TTS model cannot supply is the sound of a
 * voice coming out of a large metal body. This module adds it: a feedback comb
 * (the enclosure) followed by a soft saturation (the amplifier inside him).
 *
 * WHY THIS IS DSP AND NOT A PROMPT. `tts_style` steers delivery, and a sweep of
 * seven directives over one Finnish line (2026-09) moved tempo by up to 2.7x and
 * measurably changed timbre — so pace, weight and rasp are all promptable and
 * belong in voice_persona.c, not here. The same sweep asked the model, in
 * Finnish, for a narrow band-limited "old radio" sound and got audio with MORE
 * high-frequency energy than the unprompted control (0.77% vs 0.59% above
 * 4 kHz). The medium is not promptable; it is a filter. This is that filter.
 *
 * WHAT IS DELIBERATELY ABSENT. An earlier design sketch proposed four effects:
 * ring modulation, this comb, a bitcrusher and an impulse-train "drone". Only
 * the comb survived audition:
 *
 *   - Ring modulation is period-correct (the Dalek voice is a real early-60s
 *     studio artifact) but costs intelligibility, and Finnish contrasts long
 *     and short vowels and geminates — the language can afford it least.
 *   - Bitcrushing is a 1980s digital artifact with no place in a 1950s vision,
 *     and AND-masking two's-complement samples truncates toward -inf, adding a
 *     DC offset of about -Q/2 straight into a BTL class-D voice coil.
 *   - The "drone" sampled |x[n]| at one instant per period, which at 85 Hz over
 *     a 24 kHz waveform is an arbitrary value per period — crackle, not pitch
 *     flattening. Monotone delivery is better asked for in the prompt.
 *
 * A BAND-LIMIT IS ALSO ABSENT, and that was a choice rather than an omission.
 * Band-limiting says "you are hearing a 1956 recording OF Robby"; leaving it out
 * says "Robby is in the room with you". Teuvo is in the room. The chain is
 * therefore comb + saturation at full bandwidth (the `in-the-room` preview
 * chain in tools/voice-fx-preview.py, which is where these defaults came from).
 *
 * THE (1-g) INPUT TRIM IS LOAD-BEARING. A feedback comb has DC gain 1/(1-g) —
 * 3.6x at the default 0.72, 6.7x at the maximum — so an untrimmed comb is a
 * clipper wearing a resonator's name. With the trim, |x| <= 1 implies |y| <= 1
 * by induction from a zeroed delay line, which is what lets the int16 cast at
 * the end be safe rather than lucky. Pinned by test_voice_fx_core.c, which was
 * mutation-checked by removing the trim.
 *
 * No ESP-IDF dependency: this compiles into the firmware and into test/ from
 * the same source, per the shared-core pattern in .claude/rules/testing.md.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest supported body delay. 512 samples is 21.3 ms at 24 kHz, i.e. a comb
 *  fundamental down to ~47 Hz — well past any body worth hearing, and 2 kB of
 *  .bss. Sized in SAMPLES rather than ms because the buffer is static. */
#define VOICE_FX_MAX_DELAY_SAMPLES 512

/** Accepted parameter ranges. `voice fx` refuses anything outside them rather
 *  than clamping silently, so a typo is visible instead of being absorbed. */
#define VOICE_FX_BODY_MS_MIN 1.0f
#define VOICE_FX_BODY_MS_MAX 20.0f
#define VOICE_FX_FEEDBACK_MIN 0.0f
#define VOICE_FX_FEEDBACK_MAX 0.85f
#define VOICE_FX_DRIVE_MIN 0.1f
#define VOICE_FX_DRIVE_MAX 4.0f

/** Boot defaults — the `in-the-room` chain, chosen by ear from an
 *  eight-variant sweep. 6.5 ms resonates near 154 Hz.
 *
 *  Not persisted to NVS, for the same reason as `voice volume`, `voice scene`
 *  and `cam gainceiling`: a boot comes up at the documented default rather than
 *  at whatever last night's experiment left behind. */
#define VOICE_FX_DEFAULT_BODY_MS 6.5f
#define VOICE_FX_DEFAULT_FEEDBACK 0.72f
#define VOICE_FX_DEFAULT_DRIVE 1.5f
#define VOICE_FX_DEFAULT_ENABLED true

typedef struct {
    bool enabled;
    float feedback;   /**< g in y[n] = (1-g)x[n] + g*y[n-D]. */
    float drive;      /**< tanh drive; 1.0 is nearly linear. */
    float drive_norm; /**< tanhf(drive), cached so the hot loop has no tanhf of a constant. */
    float body_ms;    /**< Kept verbatim so `voice fx` reports what was asked for. */
    uint32_t rate_hz;
    size_t delay_samples;
    size_t write_idx;
    /** Delay line in FLOAT, not int16. The comb feeds its own output back, so
     *  an int16 line requantises every pass and accumulates that error at the
     *  feedback gain. 2 kB of .bss buys a clean tail. */
    float line[VOICE_FX_MAX_DELAY_SAMPLES];
} voice_fx_t;

/**
 * @brief Initialise to the boot defaults at @p sample_rate_hz and clear the line.
 *
 * A rate whose default delay would not fit is clamped to the buffer; the
 * accepted body range is checked against the live rate by voice_fx_set_body_ms().
 */
void voice_fx_init(voice_fx_t *fx, uint32_t sample_rate_hz);

/** @brief Zero the delay line, leaving parameters alone.
 *
 * Call between utterances. Without it the previous sentence's comb tail is the
 * first thing the next one rings through — quiet, but audible as a smear in
 * front of a word, and it makes two identical utterances sound different.
 */
void voice_fx_reset(voice_fx_t *fx);

/** @name Parameter setters
 *  Each returns false and changes nothing if the value is out of range.
 *  @{ */
bool voice_fx_set_body_ms(voice_fx_t *fx, float ms);
bool voice_fx_set_feedback(voice_fx_t *fx, float g);
bool voice_fx_set_drive(voice_fx_t *fx, float drive);
/** @} */

void voice_fx_set_enabled(voice_fx_t *fx, bool enabled);

/**
 * @brief Apply the chain in place to @p count mono int16 samples.
 *
 * State persists across calls, which is required: the player hands over ~128
 * samples at a time and a comb whose delay line reset per chunk would ring at
 * the chunk rate instead of at the body frequency.
 *
 * A no-op when disabled — the samples are left byte-identical, so `voice fx off`
 * is a true bypass rather than a unity-gain pass through the arithmetic.
 */
void voice_fx_apply(voice_fx_t *fx, int16_t *samples, size_t count);

#ifdef __cplusplus
}
#endif
