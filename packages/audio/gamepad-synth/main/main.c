/**
 * @file main.c
 * @brief ESP32-S3 Gamepad Synth — Bluetooth controller drives I2S audio
 *
 * Four sound modes cycled via View button (Xbox) / Share (PS) / - (Switch):
 *   1. Theremin  — sticks control pitch, vibrato, triggers bend pitch
 *   2. Scale     — face buttons + d-pad play notes, shoulders shift octave
 *   3. Arpeggio  — face buttons pick chord, stick controls speed/pattern
 *   4. Retro SFX — buttons trigger classic game sound effects
 *
 * Bluepad32 runs on Core 0 (BTstack event loop, blocks forever).
 * Core 1 runs two tasks:
 *   - Control task at 50 Hz: reads gamepad, updates synth parameters
 *   - Audio render task: generates samples, writes to I2S DMA (MAX98357A)
 *
 * Tested with: Xbox Series X/S controller (BLE, firmware v5.15+)
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Bluepad32 headers */
#include "uni_bt.h"
#include "uni_controller.h"
#include "uni_esp32.h"
#include "uni_gamepad.h"
#include "uni_hid_device.h"
#include "uni_platform.h"

#include "drums.h"
#include "piezo_voice.h"

static const char *TAG = "gamepad_synth";

/* ── Pin Definitions ─────────────────────────────────────── */

#define I2S_BCLK_PIN GPIO_NUM_5
#define I2S_WS_PIN GPIO_NUM_6
#define I2S_DOUT_PIN GPIO_NUM_7
#define LED_PIN GPIO_NUM_2

#define PIEZO_A_PIN GPIO_NUM_8
#define PIEZO_B_PIN GPIO_NUM_9

/* Detune between the two piezos in Drone mode. Fixed ratio so beating rate
 * scales with pitch: ~4 Hz beat at 200 Hz → ~40 Hz warble at 2 kHz. */
#define PIEZO_DETUNE_RATIO 1.02f

/* ── I2S / Audio Configuration ───────────────────────────── */

#define SAMPLE_RATE 44100
#define BLOCK_SIZE 256
#define DMA_DESC_NUM 4
#define AMPLITUDE 8000
#define SINE_TABLE_SIZE 256

static i2s_chan_handle_t s_tx_chan;

/* ── Waveform Types ──────────────────────────────────────── */

typedef enum {
    WAVE_SQUARE = 0,
    WAVE_SAWTOOTH,
    WAVE_TRIANGLE,
    WAVE_SINE,
    WAVE_NOISE,
    WAVE_COUNT,
} waveform_t;

/* Pre-computed sine table (256 entries, -AMPLITUDE to +AMPLITUDE) */
static int16_t s_sine_table[SINE_TABLE_SIZE];

static void init_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        s_sine_table[i] =
            (int16_t)(AMPLITUDE * sinf(2.0f * 3.14159265f * (float)i / (float)SINE_TABLE_SIZE));
    }
}

static inline int16_t osc_sample(float phase, waveform_t wave)
{
    switch (wave) {
        case WAVE_SQUARE:
            return (phase < 0.5f) ? AMPLITUDE : -AMPLITUDE;
        case WAVE_SAWTOOTH:
            return (int16_t)(AMPLITUDE * (2.0f * phase - 1.0f));
        case WAVE_TRIANGLE: {
            float tri = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
            return (int16_t)(AMPLITUDE * tri);
        }
        case WAVE_SINE: {
            int idx = (int)(phase * SINE_TABLE_SIZE) & (SINE_TABLE_SIZE - 1);
            return s_sine_table[idx];
        }
        case WAVE_NOISE:
            return (int16_t)((esp_random() >> 17) - AMPLITUDE);
        default:
            return 0;
    }
}

/* ── Bluepad32 Button Constants ──────────────────────────── */

#define BTN_A (1 << 0)          /* A (Xbox) / Cross (PS) */
#define BTN_B (1 << 1)          /* B (Xbox) / Circle (PS) */
#define BTN_X (1 << 2)          /* X (Xbox) / Square (PS) */
#define BTN_Y (1 << 3)          /* Y (Xbox) / Triangle (PS) */
#define BTN_SHOULDER_L (1 << 4) /* LB (Xbox) / L1 (PS) */
#define BTN_SHOULDER_R (1 << 5) /* RB (Xbox) / R1 (PS) */

#define DPAD_UP (1 << 0)
#define DPAD_DOWN (1 << 1)
#define DPAD_RIGHT (1 << 2)
#define DPAD_LEFT (1 << 3)

#define MISC_BACK (1 << 1)   /* View (Xbox) / Share (PS) */
#define MISC_HOME (1 << 2)   /* Xbox button / PS button */
#define MISC_SELECT (1 << 3) /* Menu (Xbox) / Options (PS) */

/* ── Sound Parameters ────────────────────────────────────── */

#define MIN_FREQ 100
#define MAX_FREQ 2000
#define STICK_DEADZONE 50 /* ignore stick values below this */
#define STICK_MAX 512
#define TRIGGER_MAX 1023

#define CONTROL_TASK_HZ 50
#define CONTROL_TASK_PERIOD_MS (1000 / CONTROL_TASK_HZ)

/* ── Musical Note Table (C4–B6, equal temperament) ───────── */

static const uint16_t NOTE_FREQ[] = {
    262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466, 494, /* C4–B4 */
    523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932, 988, /* C5–B5 */
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760,           /* C6–A6 */
    1865, 1976,                                                           /* A#6–B6 */
};
#define NOTE_COUNT (sizeof(NOTE_FREQ) / sizeof(NOTE_FREQ[0]))

/* C major scale: indices into NOTE_FREQ within one octave */
static const uint8_t SCALE_MAJOR[] = {0, 2, 4, 5, 7, 9, 11, 12};
#define SCALE_LEN 8

/* Chord patterns (semitone offsets from root) */
static const uint8_t CHORD_MAJOR[] = {0, 4, 7, 12};
static const uint8_t CHORD_MINOR[] = {0, 3, 7, 12};
static const uint8_t CHORD_7TH[] = {0, 4, 7, 10};
static const uint8_t CHORD_DIM[] = {0, 3, 6, 9};
#define CHORD_LEN 4

/* ── Shared Gamepad State ────────────────────────────────── */

typedef struct {
    uint16_t buttons;
    uint8_t dpad;
    uint8_t misc_buttons;
    int16_t axis_x;   /* left stick X: -511..512 */
    int16_t axis_y;   /* left stick Y: -511..512 */
    int16_t axis_rx;  /* right stick X */
    int16_t axis_ry;  /* right stick Y */
    int16_t brake;    /* L2 trigger: 0..1023 */
    int16_t throttle; /* R2 trigger: 0..1023 */
    bool connected;
} gamepad_state_t;

static volatile gamepad_state_t s_gp;

/* ── Synth State (written by control task, read by audio task) ── */

typedef enum {
    LFO_TARGET_NONE = 0,
    LFO_TARGET_PITCH,
    LFO_TARGET_CUTOFF,
    LFO_TARGET_BOTH,
} lfo_target_t;

typedef struct {
    float target_freq;
    bool active;
    waveform_t waveform;
    /* Oscillator B for dual-osc and drone modes */
    bool osc_b_enabled;
    float target_freq_b;
    waveform_t waveform_b;
    bool filter_enabled;
    float filter_cutoff;    /* Hz, [40 .. 18000] */
    float filter_resonance; /* Q-like, [0.5 .. 6.0] */
    lfo_target_t lfo_target;
    float lfo_rate;  /* Hz, [0.1 .. 20] */
    float lfo_depth; /* 0.0 .. 1.0 */
    bool delay_enabled;
    int delay_samples;    /* [1 .. DELAY_MAX_SAMPLES] */
    float delay_feedback; /* 0.0 .. 0.95 */
    float delay_mix;      /* 0.0 dry .. 1.0 wet */
} synth_state_t;

static volatile synth_state_t s_synth = {0};

/* ── State-Variable Filter (Chamberlin form) ─────────────── */

#define FILTER_CUTOFF_MIN 40.0f
#define FILTER_CUTOFF_MAX 18000.0f
#define FILTER_Q_MIN 0.5f
#define FILTER_Q_MAX 6.0f

/* Filter memory lives in the audio render task; only coefficients
 * are recomputed per block from the shared cutoff/resonance values.
 * f = 2 * sin(pi * fc / fs), q = 1 / Q (damping, lower = more resonance)
 */
static inline int16_t svf_process(int16_t sample, float f, float q, float *low, float *band)
{
    float in = (float)sample;
    *low += f * (*band);
    float high = in - (*low) - q * (*band);
    *band += f * high;
    /* Clip to int16 range to prevent resonance overflow */
    float out = *low;
    if (out > 32767.0f)
        out = 32767.0f;
    if (out < -32768.0f)
        out = -32768.0f;
    return (int16_t)out;
}

/* ── LFO (block-rate, triangle wave) ─────────────────────── */

#define LFO_RATE_MIN 0.1f
#define LFO_RATE_MAX 20.0f

/* Triangle wave in [-1, 1] given phase in [0, 1) */
static inline float lfo_triangle(float phase)
{
    return (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
}

/* ── Delay Line (circular buffer, ~44 KB in DRAM) ────────── */

#define DELAY_MAX_SAMPLES 22050 /* 0.5 seconds @ 44.1 kHz */
#define DELAY_FEEDBACK_MAX 0.95f

static int16_t s_delay_buf[DELAY_MAX_SAMPLES];
static int s_delay_write_idx = 0;

/* ── Confirmation Blip ───────────────────────────────────── */

/* Short sine ping mixed over the main voice to confirm silent state changes
 * (octave shifts, chord/interval picks, mode toggles). Control task writes;
 * audio task consumes. Ducking keeps the blip audible when a voice is active.
 */
#define BLIP_DURATION_MS 70
#define BLIP_FREQ_NEUTRAL 880.0f /* A5 — generic state toggle */
#define BLIP_FREQ_UP 1320.0f     /* E6 — "up" direction */
#define BLIP_FREQ_DOWN 660.0f    /* E5 — "down" direction */
#define BLIP_MAIN_DUCK_SHIFT 2   /* >> 2 = -12 dB on the underlying voice */

static volatile int s_blip_samples_remaining = 0;
static volatile float s_blip_freq_hz = BLIP_FREQ_NEUTRAL;

static void synth_blip(float freq_hz)
{
    s_blip_freq_hz = freq_hz;
    s_blip_samples_remaining = (SAMPLE_RATE * BLIP_DURATION_MS) / 1000;
}

static inline int16_t clip_i16(float x)
{
    if (x > 32767.0f)
        return 32767;
    if (x < -32768.0f)
        return -32768;
    return (int16_t)x;
}

static inline int16_t delay_process(int16_t in, int delay_samples, float feedback, float mix)
{
    int read_idx = s_delay_write_idx - delay_samples;
    if (read_idx < 0)
        read_idx += DELAY_MAX_SAMPLES;
    int16_t delayed = s_delay_buf[read_idx];

    /* Write input + feedback into the buffer */
    s_delay_buf[s_delay_write_idx] = clip_i16((float)in + feedback * (float)delayed);
    s_delay_write_idx++;
    if (s_delay_write_idx >= DELAY_MAX_SAMPLES)
        s_delay_write_idx = 0;

    /* Output is dry + wet mix */
    return clip_i16((1.0f - mix) * (float)in + mix * (float)delayed);
}

/* ── Trigger-driven Pitch Bend ───────────────────────────── */

/* LT/RT act as a pitch-bend pair across pitched modes. Range is ±7 semitones,
 * where LT full = −7 st and RT full = +7 st. Both pressed equally cancels out
 * to a 1.0 multiplier; asymmetric presses give a fine detune. */
static inline float compute_bend_mult(const gamepad_state_t *gp)
{
    float bend_semitones =
        ((float)(gp->throttle - gp->brake) / (float)TRIGGER_MAX) * 7.0f;
    if (bend_semitones > 7.0f)
        bend_semitones = 7.0f;
    if (bend_semitones < -7.0f)
        bend_semitones = -7.0f;
    return powf(2.0f, bend_semitones / 12.0f);
}

/* ── Sound Modes ─────────────────────────────────────────── */

typedef enum {
    MODE_MONO = 0,    /* Monotron-style single osc + filter + LFO */
    MODE_DUAL_OSC,    /* Two oscillators with selectable interval + detune */
    MODE_DELAY_SYNTH, /* Single osc + dynamic delay (RY=time, RX=feedback) */
    MODE_SCALE,
    MODE_ARPEGGIO,
    MODE_SFX,
    MODE_DRONE, /* Two sustained oscillators with LFO modulation */
    MODE_COUNT,
} sound_mode_t;

static sound_mode_t s_mode = MODE_MONO;

/* Arpeggiator state */
static const uint8_t *s_arp_chord = CHORD_MAJOR;
static int s_arp_root = 0;      /* semitone offset from C4 */
static int s_arp_index = 0;     /* current note in chord */
static int s_arp_direction = 1; /* 1=up, -1=down */
static bool s_arp_running = false;
static uint32_t s_arp_last_ms = 0;

/* SFX state */
typedef struct {
    float freq;
    float freq_end;
    float step;
    int ticks_left;
} sfx_state_t;
static sfx_state_t s_sfx = {0};

/* Scale octave: 0=C4, 1=C5, 2=C6 */
static int s_octave = 1;

/* Mode switch debounce */
static uint8_t s_prev_misc = 0;

/* Drum engine: remembered pattern when toggled on via Home button.
 * TODO(settings-integration): when the settings-page branch lands and
 * s_settings.drum_pattern / drum_tempo_bpm / drum_volume appear in this
 * file, replace this static with the settings struct and push tempo/volume
 * from the control task each tick. */
static int s_drum_pattern_remembered = 1;
static bool s_drum_on = false;

/* ── Tone Helpers (now set synth state instead of LEDC) ──── */

static void tone_play(uint32_t freq_hz)
{
    if (freq_hz < MIN_FREQ)
        freq_hz = MIN_FREQ;
    if (freq_hz > MAX_FREQ)
        freq_hz = MAX_FREQ;
    s_synth.target_freq = (float)freq_hz;
    s_synth.active = true;
}

static void synth_set_waveform(waveform_t wave)
{
    s_synth.waveform = wave;
}

static void synth_set_osc_b(bool enabled, float freq_hz, waveform_t wave)
{
    if (freq_hz < (float)MIN_FREQ)
        freq_hz = (float)MIN_FREQ;
    if (freq_hz > (float)MAX_FREQ)
        freq_hz = (float)MAX_FREQ;
    s_synth.osc_b_enabled = enabled;
    s_synth.target_freq_b = freq_hz;
    s_synth.waveform_b = wave;
}

static void synth_set_filter(bool enabled, float cutoff, float resonance)
{
    if (cutoff < FILTER_CUTOFF_MIN)
        cutoff = FILTER_CUTOFF_MIN;
    if (cutoff > FILTER_CUTOFF_MAX)
        cutoff = FILTER_CUTOFF_MAX;
    if (resonance < FILTER_Q_MIN)
        resonance = FILTER_Q_MIN;
    if (resonance > FILTER_Q_MAX)
        resonance = FILTER_Q_MAX;
    s_synth.filter_enabled = enabled;
    s_synth.filter_cutoff = cutoff;
    s_synth.filter_resonance = resonance;
}

static void synth_set_lfo(lfo_target_t target, float rate, float depth)
{
    if (rate < LFO_RATE_MIN)
        rate = LFO_RATE_MIN;
    if (rate > LFO_RATE_MAX)
        rate = LFO_RATE_MAX;
    if (depth < 0.0f)
        depth = 0.0f;
    if (depth > 1.0f)
        depth = 1.0f;
    s_synth.lfo_target = target;
    s_synth.lfo_rate = rate;
    s_synth.lfo_depth = depth;
}

static void synth_set_delay(bool enabled, int samples, float feedback, float mix)
{
    if (samples < 1)
        samples = 1;
    if (samples > DELAY_MAX_SAMPLES)
        samples = DELAY_MAX_SAMPLES;
    if (feedback < 0.0f)
        feedback = 0.0f;
    if (feedback > DELAY_FEEDBACK_MAX)
        feedback = DELAY_FEEDBACK_MAX;
    if (mix < 0.0f)
        mix = 0.0f;
    if (mix > 1.0f)
        mix = 1.0f;
    s_synth.delay_enabled = enabled;
    s_synth.delay_samples = samples;
    s_synth.delay_feedback = feedback;
    s_synth.delay_mix = mix;
}

static void tone_stop(void)
{
    s_synth.active = false;
}

static uint16_t note_at(int semitone)
{
    if (semitone < 0)
        semitone = 0;
    if (semitone >= (int)NOTE_COUNT)
        semitone = NOTE_COUNT - 1;
    return NOTE_FREQ[semitone];
}

/* Map a value from [in_min..in_max] to [out_min..out_max] */
static float map_range(float val, float in_min, float in_max, float out_min, float out_max)
{
    if (val < in_min)
        val = in_min;
    if (val > in_max)
        val = in_max;
    return out_min + (out_max - out_min) * ((val - in_min) / (in_max - in_min));
}

/* ── LED Helpers ─────────────────────────────────────────── */

static void led_on(void)
{
    gpio_set_level(LED_PIN, 1);
}
static void led_off(void)
{
    gpio_set_level(LED_PIN, 0);
}

static void led_blink(int count, int on_ms, int off_ms)
{
    for (int i = 0; i < count; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

/* ── Mode 1: Mono Synth (Monotron-style) ─────────────────── */

static void mode_mono(const gamepad_state_t *gp)
{
    int16_t ly = gp->axis_y;
    int16_t lx = gp->axis_x;
    int16_t rx = gp->axis_rx;
    int16_t ry = gp->axis_ry;

    /* Dead zone check — silence if stick is centered */
    if (abs(ly) < STICK_DEADZONE && abs(lx) < STICK_DEADZONE) {
        tone_stop();
        led_off();
        return;
    }

    /* Base pitch from left stick Y */
    float pitch = map_range((float)ly, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);

    /* Manual vibrato from left stick X (depth) — fixed 5 Hz speed */
    float vib_depth = map_range((float)abs(lx), 0, STICK_MAX, 0, 100);
    static uint32_t tick = 0;
    tick++;
    float vib = vib_depth * sinf(2.0f * 3.14159f * 5.0f * (float)tick / CONTROL_TASK_HZ);
    pitch += vib;

    /* Filter: right stick Y = cutoff (logarithmic), right stick X = resonance */
    float cutoff_norm = map_range((float)ry, -STICK_MAX, STICK_MAX, 0.0f, 1.0f);
    float cutoff = FILTER_CUTOFF_MIN * powf(FILTER_CUTOFF_MAX / FILTER_CUTOFF_MIN, cutoff_norm);
    float resonance = map_range((float)abs(rx), 0, STICK_MAX, FILTER_Q_MIN, FILTER_Q_MAX);
    synth_set_filter(true, cutoff, resonance);
    synth_set_lfo(LFO_TARGET_CUTOFF, 5.0f, 0.3f);

    /* Triggers = pitch bend (±7 semitones) */
    float bend_mult = compute_bend_mult(gp);
    pitch *= bend_mult;

    tone_play((uint32_t)pitch);
    led_on();
}

/* ── Mode 2: Dual Osc (two oscillators, face-button interval) ── */

static void mode_dual_osc(const gamepad_state_t *gp)
{
    /* A=unison, B=fifth, X=octave, Y=two octaves */
    static int s_interval_semitones = 0;
    static uint16_t prev_buttons = 0;
    uint16_t pressed = gp->buttons & ~prev_buttons;
    prev_buttons = gp->buttons;

    if (pressed & BTN_A) {
        s_interval_semitones = 0; /* unison */
        synth_blip(BLIP_FREQ_DOWN);
    }
    if (pressed & BTN_B) {
        s_interval_semitones = 7; /* perfect fifth */
        synth_blip(BLIP_FREQ_NEUTRAL);
    }
    if (pressed & BTN_X) {
        s_interval_semitones = 12; /* octave */
        synth_blip(BLIP_FREQ_UP);
    }
    if (pressed & BTN_Y) {
        s_interval_semitones = 24; /* two octaves */
        synth_blip(BLIP_FREQ_UP * 1.5f);
    }

    int16_t ly = gp->axis_y;
    int16_t rx = gp->axis_rx;
    int16_t ry = gp->axis_ry;

    if (abs(ly) < STICK_DEADZONE) {
        tone_stop();
        synth_set_osc_b(false, (float)MIN_FREQ, WAVE_SAWTOOTH);
        led_off();
        return;
    }

    float pitch_a = map_range((float)ly, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);
    /* Detune from RX: ±50 cents */
    float detune_cents = map_range((float)rx, -STICK_MAX, STICK_MAX, -50.0f, 50.0f);
    float detune_mult = powf(2.0f, detune_cents / 1200.0f);
    float pitch_b = pitch_a * powf(2.0f, (float)s_interval_semitones / 12.0f) * detune_mult;

    /* Filter cutoff from RY (logarithmic) */
    float cutoff_norm = map_range((float)ry, -STICK_MAX, STICK_MAX, 0.0f, 1.0f);
    float cutoff = FILTER_CUTOFF_MIN * powf(FILTER_CUTOFF_MAX / FILTER_CUTOFF_MIN, cutoff_norm);
    synth_set_filter(true, cutoff, 1.0f);

    /* Triggers = pitch bend on both oscillators */
    float bend_mult = compute_bend_mult(gp);
    pitch_a *= bend_mult;
    pitch_b *= bend_mult;

    tone_play((uint32_t)pitch_a);
    synth_set_osc_b(true, pitch_b, WAVE_SAWTOOTH);
    led_on();
}

/* ── Mode 3: Delay Synth (single osc + dynamic delay) ────── */

static void mode_delay_synth(const gamepad_state_t *gp)
{
    /* Fixed filter — triggers are repurposed as pitch bend. */
    synth_set_filter(true, 4000.0f, 1.2f);

    int16_t ly = gp->axis_y;
    int16_t rx = gp->axis_rx;
    int16_t ry = gp->axis_ry;

    if (abs(ly) < STICK_DEADZONE) {
        tone_stop();
        led_off();
        return;
    }

    float pitch = map_range((float)ly, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);

    /* Delay time: RY from -STICK_MAX..STICK_MAX → 20..500 ms */
    float delay_ms = map_range((float)ry, -STICK_MAX, STICK_MAX, 20.0f, 500.0f);
    int delay_samples = (int)(SAMPLE_RATE * delay_ms / 1000.0f);
    /* Feedback: RX from -STICK_MAX..STICK_MAX → 0.0..0.9 */
    float feedback = map_range((float)rx, -STICK_MAX, STICK_MAX, 0.0f, 0.9f);
    synth_set_delay(true, delay_samples, feedback, 0.5f);

    /* Triggers = pitch bend (±7 semitones) */
    float bend_mult = compute_bend_mult(gp);
    pitch *= bend_mult;

    tone_play((uint32_t)pitch);
    led_on();
}

/* ── Mode 7: Drone (two sustained oscillators, LFO on both) ── */

static void mode_drone(const gamepad_state_t *gp)
{
    /* Both oscillators always active; sticks control their pitches */
    float pitch_a = map_range((float)gp->axis_y, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);
    float pitch_b = map_range((float)gp->axis_ry, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);

    /* Fixed LFO — triggers repurposed as pitch bend. */
    synth_set_lfo(LFO_TARGET_BOTH, 2.0f, 0.4f);

    /* Filter: slightly warm default, modulated by LFO */
    synth_set_filter(true, 3000.0f, 1.5f);

    /* Triggers = pitch bend on both oscillators */
    float bend_mult = compute_bend_mult(gp);
    pitch_a *= bend_mult;
    pitch_b *= bend_mult;

    tone_play((uint32_t)pitch_a);
    /* Route osc B onto the two piezos with a fixed detune so the beating
     * happens acoustically in air rather than inside the DAC mix. */
    piezo_voice_note_on(PIEZO_A, pitch_b);
    piezo_voice_note_on(PIEZO_B, pitch_b * PIEZO_DETUNE_RATIO);
    led_on();
}

/* ── Mode 4: Scale Player ────────────────────────────────── */

static void mode_scale(const gamepad_state_t *gp)
{
    /* LB/RB shift octave (rising edge) */
    static uint16_t prev_buttons = 0;
    uint16_t pressed = gp->buttons & ~prev_buttons;
    prev_buttons = gp->buttons;

    if (pressed & BTN_SHOULDER_L && s_octave > 0) {
        s_octave--;
        synth_blip(BLIP_FREQ_DOWN);
    }
    if (pressed & BTN_SHOULDER_R && s_octave < 2) {
        s_octave++;
        synth_blip(BLIP_FREQ_UP);
    }

    /* Map buttons to scale degrees */
    int note_idx = -1;
    if (gp->buttons & BTN_A)
        note_idx = 0; /* A = Do */
    if (gp->buttons & BTN_B)
        note_idx = 1; /* B = Re */
    if (gp->buttons & BTN_X)
        note_idx = 2; /* X = Mi */
    if (gp->buttons & BTN_Y)
        note_idx = 3; /* Y = Fa */
    if (gp->dpad & DPAD_UP)
        note_idx = 4; /* Sol */
    if (gp->dpad & DPAD_RIGHT)
        note_idx = 5; /* La */
    if (gp->dpad & DPAD_DOWN)
        note_idx = 6; /* Ti */
    if (gp->dpad & DPAD_LEFT)
        note_idx = 7; /* Do (high) */

    if (note_idx < 0) {
        tone_stop();
        led_off();
        return;
    }

    int semitone = (s_octave * 12) + SCALE_MAJOR[note_idx];
    float freq = (float)note_at(semitone);

    /* Pitch bend from right stick Y — fine bend in Hz. */
    float bend = map_range((float)gp->axis_ry, -STICK_MAX, STICK_MAX, -50, 50);
    freq += bend;

    /* Triggers = coarse pitch bend (±7 semitones) */
    float bend_mult = compute_bend_mult(gp);
    freq *= bend_mult;

    tone_play((uint32_t)freq);
    led_on();
}

/* ── Mode 5: Arpeggiator ────────────────────────────────── */

static void mode_arpeggio(const gamepad_state_t *gp)
{
    /* Face buttons select chord (rising edge) */
    static uint16_t prev_buttons = 0;
    uint16_t pressed = gp->buttons & ~prev_buttons;
    prev_buttons = gp->buttons;

    if (pressed & BTN_A) {
        s_arp_chord = CHORD_MAJOR; /* A = major */
        synth_blip(BLIP_FREQ_UP);
    }
    if (pressed & BTN_B) {
        s_arp_chord = CHORD_MINOR; /* B = minor */
        synth_blip(BLIP_FREQ_DOWN);
    }
    if (pressed & BTN_X) {
        s_arp_chord = CHORD_7TH; /* X = 7th */
        synth_blip(BLIP_FREQ_NEUTRAL);
    }
    if (pressed & BTN_Y) {
        s_arp_chord = CHORD_DIM; /* Y = dim */
        synth_blip(BLIP_FREQ_DOWN * 0.75f);
    }

    /* LB/RB shift octave */
    if (pressed & BTN_SHOULDER_L && s_octave > 0) {
        s_octave--;
        synth_blip(BLIP_FREQ_DOWN);
    }
    if (pressed & BTN_SHOULDER_R && s_octave < 2) {
        s_octave++;
        synth_blip(BLIP_FREQ_UP);
    }

    /* D-pad transpose root note */
    if (gp->dpad & DPAD_UP) {
        static uint32_t last_up = 0;
        uint32_t now = xTaskGetTickCount();
        if (now - last_up > pdMS_TO_TICKS(200)) {
            s_arp_root++;
            if (s_arp_root > 11)
                s_arp_root = 11;
            synth_blip(BLIP_FREQ_UP);
            last_up = now;
        }
    }
    if (gp->dpad & DPAD_DOWN) {
        static uint32_t last_down = 0;
        uint32_t now = xTaskGetTickCount();
        if (now - last_down > pdMS_TO_TICKS(200)) {
            s_arp_root--;
            if (s_arp_root < 0)
                s_arp_root = 0;
            synth_blip(BLIP_FREQ_DOWN);
            last_down = now;
        }
    }

    /* RT trigger toggles arpeggio on/off (rising edge) */
    static bool prev_r2 = false;
    bool r2 = gp->throttle > 100;
    if (r2 && !prev_r2) {
        s_arp_running = !s_arp_running;
        s_arp_index = 0;
        s_arp_direction = 1;
        synth_blip(s_arp_running ? BLIP_FREQ_UP : BLIP_FREQ_DOWN);
        if (!s_arp_running)
            tone_stop();
    }
    prev_r2 = r2;

    if (!s_arp_running) {
        led_off();
        return;
    }

    /* Speed from left stick Y: 50ms (fast) to 500ms (slow) */
    float speed_ms = map_range((float)gp->axis_y, -STICK_MAX, STICK_MAX, 50, 500);

    /* Pattern from left stick X quadrant */
    /* Left = down, right = up, center = up-down */
    int pattern = 0; /* 0=up, 1=down, 2=up-down */
    if (gp->axis_x < -STICK_DEADZONE)
        pattern = 1;
    else if (gp->axis_x > STICK_DEADZONE)
        pattern = 0;
    else
        pattern = 2;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - s_arp_last_ms >= (uint32_t)speed_ms) {
        s_arp_last_ms = now_ms;

        int semitone = (s_octave * 12) + s_arp_root + s_arp_chord[s_arp_index];
        tone_play(note_at(semitone));
        led_on();

        /* Advance index based on pattern */
        if (pattern == 0) {
            s_arp_index = (s_arp_index + 1) % CHORD_LEN;
        } else if (pattern == 1) {
            s_arp_index--;
            if (s_arp_index < 0)
                s_arp_index = CHORD_LEN - 1;
        } else {
            s_arp_index += s_arp_direction;
            if (s_arp_index >= CHORD_LEN) {
                s_arp_index = CHORD_LEN - 2;
                s_arp_direction = -1;
            } else if (s_arp_index < 0) {
                s_arp_index = 1;
                s_arp_direction = 1;
            }
        }
    }
}

/* ── Mode 6: Retro SFX ──────────────────────────────────── */

static void sfx_start(float start, float end, int ticks)
{
    s_sfx.freq = start;
    s_sfx.freq_end = end;
    s_sfx.step = (end - start) / (float)ticks;
    s_sfx.ticks_left = ticks;
}

static void sfx_tick(void)
{
    if (s_sfx.ticks_left <= 0) {
        tone_stop();
        led_off();
        return;
    }
    tone_play((uint32_t)s_sfx.freq);
    led_on();
    s_sfx.freq += s_sfx.step;
    s_sfx.ticks_left--;
}

static void mode_sfx(const gamepad_state_t *gp)
{
    /* Speed multiplier from RT trigger: 0.5x to 2x */
    float speed = map_range((float)gp->throttle, 0, TRIGGER_MAX, 1.0f, 0.3f);
    int base_ticks = 25;
    int ticks = (int)((float)base_ticks * speed);
    if (ticks < 5)
        ticks = 5;

    /* Trigger SFX on button press (rising edge) */
    static uint16_t prev_btn = 0;
    static uint8_t prev_dpad = 0;
    uint16_t btn_pressed = gp->buttons & ~prev_btn;
    uint8_t dpad_pressed = gp->dpad & ~prev_dpad;
    prev_btn = gp->buttons;
    prev_dpad = gp->dpad;

    /* A = Laser (descending sweep) */
    if (btn_pressed & BTN_A)
        sfx_start(1800, 200, ticks);

    /* B = Explosion (descending with low end) */
    if (btn_pressed & BTN_B)
        sfx_start(800, 100, ticks * 2);

    /* X = Power-up (ascending sweep) */
    if (btn_pressed & BTN_X)
        sfx_start(200, 1600, ticks);

    /* Y = Coin (high double-beep handled via short sweep) */
    if (btn_pressed & BTN_Y)
        sfx_start(1200, 1800, ticks / 2);

    /* D-pad Up = Siren (oscillating — start high, go low, we'll reverse) */
    if (dpad_pressed & DPAD_UP)
        sfx_start(400, 1200, ticks * 3);

    /* D-pad Down = Engine (low rumble) */
    if (dpad_pressed & DPAD_DOWN)
        sfx_start(100, 250, ticks * 3);

    /* D-pad Left = Jump (quick ascending chirp) */
    if (dpad_pressed & DPAD_LEFT)
        sfx_start(300, 1400, ticks / 2);

    /* D-pad Right = Warp (sweep up then down via two-phase) */
    if (dpad_pressed & DPAD_RIGHT)
        sfx_start(400, 1800, ticks);

    sfx_tick();
}

/* ── Bluepad32 Custom Platform ───────────────────────────── */

static void plat_init(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Bluepad32 platform initialized");
}

static void plat_on_init_complete(void)
{
    ESP_LOGI(TAG, "Scanning for BLE controllers...");
    uni_bt_enable_new_connections_safe(true);
}

static void plat_on_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device connected: %p", d);
}

static void plat_on_device_disconnected(uni_hid_device_t *d)
{
    ESP_LOGW(TAG, "Device disconnected: %p", d);
    gamepad_state_t empty = {0};
    memcpy((void *)&s_gp, &empty, sizeof(s_gp));
}

static uni_error_t plat_on_device_ready(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device ready: VID=0x%04X PID=0x%04X", uni_hid_device_get_vendor_id(d),
             uni_hid_device_get_product_id(d));
    s_gp.connected = true;
    return UNI_ERROR_SUCCESS;
}

static void plat_on_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    (void)d;
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
        return;
    const uni_gamepad_t *gp = &ctl->gamepad;
    s_gp.buttons = gp->buttons;
    s_gp.dpad = gp->dpad;
    s_gp.misc_buttons = gp->misc_buttons;
    s_gp.axis_x = gp->axis_x;
    s_gp.axis_y = gp->axis_y;
    s_gp.axis_rx = gp->axis_rx;
    s_gp.axis_ry = gp->axis_ry;
    s_gp.brake = gp->brake;
    s_gp.throttle = gp->throttle;
    s_gp.connected = true;
}

static int32_t plat_get_property(uni_platform_property_t key)
{
    (void)key;
    return -1;
}

static void plat_on_oob_event(uni_platform_oob_event_t event, void *data)
{
    (void)data;
    ESP_LOGD(TAG, "OOB event: %d", event);
}

static struct uni_platform s_platform = {
    .name = "gamepad_synth",
    .init = plat_init,
    .on_init_complete = plat_on_init_complete,
    .on_device_connected = plat_on_device_connected,
    .on_device_disconnected = plat_on_device_disconnected,
    .on_device_ready = plat_on_device_ready,
    .on_gamepad_data = NULL,
    .on_controller_data = plat_on_controller_data,
    .get_property = plat_get_property,
    .on_oob_event = plat_on_oob_event,
    .device_dump = NULL,
    .register_console_cmds = NULL,
};

struct uni_platform *uni_platform_custom_create(void)
{
    return &s_platform;
}

/* ── Hardware Init ───────────────────────────────────────── */

static void init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = BLOCK_SIZE;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = I2S_BCLK_PIN,
                .ws = I2S_WS_PIN,
                .dout = I2S_DOUT_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {0},
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_LOGI(TAG, "I2S initialized: BCLK=GPIO%d WS=GPIO%d DOUT=GPIO%d @ %d Hz", I2S_BCLK_PIN,
             I2S_WS_PIN, I2S_DOUT_PIN, SAMPLE_RATE);
}

static void init_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    led_off();
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/* ── Startup Jingle (blocking, runs before tasks) ────────── */

static void play_startup_jingle(void)
{
    /* C5 E5 G5 C6 — major arpeggio rendered directly via I2S */
    const uint16_t notes[] = {523, 659, 784, 1047};
    const int note_samples = SAMPLE_RATE * 120 / 1000; /* 120 ms per note */
    const int gap_samples = SAMPLE_RATE * 60 / 1000;   /* 60 ms gap */
    int16_t stereo_buf[BLOCK_SIZE * 2];

    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    for (int n = 0; n < 4; n++) {
        float phase = 0.0f;
        float phase_inc = (float)notes[n] / (float)SAMPLE_RATE;
        int remaining = note_samples;

        led_on();
        while (remaining > 0) {
            int chunk = (remaining < BLOCK_SIZE) ? remaining : BLOCK_SIZE;
            for (int i = 0; i < chunk; i++) {
                int16_t sample = (phase < 0.5f) ? AMPLITUDE : -AMPLITUDE;
                stereo_buf[i * 2] = sample;
                stereo_buf[i * 2 + 1] = sample;
                phase += phase_inc;
                if (phase >= 1.0f)
                    phase -= 1.0f;
            }
            size_t written;
            i2s_channel_write(s_tx_chan, stereo_buf, chunk * 2 * sizeof(int16_t), &written,
                              portMAX_DELAY);
            remaining -= chunk;
        }

        /* Silence gap */
        led_off();
        remaining = gap_samples;
        memset(stereo_buf, 0, sizeof(stereo_buf));
        while (remaining > 0) {
            int chunk = (remaining < BLOCK_SIZE) ? remaining : BLOCK_SIZE;
            size_t written;
            i2s_channel_write(s_tx_chan, stereo_buf, chunk * 2 * sizeof(int16_t), &written,
                              portMAX_DELAY);
            remaining -= chunk;
        }
    }

    ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
}

/* ── Audio Render Task (Core 1, high priority) ───────────── */

static void audio_render_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Audio render task started on core %d", xPortGetCoreID());

    float phase = 0.0f;
    float phase_b = 0.0f;
    float svf_low = 0.0f;
    float svf_band = 0.0f;
    float lfo_phase = 0.0f;
    float blip_phase = 0.0f;
    int16_t stereo_buf[BLOCK_SIZE * 2];

    const float block_period = (float)BLOCK_SIZE / (float)SAMPLE_RATE;

    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    while (1) {
        float freq = s_synth.target_freq;
        bool active = s_synth.active;

        if (!active || freq < (float)MIN_FREQ) {
            memset(stereo_buf, 0, sizeof(stereo_buf));
            phase = 0.0f;
            phase_b = 0.0f;
            svf_low = 0.0f;
            svf_band = 0.0f;
        } else {
            waveform_t wave = s_synth.waveform;
            bool filter_on = s_synth.filter_enabled;
            float cutoff = s_synth.filter_cutoff;
            bool osc_b_on = s_synth.osc_b_enabled;
            float freq_b = s_synth.target_freq_b;
            waveform_t wave_b = s_synth.waveform_b;

            /* Update LFO (block-rate triangle) and apply to pitch/cutoff */
            lfo_target_t lfo_tgt = s_synth.lfo_target;
            if (lfo_tgt != LFO_TARGET_NONE) {
                lfo_phase += s_synth.lfo_rate * block_period;
                if (lfo_phase >= 1.0f)
                    lfo_phase -= (float)((int)lfo_phase);
                float lfo_val = lfo_triangle(lfo_phase); /* [-1, 1] */
                float depth = s_synth.lfo_depth;
                if (lfo_tgt == LFO_TARGET_PITCH || lfo_tgt == LFO_TARGET_BOTH) {
                    /* Up to one octave pitch mod at full depth */
                    freq *= powf(2.0f, depth * lfo_val);
                }
                if (lfo_tgt == LFO_TARGET_CUTOFF || lfo_tgt == LFO_TARGET_BOTH) {
                    /* Up to two octaves cutoff mod at full depth */
                    cutoff *= powf(2.0f, 2.0f * depth * lfo_val);
                    if (cutoff < FILTER_CUTOFF_MIN)
                        cutoff = FILTER_CUTOFF_MIN;
                    if (cutoff > FILTER_CUTOFF_MAX)
                        cutoff = FILTER_CUTOFF_MAX;
                }
            }

            float phase_inc = freq / (float)SAMPLE_RATE;
            float phase_inc_b = freq_b / (float)SAMPLE_RATE;

            /* Recompute filter coefficients once per block */
            float f_coef = 2.0f * sinf(3.14159265f * cutoff / (float)SAMPLE_RATE);
            float q_coef = 1.0f / s_synth.filter_resonance;

            bool delay_on = s_synth.delay_enabled;
            int delay_samples = s_synth.delay_samples;
            float delay_fb = s_synth.delay_feedback;
            float delay_mix = s_synth.delay_mix;

            for (int i = 0; i < BLOCK_SIZE; i++) {
                int16_t sample = osc_sample(phase, wave);
                if (osc_b_on) {
                    int16_t sample_b = osc_sample(phase_b, wave_b);
                    /* Average A and B to stay within int16 range */
                    sample = (int16_t)(((int32_t)sample + (int32_t)sample_b) / 2);
                    phase_b += phase_inc_b;
                    if (phase_b >= 1.0f)
                        phase_b -= 1.0f;
                }
                if (filter_on) {
                    sample = svf_process(sample, f_coef, q_coef, &svf_low, &svf_band);
                }
                if (delay_on) {
                    sample = delay_process(sample, delay_samples, delay_fb, delay_mix);
                }
                stereo_buf[i * 2] = sample;
                stereo_buf[i * 2 + 1] = sample;
                phase += phase_inc;
                if (phase >= 1.0f)
                    phase -= 1.0f;
            }
        }

        /* Mix background drum engine into the synth output before the blip.
         * drums_render_block() performs saturating add into L/R. */
        drums_render_block(stereo_buf, BLOCK_SIZE);

        /* Mix confirmation blip over the top (post-main synthesis). The main
         * voice is ducked so the blip is audible even at full volume. */
        int blip_rem = s_blip_samples_remaining;
        if (blip_rem > 0) {
            float blip_inc = s_blip_freq_hz / (float)SAMPLE_RATE;
            int total_samples = (SAMPLE_RATE * BLIP_DURATION_MS) / 1000;
            int count = (blip_rem < BLOCK_SIZE) ? blip_rem : BLOCK_SIZE;
            for (int i = 0; i < count; i++) {
                /* Linear fade-out over the last 20% of the blip to avoid a click. */
                int pos = total_samples - blip_rem + i;
                float env = 1.0f;
                int fade_start = (total_samples * 4) / 5;
                if (pos > fade_start) {
                    env = (float)(total_samples - pos) / (float)(total_samples - fade_start);
                    if (env < 0.0f)
                        env = 0.0f;
                }
                int idx = (int)(blip_phase * SINE_TABLE_SIZE) & (SINE_TABLE_SIZE - 1);
                int16_t blip_sample = (int16_t)((float)s_sine_table[idx] * env);
                int16_t ducked = stereo_buf[i * 2] >> BLIP_MAIN_DUCK_SHIFT;
                int16_t mixed = clip_i16((float)ducked + (float)blip_sample);
                stereo_buf[i * 2] = mixed;
                stereo_buf[i * 2 + 1] = mixed;
                blip_phase += blip_inc;
                if (blip_phase >= 1.0f)
                    blip_phase -= 1.0f;
            }
            s_blip_samples_remaining = blip_rem - count;
            if (s_blip_samples_remaining <= 0)
                blip_phase = 0.0f;
        }

        size_t written;
        i2s_channel_write(s_tx_chan, stereo_buf, sizeof(stereo_buf), &written, portMAX_DELAY);
    }
}

/* ── Control Task (Core 1, reads gamepad, updates synth) ── */

static void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Control task started on core %d", xPortGetCoreID());

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        gamepad_state_t gp;
        memcpy(&gp, (const void *)&s_gp, sizeof(gp));

        if (!gp.connected) {
            tone_stop();
            piezo_voice_note_off(PIEZO_A);
            piezo_voice_note_off(PIEZO_B);
            led_off();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        /* Mode switch on Share button (rising edge) */
        uint8_t misc_pressed = gp.misc_buttons & ~s_prev_misc;
        s_prev_misc = gp.misc_buttons;

        if (misc_pressed & MISC_BACK) {
            tone_stop();
            piezo_voice_note_off(PIEZO_A);
            piezo_voice_note_off(PIEZO_B);
            s_mode = (s_mode + 1) % MODE_COUNT;
            s_arp_running = false;
            s_sfx.ticks_left = 0;

            /* Set default waveform per mode */
            static const waveform_t mode_waves[] = {
                [MODE_MONO] = WAVE_SAWTOOTH,        [MODE_DUAL_OSC] = WAVE_SAWTOOTH,
                [MODE_DELAY_SYNTH] = WAVE_SAWTOOTH, [MODE_SCALE] = WAVE_SINE,
                [MODE_ARPEGGIO] = WAVE_SQUARE,      [MODE_SFX] = WAVE_SQUARE,
                [MODE_DRONE] = WAVE_SAWTOOTH,
            };
            synth_set_waveform(mode_waves[s_mode]);

            /* Turn off osc B unless the mode uses it */
            synth_set_osc_b(false, (float)MIN_FREQ, WAVE_SAWTOOTH);

            /* Set per-mode filter defaults */
            switch (s_mode) {
                case MODE_MONO:
                case MODE_DUAL_OSC:
                case MODE_DELAY_SYNTH:
                case MODE_DRONE:
                    /* Updated dynamically from sticks each tick */
                    synth_set_filter(true, 6000.0f, 1.5f);
                    break;
                case MODE_SCALE:
                    synth_set_filter(true, 5000.0f, 0.8f);
                    break;
                case MODE_ARPEGGIO:
                    synth_set_filter(true, 4000.0f, 1.2f);
                    break;
                case MODE_SFX:
                default:
                    synth_set_filter(false, FILTER_CUTOFF_MAX, FILTER_Q_MIN);
                    break;
            }

            /* LFO: Mono (triggers) and Drone (triggers) drive it per-tick.
             * Others get it disabled. */
            if (s_mode != MODE_MONO && s_mode != MODE_DRONE) {
                synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
            }

            /* Per-mode delay preset */
            switch (s_mode) {
                case MODE_SCALE:
                    synth_set_delay(true, SAMPLE_RATE * 120 / 1000, 0.25f, 0.3f);
                    break;
                case MODE_ARPEGGIO:
                    synth_set_delay(true, SAMPLE_RATE * 250 / 1000, 0.55f, 0.4f);
                    break;
                case MODE_DELAY_SYNTH:
                    /* Starting preset — mode_delay_synth updates each tick */
                    synth_set_delay(true, SAMPLE_RATE * 200 / 1000, 0.5f, 0.5f);
                    break;
                case MODE_MONO:
                case MODE_DUAL_OSC:
                case MODE_SFX:
                case MODE_DRONE:
                default:
                    synth_set_delay(false, 1, 0.0f, 0.0f);
                    break;
            }

            ESP_LOGI(TAG, "Mode: %d waveform: %d", s_mode, mode_waves[s_mode]);
            synth_blip(BLIP_FREQ_NEUTRAL);
            led_blink(s_mode + 1, 80, 80);
        }

        /* Home button toggles background drum engine (rising edge). */
        if (misc_pressed & MISC_HOME) {
            s_drum_on = !s_drum_on;
            if (s_drum_on) {
                int pat = s_drum_pattern_remembered;
                if (pat < 1 || pat > 4)
                    pat = 1;
                drums_set_pattern(pat);
                synth_blip(BLIP_FREQ_UP);
            } else {
                /* Remember whatever pattern was active so next toggle resumes it. */
                if (s_drum_pattern_remembered < 1 || s_drum_pattern_remembered > 4)
                    s_drum_pattern_remembered = 1;
                drums_set_pattern(0);
                synth_blip(BLIP_FREQ_DOWN);
            }
            ESP_LOGI(TAG, "Drums: %s", s_drum_on ? "on" : "off");
        }

        /* TODO(settings-integration): when s_settings is merged, call
         *   drums_set_tempo(s_settings.drum_tempo_bpm);
         *   drums_set_volume(s_settings.drum_volume);
         * here so settings-page changes take effect live. For now the init
         * defaults (110 BPM, 0.5 volume) are used. */

        /* Dispatch to current mode */
        switch (s_mode) {
            case MODE_MONO:
                mode_mono(&gp);
                break;
            case MODE_DUAL_OSC:
                mode_dual_osc(&gp);
                break;
            case MODE_DELAY_SYNTH:
                mode_delay_synth(&gp);
                break;
            case MODE_SCALE:
                mode_scale(&gp);
                break;
            case MODE_ARPEGGIO:
                mode_arpeggio(&gp);
                break;
            case MODE_SFX:
                mode_sfx(&gp);
                break;
            case MODE_DRONE:
                mode_drone(&gp);
                break;
            default:
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

/* ── Entry Point ─────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Gamepad Synth starting!");

    init_nvs();
    init_sine_table();
    drums_init();
    init_i2s();
    init_led();
    piezo_voice_init(PIEZO_A, PIEZO_A_PIN);
    piezo_voice_init(PIEZO_B, PIEZO_B_PIN);
    synth_set_waveform(WAVE_SAWTOOTH);
    synth_set_filter(true, 6000.0f, 1.5f);
    synth_set_lfo(LFO_TARGET_NONE, 5.0f, 0.0f);
    synth_set_delay(false, 1, 0.0f, 0.0f);

    play_startup_jingle();

    ESP_LOGI(TAG, "Starting audio render task on Core 1 (priority 10)...");
    xTaskCreatePinnedToCore(audio_render_task, "audio", 4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG, "Starting control task on Core 1 (priority 5)...");
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Starting Bluepad32 on Core 0 (will not return)...");
    ESP_LOGI(TAG, "Put controller in pairing mode (Xbox: hold pair button on top)");
    uni_esp32_main();
}
