/**
 * @file voice_preroll.h
 * @brief The last second or so of accepted microphone PCM, so a hands-free voice
 *        turn starts with the words that triggered it.
 *
 * ## The problem this closes (issue #616)
 *
 * A VAD trigger fires on the onset of speech — the "Teu-" of "Teuvo, what do you
 * see?". The voice turn used to beep, settle, flush the microphone and only then
 * start recording, so everything up to and including the trigger was thrown away
 * and the model was handed the tail of a sentence or silence. It replied
 * `__IGNORE__`, and talking to the robot got no reaction.
 *
 * The ambient listener is the only code that reads every microphone frame, so it
 * is the only place that can remember them. It offers each frame here; the voice
 * turn takes the ring's contents as the start of its clip and records the rest
 * from the DMA straight after, with no flush in between, so the clip is one
 * continuous stretch of audio from before the trigger to the end of speech.
 *
 * ## What may enter
 *
 * Three cases, decided per frame by voice_preroll_offer():
 *
 *   - **Quarantined** (the amplifier is sounding, or inside the hangover after
 *     it — ambient_capture_allowed() said no): the ring is EMPTIED, not merely
 *     skipped. A frame of the robot's own voice must never reach the model, and
 *     neither may audio from before it: splicing speech from before the robot
 *     talked onto speech after it would hand the model two halves of different
 *     moments as one utterance.
 *   - **Start cue** (the voice turn's beep and its settle time): the frame is
 *     kept as SILENCE of the same length. The piezo sits next to the microphone,
 *     so a recorded beep would both be transcribed and set the clip's peak,
 *     which pins audio_clip_normalise()'s gain near 1x and leaves the speech
 *     inaudible. Silence keeps the timing honest instead; words spoken over the
 *     beep are lost, which is the cost of keeping the cue.
 *   - **Otherwise** the frame is kept as-is, displacing the oldest samples once
 *     the ring is full.
 *
 * Quarantine outranks the cue: if both apply, the ring is emptied.
 *
 * ## Taking it
 *
 * voice_preroll_take() copies out the newest samples, oldest first, and empties
 * the ring. Emptying is not tidiness: once the voice turn holds the microphone,
 * the listener stops reading, so whatever the ring still held afterwards would be
 * from before the turn and would be spliced onto the start of the next one.
 *
 * Pure C — no FreeRTOS, no ESP-IDF — and storage is caller-supplied (PSRAM on the
 * device), so test/test_voice_preroll.c builds it on the host. Not thread-safe on
 * its own: the firmware offers and takes only while holding the microphone lock
 * (mic_pdm_lock()), which is what serialises the listener against a voice turn.
 */

#ifndef VOICE_PREROLL_H
#define VOICE_PREROLL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** How much audio the ring holds, in ms at 16 kHz.
 *
 *  About one second before the trigger, plus the start cue (a 200 ms beep and a
 *  120 ms settle), plus slack for the voice-turn task waking up. Everything in
 *  the ring is taken, so the lead before the trigger is roughly this minus the
 *  cue — and it shrinks, rather than failing, if the task is late. */
#define VOICE_PREROLL_MS 1500u

/** Ring capacity in samples at the microphone's 16 kHz. */
#define VOICE_PREROLL_SAMPLES ((VOICE_PREROLL_MS * 16000u) / 1000u)

typedef struct {
    int16_t *buf; /**< Caller-supplied storage, @c cap samples. */
    size_t cap;   /**< Capacity in samples; 0 disables the ring. */
    size_t head;  /**< Index the next sample is written at. */
    size_t count; /**< Samples currently held, <= cap. */
} voice_preroll_t;

/**
 * @brief Attach storage and empty the ring.
 *
 * A NULL @p storage or a zero @p cap gives a disabled ring that accepts offers,
 * holds nothing and takes nothing — the device's behaviour when the PSRAM
 * allocation fails, which must degrade to "no pre-roll" rather than crash.
 */
void voice_preroll_init(voice_preroll_t *rb, int16_t *storage, size_t cap);

/** @brief Drop everything held. */
void voice_preroll_reset(voice_preroll_t *rb);

/**
 * @brief Offer one microphone frame. See the header note for the three cases.
 *
 * @param pcm              The frame; may be NULL only when @p capture_allowed is
 *                         false or @p cue_active is true (nothing is copied).
 * @param n                Samples in the frame.
 * @param capture_allowed  ambient_capture_allowed()'s verdict for this frame.
 * @param cue_active       The voice turn's start cue overlapped this frame.
 */
void voice_preroll_offer(voice_preroll_t *rb, const int16_t *pcm, size_t n, bool capture_allowed,
                         bool cue_active);

/** @brief Samples currently held. */
size_t voice_preroll_count(const voice_preroll_t *rb);

/**
 * @brief Copy out up to @p max of the NEWEST samples, oldest first, and empty
 *        the ring.
 *
 * When the ring holds more than @p max, the oldest are the ones dropped: the
 * newest samples are the ones contiguous with the recording that follows.
 * @p dst may be NULL with @p max 0 to discard the contents.
 *
 * @return Samples written to @p dst.
 */
size_t voice_preroll_take(voice_preroll_t *rb, int16_t *dst, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_PREROLL_H */
