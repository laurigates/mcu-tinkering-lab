#include "speech_evidence.h"

/* Shared by both audio clauses: the request carries an image and no audio, so
 * the model must be told it cannot know what made the sound. Without this it
 * fills the gap with a confident invention (issue #618). */
#define SPEECH_EVIDENCE_NO_AUDIO                                                    \
    "You have only this image and NO audio, so the source of the sound is unknown " \
    "to you. Do NOT name, describe or guess what made it or where it came from. "

/* Shared by both audio-only clauses: what the model may do with a sound it
 * cannot hear. */
#define SPEECH_EVIDENCE_SOUND_ONLY_ACTION                                              \
    "If you speak about it at all, only acknowledge that something sounded different " \
    "or ask what happened. "

static const char k_scene_only[] = "The view has changed since you last spoke. ";

static const char k_audio_only[] =
    "The view has NOT changed, but the room's sound level or character has changed "
    "since you last spoke. " SPEECH_EVIDENCE_NO_AUDIO SPEECH_EVIDENCE_SOUND_ONLY_ACTION;

/* The sound changed, but the view was never compared — the scene gate is off
 * (`voice scene 0`), or the robot has not yet spoken about a frame it could
 * decode. Before issue #631 this state got "The view has changed", and "The
 * view has NOT changed" would be no truer: either asserts a comparison nobody
 * made, so the view is simply not mentioned. */
static const char k_audio_view_unknown[] =
    "The room's sound level or character has changed since you last "
    "spoke. " SPEECH_EVIDENCE_NO_AUDIO SPEECH_EVIDENCE_SOUND_ONLY_ACTION;

static const char k_both[] =
    "The view has changed, and the room's sound level or character has also changed "
    "since you last spoke. " SPEECH_EVIDENCE_NO_AUDIO "Remark on what you can see. ";

speech_sense_t speech_sense_from_gate(bool novel, bool compared)
{
    if (!compared) {
        return SPEECH_SENSE_UNKNOWN;
    }
    return novel ? SPEECH_SENSE_CHANGED : SPEECH_SENSE_SAME;
}

const char *speech_evidence_clause(speech_sense_t view, speech_sense_t sound)
{
    const bool view_changed = (view == SPEECH_SENSE_CHANGED);
    const bool sound_changed = (sound == SPEECH_SENSE_CHANGED);

    if (view_changed && sound_changed) {
        return k_both;
    }
    if (view_changed) {
        return k_scene_only;
    }
    if (sound_changed) {
        return (view == SPEECH_SENSE_SAME) ? k_audio_only : k_audio_view_unknown;
    }
    /* Nothing measured a change. Either the gate is closed, or it opened on a
     * first impression / a disabled scene gate — and neither is a comparison
     * the model may be told about. */
    return "";
}
