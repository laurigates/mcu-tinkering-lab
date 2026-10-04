#include "speech_evidence.h"

/* Shared by both audio clauses: the request carries an image and no audio, so
 * the model must be told it cannot know what made the sound. Without this it
 * fills the gap with a confident invention (issue #618). */
#define SPEECH_EVIDENCE_NO_AUDIO                                                    \
    "You have only this image and NO audio, so the source of the sound is unknown " \
    "to you. Do NOT name, describe or guess what made it or where it came from. "

static const char k_scene_only[] = "The view has changed since you last spoke. ";

static const char k_audio_only[] =
    "The view has NOT changed, but the room's sound level or character has changed "
    "since you last spoke. " SPEECH_EVIDENCE_NO_AUDIO
    "If you speak about it at all, only acknowledge that something sounded different "
    "or ask what happened. ";

static const char k_both[] =
    "The view has changed, and the room's sound level or character has also changed "
    "since you last spoke. " SPEECH_EVIDENCE_NO_AUDIO "Remark on what you can see. ";

const char *speech_evidence_clause(bool scene_novel, bool audio_novel)
{
    if (scene_novel && audio_novel) {
        return k_both;
    }
    if (scene_novel) {
        return k_scene_only;
    }
    if (audio_novel) {
        return k_audio_only;
    }
    return "";
}
