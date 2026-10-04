/**
 * @file speech_evidence.h
 * @brief The prompt clause that tells the planner WHY it may speak this cycle.
 *
 * The planner request is stateless and carries exactly one input besides the
 * text: the camera JPEG. It carries no audio. So when the speech gate opens, the
 * model cannot see the gate that opened it, and it has to be told — but only
 * what the robot actually knows.
 *
 * This clause has already failed in two opposite directions:
 *
 *  1. Missing entirely. The old wording said "do so only if THIS FRAME shows
 *     something worth remarking on". On an audio-only opening the frame shows
 *     nothing new by construction, so the model was told to stay silent in
 *     exactly the case the gate had opened for.
 *
 *  2. Over-asserting (issue #618). The fix for (1) said "something happened out
 *     of frame or behind you. Remark on that". The model has no audio, so the
 *     only thing it could remark on was something it invented — and it invented
 *     fluently: bangs, echoes, moving furniture. The gate's boolean had become
 *     an assertion of fact the model had no channel to doubt
 *     (.claude/rules/stateless-model-gating.md §4).
 *
 * So the audio clauses state only what the ambient detector measured — the
 * room's sound level or character changed — say explicitly that the model has
 * no audio and does not know the source, allow at most a non-specific
 * acknowledgement or question, and forbid naming or describing a source.
 *
 *  3. Asserting a comparison nobody made (issue #631). Every clause says "since
 *     you last spoke", but both gates also answer novel when they compared
 *     nothing: before the robot has spoken about any frame or room (the first
 *     impression), and the scene gate with its threshold at 0 (`voice scene
 *     0`). A boolean cannot tell those apart from a measured change, so each
 *     sense now arrives as a speech_sense_t, and a sense that was not compared
 *     contributes no sentence at all — neither "has changed" nor "has NOT
 *     changed".
 *
 * Pure C with no ESP-IDF dependency, so the wording is pinned by
 * test/test_speech_evidence.c rather than by a bench that cannot stage it.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What one sense can truthfully be said to have measured this cycle. */
typedef enum {
    /** No comparison was made: nothing measured, nothing spoken about yet, or
     *  the gate disabled. Nothing may be said about this sense. */
    SPEECH_SENSE_UNKNOWN = 0,
    /** Compared against the last spoken-about reference; below threshold. */
    SPEECH_SENSE_SAME,
    /** Compared against the last spoken-about reference; at or over threshold. */
    SPEECH_SENSE_CHANGED,
} speech_sense_t;

/**
 * @brief Classify one gate's verdict.
 *
 * @param novel    the gate's novel() answer this cycle.
 * @param compared the gate's compared() answer this cycle
 *                 (scene_change_compared() / ambient_audio_compared()).
 */
speech_sense_t speech_sense_from_gate(bool novel, bool compared);

/**
 * @brief The evidence clause for the speak prompt.
 *
 * @param view  the scene gate, via speech_sense_from_gate().
 * @param sound the ambient gate, via speech_sense_from_gate().
 * @return a static, NUL-terminated clause ending in a space; the empty string
 *         when neither sense measured a change, so no evidence is ever asserted
 *         that was not measured. A view that was not compared is left out of
 *         the sound clause rather than stated as unchanged.
 */
const char *speech_evidence_clause(speech_sense_t view, speech_sense_t sound);

#ifdef __cplusplus
}
#endif
