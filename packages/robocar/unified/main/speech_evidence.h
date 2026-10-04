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
 * Pure C with no ESP-IDF dependency, so the wording is pinned by
 * test/test_speech_evidence.c rather than by a bench that cannot stage it.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The evidence clause for the speak prompt.
 *
 * @param scene_novel scene_change_novel() for this cycle.
 * @param audio_novel ambient_audio_novel() for this cycle.
 * @return a static, NUL-terminated clause ending in a space; the empty string
 *         when neither sense reported a change, so no evidence is ever asserted
 *         that was not measured.
 */
const char *speech_evidence_clause(bool scene_novel, bool audio_novel);

#ifdef __cplusplus
}
#endif
