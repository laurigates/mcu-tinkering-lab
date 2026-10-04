/**
 * @file voice_history.h
 * @brief Rolling multi-turn conversational memory and prompt formatting.
 *
 * Maintains a rolling history of recent voice turns (up to
 * VOICE_HISTORY_MAX_TURNS) with an idle expiration timeout
 * (VOICE_HISTORY_IDLE_TIMEOUT_MS). Formats multi-turn contents and multimodal
 * parts for the Gemini API request body.
 *
 * Pure C by design — no FreeRTOS, no ESP-IDF dependencies — so host tests
 * (test/test_voice_history.c) compile and run without shims.
 */

#ifndef VOICE_HISTORY_H
#define VOICE_HISTORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"
#include "reactive_controller.h"
#include "speech_queue.h"

#define VOICE_HISTORY_MAX_TURNS 4
#define VOICE_HISTORY_IDLE_TIMEOUT_MS 60000
#define VOICE_HISTORY_MAX_OUTPUT_TOKENS 2048
#define VOICE_HISTORY_CONVERSATION_WINDOW_MS 7000
#define VOICE_HISTORY_IGNORE_TAG "__IGNORE__"

/** Bytes the request-body buffer reserves beyond the two base64 payloads, for
 *  the JSON skeleton: keys, the system prompt, the current prompt and the
 *  history replies. The largest skeleton the firmware can build (a full history
 *  ring of SPEECH_TEXT_MAX replies, a filled 1536-byte system prompt) prints to
 *  ~3.7 kB unescaped; this is twice that and more, so escaped quotes and
 *  newlines still fit. test_request_body_peak_is_one_copy checks that the
 *  unescaped worst case fits without the print buffer growing; the escaped case
 *  is not tested, and overflowing the headroom costs peak memory, not
 *  correctness (cJSON grows the buffer). */
#define VOICE_HISTORY_BODY_HEADROOM (8 * 1024)

typedef struct {
    char reply[SPEECH_TEXT_MAX];
} voice_history_entry_t;

/**
 * @brief Check whether a model reply is an ignore sentinel.
 *
 * Returns true if reply is NULL, empty, or begins with "__IGNORE__"
 * (after skipping leading whitespace).
 *
 * @param reply Model reply string.
 * @return True if reply should be ignored, false otherwise.
 */
bool voice_history_is_ignore(const char *reply);

/**
 * @brief Record that a spoken reply was delivered, opening the active conversation window.
 *
 * Sets the active conversation deadline to now_ms + VOICE_HISTORY_CONVERSATION_WINDOW_MS.
 *
 * @param now_ms Current timestamp in milliseconds.
 */
void voice_history_mark_conversation_active(uint32_t now_ms);

/**
 * @brief Check whether the active conversation window is currently open.
 *
 * @param now_ms Current timestamp in milliseconds.
 * @return True if within VOICE_HISTORY_CONVERSATION_WINDOW_MS of the last spoken reply.
 */
bool voice_history_in_conversation(uint32_t now_ms);

/**
 * @brief Reset the conversational history.
 *
 * Clears all recorded past turns and resets the idle timer.
 */
void voice_history_reset(void);

/**
 * @brief Record a model reply in the rolling history buffer.
 *
 * If the buffer is full (VOICE_HISTORY_MAX_TURNS), the oldest turn is evicted.
 * Updates the timestamp of the most recent interaction.
 *
 * @param reply Model reply string (NUL-terminated).
 * @param now_ms Current timestamp in milliseconds.
 */
void voice_history_record(const char *reply, uint32_t now_ms);

/**
 * @brief Expire history if idle timeout has elapsed.
 *
 * If more than VOICE_HISTORY_IDLE_TIMEOUT_MS has elapsed since the last turn,
 * clears all past history. Safe against 32-bit unsigned millisecond wrap.
 *
 * @param now_ms Current timestamp in milliseconds.
 */
void voice_history_expire(uint32_t now_ms);

/**
 * @brief Return count of active past turns, after applying idle expiration.
 *
 * @param now_ms Current timestamp in milliseconds.
 * @return Number of active past turns (0 .. VOICE_HISTORY_MAX_TURNS).
 */
size_t voice_history_count(uint32_t now_ms);

/**
 * @brief Retrieve a past turn's reply by chronological index.
 *
 * Index 0 is the oldest valid turn, count-1 is the newest.
 *
 * @param index Chronological index (0 .. count-1).
 * @param now_ms Current timestamp in milliseconds.
 * @return Pointer to reply string, or NULL if index is out of bounds or expired.
 */
const char *voice_history_get_reply(size_t index, uint32_t now_ms);

/**
 * @brief Format physical telemetry into a concise prompt summary string.
 *
 * Examples:
 *   - "Physical state: distance to obstacle ahead is 45 cm."
 *   - "Physical state: distance to obstacle ahead is 8 cm (obstacle reflex active)."
 *   - "Physical state: ultrasonic sensor unavailable."
 *
 * @param tele Pointer to telemetry snapshot (may be NULL).
 * @param out Destination buffer.
 * @param out_size Size of destination buffer.
 * @return True if a summary was formatted, false if tele is NULL or buffer invalid.
 */
bool voice_history_format_telemetry(const reactive_telemetry_t *tele, char *out, size_t out_size);

/**
 * @brief Build the current turn's user prompt text.
 *
 * Combines robot identity, audio/camera indication, and optional telemetry.
 *
 * @param name Robot name (e.g. persona name, or NULL for "Robocar").
 * @param has_image True if camera JPEG is attached to the turn.
 * @param tele_summary Formatted telemetry string (or NULL/empty).
 * @param out Destination buffer.
 * @param out_size Size of destination buffer.
 */
void voice_history_build_prompt(const char *name, bool has_image, const char *tele_summary,
                                char *out, size_t out_size);

/**
 * @brief Populate Gemini `contents` array with past turns and current turn.
 *
 * Past turns alternate:
 *   user:  [{ "text": "(User spoke to you)" }]
 *   model: [{ "text": history[i].reply }]
 *
 * Current turn:
 *   user:  [
 *            { "text": prompt_text },
 *            { "inlineData": { "mimeType": "image/jpeg", "data": b64_jpeg } }, // if b64_jpeg !=
 * NULL { "inlineData": { "mimeType": "audio/wav", "data": b64_wav } }
 *          ]
 *
 * @param contents cJSON array object to append turns to.
 * @param prompt_text Prompt text for the current user turn.
 * @param b64_jpeg Optional base64 JPEG data (NULL to omit).
 * @param b64_wav Base64 audio WAV data.
 * @param now_ms Current timestamp in milliseconds.
 * @return True on success, false on allocation failure.
 *
 * The two `data` strings are attached BY REFERENCE, not copied: @p b64_jpeg and
 * @p b64_wav must outlive the tree, and deleting the tree does not free them.
 * Copying would put a second 341 kB clip in PSRAM beside the caller's (#625).
 */
bool voice_history_build_contents(cJSON *contents, const char *prompt_text, const char *b64_jpeg,
                                  const char *b64_wav, uint32_t now_ms);

/**
 * @brief Build complete unformatted JSON request body for Gemini API.
 *
 * Caller is responsible for free()ing the returned string.
 *
 * Memory: the only large allocation is the returned body, presized to both
 * payloads plus VOICE_HISTORY_BODY_HEADROOM, so the peak while building is the
 * caller's two base64 strings plus one body — not the ~2x that
 * cJSON_PrintUnformatted()'s doubling growth and a copied tree used to cost.
 *
 * @param name Robot persona name (NULL for "Robocar").
 * @param sys_prompt System instruction prompt text (NULL for "Be brief.").
 * @param tele Optional reactive controller telemetry (NULL if unavailable).
 * @param has_telemetry True if tele is valid.
 * @param b64_jpeg Optional base64 JPEG data (NULL to omit).
 * @param b64_wav Base64 audio WAV data.
 * @param now_ms Current timestamp in milliseconds.
 * @return Heap-allocated unformatted JSON string (caller frees), or NULL on
 *         allocation failure or when @p b64_wav is NULL.
 */
char *voice_history_build_request_body(const char *name, const char *sys_prompt,
                                       const reactive_telemetry_t *tele, bool has_telemetry,
                                       const char *b64_jpeg, const char *b64_wav, uint32_t now_ms);

#endif  // VOICE_HISTORY_H
