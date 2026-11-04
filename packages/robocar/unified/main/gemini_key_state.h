/**
 * @file gemini_key_state.h
 * @brief What the last Gemini response said about the API key (issue #845).
 *
 * A board with no key, a key Google rejects and a key whose prepaid balance
 * is spent all look the same from across the room: the robot goes quiet. They
 * need different fixes (`gemini auth <key>`, a new or unrestricted key, a
 * top-up), so every response's status is noted here at the one choke point,
 * gemini_http_post(), and `self_report`, `trace` and `gemini` read it back.
 *
 *   200       -> OK
 *   401, 403  -> REJECTED   (key invalid, blocked, or restricted from the API)
 *   402       -> EXHAUSTED  (prepaid credit spent)
 *   anything else, including a transport failure, leaves the state as it was:
 *   a 429 or a dropped connection says nothing about the key.
 *
 * Pure C, no ESP-IDF, so the mapping is host-tested.
 */

#ifndef GEMINI_KEY_STATE_H
#define GEMINI_KEY_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GEMINI_KEY_UNTESTED = 0, /**< No response that speaks to the key yet. */
    GEMINI_KEY_OK,           /**< The last such response was a 200. */
    GEMINI_KEY_REJECTED,     /**< 401 or 403. */
    GEMINI_KEY_EXHAUSTED,    /**< 402. */
} gemini_key_state_t;

/** Fold an HTTP status into the state. 0 (transport failure) is ignored. */
void gemini_key_state_note(int http_status);

/** Back to UNTESTED; called when the key changes. */
void gemini_key_state_reset(void);

gemini_key_state_t gemini_key_state_get(void);

/** "untested", "ok", "rejected" or "exhausted". */
const char *gemini_key_state_name(gemini_key_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* GEMINI_KEY_STATE_H */
