/**
 * @file gemini_key_state.c
 * @brief See gemini_key_state.h.
 */

#include "gemini_key_state.h"

/* One word, written by whichever task's request returned last and read by the
 * console and self-report. A torn read is impossible for an aligned int, and a
 * stale one is a single request out of date, so no lock. */
static volatile gemini_key_state_t s_state = GEMINI_KEY_UNTESTED;

void gemini_key_state_note(int http_status)
{
    switch (http_status) {
        case 200:
            s_state = GEMINI_KEY_OK;
            break;
        case 401:
        case 403:
            s_state = GEMINI_KEY_REJECTED;
            break;
        case 402:
            s_state = GEMINI_KEY_EXHAUSTED;
            break;
        default:
            break;
    }
}

void gemini_key_state_reset(void)
{
    s_state = GEMINI_KEY_UNTESTED;
}

gemini_key_state_t gemini_key_state_get(void)
{
    return s_state;
}

const char *gemini_key_state_name(gemini_key_state_t state)
{
    switch (state) {
        case GEMINI_KEY_OK:
            return "ok";
        case GEMINI_KEY_REJECTED:
            return "rejected";
        case GEMINI_KEY_EXHAUSTED:
            return "exhausted";
        case GEMINI_KEY_UNTESTED:
        default:
            return "untested";
    }
}
