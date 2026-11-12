/**
 * @file ota_announce.c
 * @brief Pre-OTA announcement text and cancel window. See ota_announce.h.
 */

#include "ota_announce.h"

#include <stdio.h>
#include <string.h>

static const char k_not_listed[] = "not listed";

size_t ota_announce_facts(const char *new_version, const char *running_version,
                          const char *const *notes, size_t n_notes, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return 0;
    }
    out[0] = '\0';
    if (new_version == NULL || running_version == NULL) {
        return 0;
    }

    const int head = snprintf(
        out, out_len, "Firmware version %s is available (running %s). Changes: ", new_version,
        running_version);
    /* Room for the clause, the shortest ending ("not listed.") and the NUL. */
    if (head < 0 || (size_t)head + sizeof(k_not_listed) + 1 > out_len) {
        out[0] = '\0';
        return 0;
    }
    size_t pos = (size_t)head;
    size_t added = 0;

    for (size_t i = 0; i < n_notes && notes != NULL; ++i) {
        const char *note = notes[i];
        if (note == NULL || note[0] == '\0') {
            continue;
        }
        const size_t sep = added ? 2 : 0; /* "; " */
        const size_t need = sep + strlen(note) + 1 /* "." */ + 1 /* NUL */;
        if (pos + need > out_len) {
            continue; /* whole or not at all; a shorter later note may still fit */
        }
        if (sep) {
            memcpy(out + pos, "; ", 2);
            pos += 2;
        }
        memcpy(out + pos, note, strlen(note));
        pos += strlen(note);
        ++added;
    }

    if (added == 0) {
        memcpy(out + pos, k_not_listed, sizeof(k_not_listed) - 1);
        pos += sizeof(k_not_listed) - 1;
    }
    out[pos++] = '.';
    out[pos] = '\0';
    return pos;
}

size_t ota_announce_fallback(const char *new_version, bool finnish, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return 0;
    }
    const char *v = new_version ? new_version : "?";
    const int n = finnish ? snprintf(out, out_len, "Minulle on tarjolla päivitys, versio %s.", v)
                          : snprintf(out, out_len, "There is an update for me, version %s.", v);
    if (n < 0) {
        out[0] = '\0';
        return 0;
    }
    return ((size_t)n < out_len) ? (size_t)n : out_len - 1;
}

void ota_window_open(ota_window_t *w, uint32_t now_ms, uint32_t length_ms)
{
    w->opened_ms = now_ms;
    w->length_ms = length_ms;
    w->postponed = false;
    w->open = true;
}

ota_window_state_t ota_window_state(const ota_window_t *w, uint32_t now_ms)
{
    if (w->postponed) {
        return OTA_WINDOW_POSTPONED;
    }
    if (!w->open || (uint32_t)(now_ms - w->opened_ms) >= w->length_ms) {
        return OTA_WINDOW_EXPIRED;
    }
    return OTA_WINDOW_OPEN;
}

bool ota_window_postpone(ota_window_t *w, uint32_t now_ms)
{
    if (ota_window_state(w, now_ms) != OTA_WINDOW_OPEN) {
        return false;
    }
    w->postponed = true;
    return true;
}
