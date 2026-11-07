/**
 * @file http_timing.c
 * @brief Phase stamps for one HTTP request (issue #948). See http_timing.h.
 */

#include "http_timing.h"

#include <stdio.h>
#include <string.h>

void http_timing_start(http_timing_t *t, uint32_t now_ms)
{
    memset(t, 0, sizeof(*t));
    t->start_ms = now_ms;
}

void http_timing_note(http_timing_t *t, http_timing_event_t ev, uint32_t now_ms)
{
    if ((unsigned)ev >= HTTP_TIMING_EVENT_COUNT || (t->seen & (1u << ev))) {
        return;
    }
    t->at_ms[ev] = now_ms;
    t->seen |= 1u << ev;
}

/* "<ms>" for a phase whose two ends were both stamped, "-" otherwise. A start
 * index of HTTP_TIMING_EVENT_COUNT means the request start. */
static void phase(const http_timing_t *t, int from, http_timing_event_t to, char *out, size_t len)
{
    const int from_seen = from == HTTP_TIMING_EVENT_COUNT || (t->seen & (1u << from));
    if (!from_seen || !(t->seen & (1u << to))) {
        snprintf(out, len, "-");
        return;
    }
    const uint32_t a = from == HTTP_TIMING_EVENT_COUNT ? t->start_ms : t->at_ms[from];
    snprintf(out, len, "%u", (unsigned)(uint32_t)(t->at_ms[to] - a));
}

int http_timing_format(const http_timing_t *t, char *buf, size_t len)
{
    char connect[12], request[12], download[12], total[12];
    phase(t, HTTP_TIMING_EVENT_COUNT, HTTP_TIMING_CONNECTED, connect, sizeof(connect));
    phase(t, HTTP_TIMING_HEADERS_SENT, HTTP_TIMING_FIRST_HEADER, request, sizeof(request));
    phase(t, HTTP_TIMING_FIRST_HEADER, HTTP_TIMING_FINISHED, download, sizeof(download));
    phase(t, HTTP_TIMING_EVENT_COUNT, HTTP_TIMING_FINISHED, total, sizeof(total));
    return snprintf(buf, len, "connect=%s request=%s download=%s total=%s ms", connect, request,
                    download, total);
}
