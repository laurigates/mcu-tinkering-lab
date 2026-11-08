/**
 * @file http_timing.h
 * @brief Phase stamps for one HTTP request: pure C, host-tested (issue #948).
 *
 * A voice turn took 8–13 s from clip to reply text (2026-11-05), and nothing
 * said whether that was the TLS handshake, the upload or the model. The caller
 * maps esp_http_client events onto these and logs the result:
 *
 *   connect   start → HTTP_EVENT_ON_CONNECTED (DNS, TCP and the TLS handshake)
 *   request   HTTP_EVENT_HEADERS_SENT → first HTTP_EVENT_ON_HEADER
 *             (the body upload, the model's time including thinking, one RTT)
 *   download  first HTTP_EVENT_ON_HEADER → HTTP_EVENT_ON_FINISH
 *   total     start → HTTP_EVENT_ON_FINISH
 *
 * Upload and model time stay summed in `request`: esp_http_client_perform()
 * writes the body between HEADERS_SENT and reading the response without
 * dispatching an event (esp_http_client.c, ESP-IDF v5.4). Splitting them needs
 * the open/write/fetch_headers API instead of perform().
 *
 * Each event keeps its first stamp, since ON_HEADER fires per header and
 * ON_DATA per chunk. A phase whose events did not both happen prints "-".
 * Durations are uint32 differences, so they survive the millisecond wrap.
 */

#ifndef HTTP_TIMING_H
#define HTTP_TIMING_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HTTP_TIMING_CONNECTED = 0,
    HTTP_TIMING_HEADERS_SENT,
    HTTP_TIMING_FIRST_HEADER,
    HTTP_TIMING_FIRST_DATA,
    HTTP_TIMING_FINISHED,
    HTTP_TIMING_EVENT_COUNT
} http_timing_event_t;

typedef struct {
    uint32_t start_ms;
    uint32_t at_ms[HTTP_TIMING_EVENT_COUNT];
    uint32_t seen; /**< bit per http_timing_event_t; 0 ms is a legal stamp */
} http_timing_t;

/** Reset @p t and stamp the request start. */
void http_timing_start(http_timing_t *t, uint32_t now_ms);

/** Stamp @p ev at @p now_ms unless it was already stamped. Unknown ids are ignored. */
void http_timing_note(http_timing_t *t, http_timing_event_t ev, uint32_t now_ms);

/**
 * @brief Write "connect=<ms> request=<ms> download=<ms> total=<ms> ms" into @p buf.
 *
 * @return snprintf's return value: the length it would have written.
 */
int http_timing_format(const http_timing_t *t, char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_TIMING_H */
