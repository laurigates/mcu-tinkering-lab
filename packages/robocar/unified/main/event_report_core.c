/**
 * @file event_report_core.c
 * @brief Sentry-shaped event JSON and the breadcrumb ring (ADR-033).
 *
 * See event_report_core.h. Host-tested in test/test_event_report_core.c.
 */

#include "event_report_core.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"

static size_t clamp(size_t n, size_t max)
{
    return n < max ? n : max;
}

void event_crumbs_reset(event_crumbs_t *ring)
{
    if (ring == NULL) {
        return;
    }
    memset(ring, 0, sizeof(*ring));
    ring->magic = EVENT_CRUMBS_MAGIC;
}

bool event_crumbs_adopt(event_crumbs_t *ring)
{
    if (ring == NULL || ring->magic != EVENT_CRUMBS_MAGIC) {
        return false;
    }
    for (size_t i = 0; i < EVENT_CRUMBS_LINES; i++) {
        ring->lines[i][EVENT_CRUMBS_LINE_MAX - 1] = '\0';
    }
    return true;
}

/* Copy @p text into @p out, dropping ANSI CSI escapes ("ESC [ ... final") and
 * CR/LF, and turning any other control byte into '?'. Returns the length. */
static size_t clean_line(char *out, size_t cap, const char *text, size_t len)
{
    size_t o = 0;
    size_t i = 0;
    while (i < len && text[i] != '\0' && o + 1 < cap) {
        const unsigned char c = (unsigned char)text[i];
        if (c == 0x1b) {
            i++;
            if (i < len && text[i] == '[') {
                i++;
                /* Parameter and intermediate bytes, then one final byte. */
                while (i < len && text[i] != '\0' &&
                       !((unsigned char)text[i] >= 0x40 && (unsigned char)text[i] <= 0x7e)) {
                    i++;
                }
                if (i < len && text[i] != '\0') {
                    i++;
                }
            }
            continue;
        }
        if (c == '\r' || c == '\n') {
            i++;
            continue;
        }
        out[o++] = (c < 0x20 || c == 0x7f) ? '?' : (char)c;
        i++;
    }
    out[o] = '\0';
    return o;
}

bool event_crumbs_write(event_crumbs_t *ring, uint32_t seq, const char *text, size_t len)
{
    if (ring == NULL || text == NULL) {
        return false;
    }
    const size_t idx = seq % EVENT_CRUMBS_LINES;
    ring->seqs[idx] = seq;
    /* An empty result leaves the slot empty, which collect() skips. */
    return clean_line(ring->lines[idx], EVENT_CRUMBS_LINE_MAX, text, len) > 0;
}

size_t event_crumbs_collect(const event_crumbs_t *ring, const char **out, size_t max)
{
    if (ring == NULL || out == NULL || max == 0) {
        return 0;
    }
    /* Order the occupied slots by sequence number (insertion sort over at most
     * EVENT_CRUMBS_LINES entries). Differences from the first one are compared
     * as signed, so the order survives the counter wrapping. */
    size_t order[EVENT_CRUMBS_LINES];
    size_t count = 0;
    for (size_t i = 0; i < EVENT_CRUMBS_LINES; i++) {
        if (ring->lines[i][0] == '\0') {
            continue;
        }
        size_t j = count++;
        while (j > 0 && (int32_t)(ring->seqs[order[j - 1]] - ring->seqs[i]) > 0) {
            order[j] = order[j - 1];
            j--;
        }
        order[j] = i;
    }
    const size_t take = clamp(count, max);
    for (size_t k = 0; k < take; k++) {
        out[k] = ring->lines[order[count - take + k]];
    }
    return take;
}

bool event_report_reset_is_reportable(const char *reason_name)
{
    if (reason_name == NULL) {
        return true;
    }
    static const char *const k_deliberate[] = {
        "POWERON", "SW", "EXT", "USB", "JTAG", "DEEPSLEEP",
    };
    for (size_t i = 0; i < sizeof(k_deliberate) / sizeof(k_deliberate[0]); i++) {
        if (strcmp(reason_name, k_deliberate[i]) == 0) {
            return false;
        }
    }
    return true;
}

void event_report_format_id(const uint32_t words[4], char out[EVENT_REPORT_ID_LEN + 1])
{
    snprintf(out, EVENT_REPORT_ID_LEN + 1, "%08x%08x%08x%08x", (unsigned)words[0],
             (unsigned)words[1], (unsigned)words[2], (unsigned)words[3]);
}

static bool add_exception(cJSON *root, const event_report_t *ev)
{
    cJSON *exc = cJSON_AddObjectToObject(root, "exception");
    if (exc == NULL || cJSON_AddStringToObject(exc, "type", ev->exception_type) == NULL ||
        cJSON_AddStringToObject(exc, "value", ev->exception_value ? ev->exception_value : "") ==
            NULL) {
        return false;
    }
    cJSON *frames = cJSON_AddArrayToObject(exc, "frames");
    if (frames == NULL) {
        return false;
    }
    const size_t count = clamp(ev->frame_count, EVENT_REPORT_FRAMES_MAX);
    for (size_t i = 0; i < count; i++) {
        char addr[11];
        snprintf(addr, sizeof(addr), "0x%08x", (unsigned)ev->frames[i]);
        cJSON *frame = cJSON_CreateObject();
        if (frame == NULL || cJSON_AddStringToObject(frame, "instruction_addr", addr) == NULL) {
            cJSON_Delete(frame);
            return false;
        }
        cJSON_AddItemToArray(frames, frame);
    }
    return true;
}

static bool add_body(cJSON *root, const event_report_t *ev)
{
    if (cJSON_AddStringToObject(root, "event_id", ev->event_id) == NULL ||
        cJSON_AddNumberToObject(root, "timestamp", (double)ev->timestamp) == NULL ||
        cJSON_AddStringToObject(root, "level", ev->level) == NULL ||
        cJSON_AddStringToObject(root, "source", ev->source) == NULL ||
        cJSON_AddStringToObject(root, "release", ev->release) == NULL ||
        cJSON_AddStringToObject(root, "message", ev->message) == NULL) {
        return false;
    }

    cJSON *fp = cJSON_AddArrayToObject(root, "fingerprint");
    if (fp == NULL) {
        return false;
    }
    for (size_t i = 0; i < clamp(ev->fingerprint_count, EVENT_REPORT_FINGERPRINT_MAX); i++) {
        if (ev->fingerprint[i] != NULL) {
            cJSON_AddItemToArray(fp, cJSON_CreateString(ev->fingerprint[i]));
        }
    }

    cJSON *tags = cJSON_AddObjectToObject(root, "tags");
    if (tags == NULL) {
        return false;
    }
    for (size_t i = 0; i < clamp(ev->tag_count, EVENT_REPORT_TAGS_MAX); i++) {
        if (ev->tags[i].key != NULL && ev->tags[i].value != NULL &&
            cJSON_AddStringToObject(tags, ev->tags[i].key, ev->tags[i].value) == NULL) {
            return false;
        }
    }

    cJSON *extra = cJSON_AddObjectToObject(root, "extra");
    if (extra == NULL) {
        return false;
    }
    for (size_t i = 0; i < clamp(ev->extra_count, EVENT_REPORT_EXTRAS_MAX); i++) {
        if (ev->extras[i].key != NULL &&
            cJSON_AddNumberToObject(extra, ev->extras[i].key, (double)ev->extras[i].value) ==
                NULL) {
            return false;
        }
    }

    cJSON *crumbs = cJSON_AddArrayToObject(root, "breadcrumbs");
    if (crumbs == NULL) {
        return false;
    }
    for (size_t i = 0; ev->breadcrumbs != NULL && i < ev->breadcrumb_count; i++) {
        if (ev->breadcrumbs[i] == NULL) {
            continue;
        }
        cJSON *crumb = cJSON_CreateObject();
        if (crumb == NULL ||
            cJSON_AddStringToObject(crumb, "message", ev->breadcrumbs[i]) == NULL ||
            cJSON_AddStringToObject(crumb, "category",
                                    ev->breadcrumb_category != NULL ? ev->breadcrumb_category
                                                                    : "log") == NULL) {
            cJSON_Delete(crumb);
            return false;
        }
        cJSON_AddItemToArray(crumbs, crumb);
    }

    return ev->exception_type == NULL || add_exception(root, ev);
}

char *event_report_render(const event_report_t *ev)
{
    if (ev == NULL || ev->event_id == NULL || ev->level == NULL || ev->source == NULL ||
        ev->release == NULL || ev->message == NULL) {
        return NULL;
    }
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }
    char *json = add_body(root, ev) ? cJSON_PrintUnformatted(root) : NULL;
    cJSON_Delete(root);
    return json;
}
