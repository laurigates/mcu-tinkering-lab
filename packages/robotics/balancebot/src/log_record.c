#include "log_record.h"

#include <stdio.h>
#include <string.h>

#include "crc32.h"

log_row_t log_row_from_snapshot(const balance_snapshot_t *s, uint32_t t_ms, uint16_t pack_cv)
{
    log_row_t row = {
        .t_ms = t_ms,
        .state = s->state,
        .ctrl = s->ctrl_mode,
        .pack_cv = pack_cv,
        .angle_deg = s->pitch_deg,
        .gyro_dps = s->gyro_dps,
        .setpoint_deg = s->angle_setpoint,
        .rate_sps = s->rate_cmd_sps,
        .loop_us = s->loop_us,
        .pos_m = s->pos_m,
    };
    return row;
}

size_t log_row_format_csv(const log_row_t *row, char *buf, size_t len)
{
    // The format `stream` printed before this module existed; keep it so a
    // dumped log and a USB capture read the same in analyze_capture.py.
    char pack[12];
    if (row->pack_cv == 0u) {
        strcpy(pack, "nan");
    } else {
        snprintf(pack, sizeof pack, "%u.%02u", (unsigned)(row->pack_cv / 100u),
                 (unsigned)(row->pack_cv % 100u));
    }
    int n = snprintf(buf, len, "%lu,%u,%.3f,%.2f,%.3f,%.1f,%.1f,%u,%.4f,%s\n",
                     (unsigned long)row->t_ms, (unsigned)row->state, (double)row->angle_deg,
                     (double)row->gyro_dps, (double)row->setpoint_deg, (double)row->rate_sps,
                     (double)row->loop_us, (unsigned)row->ctrl, (double)row->pos_m, pack);
    if (n < 0 || (size_t)n >= len) {
        return 0;
    }
    return (size_t)n;
}

void log_ring_init(log_ring_t *r, log_row_t *storage, uint32_t capacity)
{
    r->rows = storage;
    r->capacity = capacity;
    log_ring_clear(r);
}

void log_ring_clear(log_ring_t *r)
{
    r->head = 0;
    r->count = 0;
    r->pushed = 0;
}

void log_ring_push(log_ring_t *r, const log_row_t *row)
{
    if (r->capacity == 0) {
        return;
    }
    r->rows[r->head] = *row;
    r->head = (r->head + 1u) % r->capacity;
    if (r->count < r->capacity) {
        r->count++;
    }
    r->pushed++;
}

const log_row_t *log_ring_at(const log_ring_t *r, uint32_t i)
{
    // The oldest row sits at head once the ring has wrapped, else at 0.
    uint32_t oldest = r->count < r->capacity ? 0u : r->head;
    return &r->rows[(oldest + i) % r->capacity];
}

void log_session_init(log_session_t *s, log_row_t *storage, uint32_t capacity, uint32_t tail_rows)
{
    log_ring_init(&s->ring, storage, capacity);
    s->phase = LOG_PHASE_IDLE;
    s->end_state = BAL_STATE_IDLE;
    s->tail_rows = tail_rows;
    s->tail_left = 0;
}

static void start_run(log_session_t *s, const log_row_t *row)
{
    log_ring_clear(&s->ring);
    log_ring_push(&s->ring, row);
    s->phase = LOG_PHASE_RECORDING;
}

// The first non-RUN sample is the tail's first row.
static log_action_t end_run(log_session_t *s, const log_row_t *row)
{
    log_ring_push(&s->ring, row);
    s->end_state = row->state;
    if (s->tail_rows <= 1u) {
        s->phase = LOG_PHASE_IDLE;
        return LOG_ACTION_SAVE;
    }
    s->tail_left = s->tail_rows - 1u;
    s->phase = LOG_PHASE_TAIL;
    return LOG_ACTION_NONE;
}

log_action_t log_session_sample(log_session_t *s, const log_row_t *row)
{
    const bool run = row->state == BAL_STATE_RUN;
    switch (s->phase) {
        case LOG_PHASE_IDLE:
            if (run) {
                start_run(s, row);
            }
            return LOG_ACTION_NONE;
        case LOG_PHASE_RECORDING:
            if (run) {
                log_ring_push(&s->ring, row);
                return LOG_ACTION_NONE;
            }
            return end_run(s, row);
        case LOG_PHASE_TAIL:
            if (run) {
                // Flash cannot be written in RUN, so the run before is gone.
                start_run(s, row);
                return LOG_ACTION_LOST;
            }
            log_ring_push(&s->ring, row);
            if (--s->tail_left == 0u) {
                s->phase = LOG_PHASE_IDLE;
                return LOG_ACTION_SAVE;
            }
            return LOG_ACTION_NONE;
        default:
            s->phase = LOG_PHASE_IDLE;
            return LOG_ACTION_NONE;
    }
}

size_t log_header_encode(log_header_t *h, uint8_t *buf, size_t len)
{
    if (len < LOG_HEADER_SIZE) {
        return 0;
    }
    h->magic = LOG_MAGIC;
    h->version = LOG_VERSION;
    h->row_size = LOG_ROW_SIZE;
    memset(h->reserved, 0, sizeof h->reserved);
    h->header_crc = crc32_calc(h, offsetof(log_header_t, header_crc));
    memcpy(buf, h, LOG_HEADER_SIZE);
    return LOG_HEADER_SIZE;
}

bool log_header_decode(const uint8_t *buf, size_t len, uint32_t max_rows, log_header_t *out)
{
    if (len < LOG_HEADER_SIZE) {
        return false;
    }
    log_header_t h;
    memcpy(&h, buf, LOG_HEADER_SIZE);
    if (h.magic != LOG_MAGIC || h.version != LOG_VERSION || h.row_size != LOG_ROW_SIZE) {
        return false;
    }
    if (h.header_crc != crc32_calc(&h, offsetof(log_header_t, header_crc))) {
        return false;
    }
    if (h.row_count == 0u || h.row_count > max_rows) {
        return false;
    }
    *out = h;
    return true;
}
