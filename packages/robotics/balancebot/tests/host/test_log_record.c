// Host unit tests for log_record.c (no Pico SDK).
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crc32.h"
#include "log_record.h"

static log_row_t row_at(uint32_t t, uint8_t state)
{
    log_row_t r = {.t_ms = t, .state = state, .angle_deg = (float)t * 0.5f};
    return r;
}

static void test_csv_matches_stream_format(void)
{
    // The line `stream` printed before log_record.c took the format over.
    log_row_t r = {.t_ms = 123456u,
                   .state = BAL_STATE_RUN,
                   .ctrl = CTRL_LQR,
                   .angle_deg = -1.23456f,
                   .gyro_dps = 12.345f,
                   .setpoint_deg = 0.5f,
                   .rate_sps = -3999.96f,
                   .loop_us = 87.25f,
                   .pos_m = 0.012345f,
                   .pack_cv = 1260};
    char buf[LOG_CSV_LINE_MAX];
    size_t n = log_row_format_csv(&r, buf, sizeof buf);
    const char *want = "123456,1,-1.235,12.35,0.500,-4000.0,87.2,1,0.0123,12.60\n";
    assert(n == strlen(want));
    assert(strcmp(buf, want) == 0);
    // No pack on D0 (pack_cv 0) is `nan`: analyze_capture.py reads every
    // column as a float, and 0.00 would pass for a flat pack.
    r.pack_cv = 0;
    n = log_row_format_csv(&r, buf, sizeof buf);
    const char *want_absent = "123456,1,-1.235,12.35,0.500,-4000.0,87.2,1,0.0123,nan\n";
    assert(n == strlen(want_absent));
    assert(strcmp(buf, want_absent) == 0);
    r.pack_cv = 1260;
    // The header has as many columns as a row.
    int commas_header = 0, commas_row = 0;
    for (const char *p = LOG_CSV_HEADER; *p; p++) {
        commas_header += *p == ',';
    }
    for (const char *p = buf; *p; p++) {
        commas_row += *p == ',';
    }
    assert(commas_header == commas_row);
    // Too short a buffer writes nothing usable and says so.
    assert(log_row_format_csv(&r, buf, 10) == 0);
}

static void test_csv_widest_row_fits(void)
{
    log_row_t r = {.t_ms = UINT32_MAX,
                   .state = 255,
                   .ctrl = 255,
                   .angle_deg = -1e30f,
                   .gyro_dps = -1e30f,
                   .setpoint_deg = -1e30f,
                   .rate_sps = -1e30f,
                   .loop_us = -1e30f,
                   .pos_m = -1e30f,
                   .pack_cv = 65535};
    char buf[512];
    size_t n = log_row_format_csv(&r, buf, sizeof buf);
    assert(n > 0);
    // Far past any real reading; LOG_CSV_LINE_MAX covers the plausible range.
    log_row_t plausible = {.t_ms = UINT32_MAX,
                           .state = 2,
                           .ctrl = 1,
                           .angle_deg = -180.0f,
                           .gyro_dps = -2000.0f,
                           .setpoint_deg = -180.0f,
                           .rate_sps = -100000.0f,
                           .loop_us = 100000.0f,
                           .pos_m = -1000.0f,
                           .pack_cv = 65535};
    char small[LOG_CSV_LINE_MAX];
    assert(log_row_format_csv(&plausible, small, sizeof small) > 0);
}

static void test_row_from_snapshot(void)
{
    balance_snapshot_t s;
    memset(&s, 0, sizeof s);
    s.state = BAL_STATE_FAULT;
    s.ctrl_mode = CTRL_PID;
    s.pitch_deg = 46.0f;
    s.gyro_dps = -3.0f;
    s.angle_setpoint = 0.25f;
    s.rate_cmd_sps = 1200.0f;
    s.loop_us = 90.0f;
    s.pos_m = 0.3f;
    log_row_t r = log_row_from_snapshot(&s, 777u, 1050u);
    assert(r.t_ms == 777u);
    assert(r.state == BAL_STATE_FAULT && r.ctrl == CTRL_PID);
    assert(r.angle_deg == 46.0f && r.gyro_dps == -3.0f && r.setpoint_deg == 0.25f);
    assert(r.rate_sps == 1200.0f && r.loop_us == 90.0f && r.pos_m == 0.3f);
    assert(r.pack_cv == 1050u);
    assert(log_row_from_snapshot(&s, 778u, 0u).pack_cv == 0u);
}

static void test_ring_order_and_wrap(void)
{
    log_row_t store[4];
    log_ring_t ring;
    log_ring_init(&ring, store, 4);
    for (uint32_t t = 1; t <= 3; t++) {
        log_row_t r = row_at(t, BAL_STATE_RUN);
        log_ring_push(&ring, &r);
    }
    assert(ring.count == 3 && ring.pushed == 3);
    assert(log_ring_at(&ring, 0)->t_ms == 1 && log_ring_at(&ring, 2)->t_ms == 3);

    for (uint32_t t = 4; t <= 10; t++) {
        log_row_t r = row_at(t, BAL_STATE_RUN);
        log_ring_push(&ring, &r);
    }
    // Keeps the newest four, oldest first.
    assert(ring.count == 4 && ring.pushed == 10);
    for (uint32_t i = 0; i < 4; i++) {
        assert(log_ring_at(&ring, i)->t_ms == 7 + i);
    }
    log_ring_clear(&ring);
    assert(ring.count == 0 && ring.pushed == 0);
}

static void test_session_records_run_and_tail(void)
{
    log_row_t store[16];
    log_session_t s;
    log_session_init(&s, store, 16, 3);
    uint32_t t = 0;

    // IDLE samples before arming are not recorded.
    for (int i = 0; i < 5; i++) {
        log_row_t r = row_at(t++, BAL_STATE_IDLE);
        assert(log_session_sample(&s, &r) == LOG_ACTION_NONE);
    }
    assert(s.ring.count == 0 && s.phase == LOG_PHASE_IDLE);

    for (int i = 0; i < 6; i++) {
        log_row_t r = row_at(t++, BAL_STATE_RUN);
        assert(log_session_sample(&s, &r) == LOG_ACTION_NONE);
    }
    assert(s.phase == LOG_PHASE_RECORDING && s.ring.count == 6);

    // Falls: the FAULT row and two more make the 3-row tail; the save comes
    // with the last of them.
    log_row_t f = row_at(t++, BAL_STATE_FAULT);
    assert(log_session_sample(&s, &f) == LOG_ACTION_NONE);
    assert(s.phase == LOG_PHASE_TAIL);
    f = row_at(t++, BAL_STATE_FAULT);
    assert(log_session_sample(&s, &f) == LOG_ACTION_NONE);
    f = row_at(t++, BAL_STATE_FAULT);
    assert(log_session_sample(&s, &f) == LOG_ACTION_SAVE);
    assert(s.phase == LOG_PHASE_IDLE && s.end_state == BAL_STATE_FAULT);
    assert(s.ring.count == 9);
    assert(log_ring_at(&s.ring, 0)->state == BAL_STATE_RUN);
    assert(log_ring_at(&s.ring, 6)->state == BAL_STATE_FAULT);

    // The finished run stays in the ring until the next one starts.
    log_row_t idle = row_at(t++, BAL_STATE_IDLE);
    assert(log_session_sample(&s, &idle) == LOG_ACTION_NONE);
    assert(s.ring.count == 9);
    log_row_t run = row_at(t++, BAL_STATE_RUN);
    assert(log_session_sample(&s, &run) == LOG_ACTION_NONE);
    assert(s.ring.count == 1 && log_ring_at(&s.ring, 0)->t_ms == run.t_ms);
}

static void test_session_disarm_ends_run(void)
{
    log_row_t store[8];
    log_session_t s;
    log_session_init(&s, store, 8, 1);
    log_row_t r = row_at(1, BAL_STATE_RUN);
    log_session_sample(&s, &r);
    // A one-row tail saves on the first non-RUN sample.
    r = row_at(2, BAL_STATE_IDLE);
    assert(log_session_sample(&s, &r) == LOG_ACTION_SAVE);
    assert(s.end_state == BAL_STATE_IDLE && s.ring.count == 2);
}

static void test_session_rearm_during_tail_loses_run(void)
{
    log_row_t store[8];
    log_session_t s;
    log_session_init(&s, store, 8, 5);
    log_row_t r = row_at(1, BAL_STATE_RUN);
    log_session_sample(&s, &r);
    r = row_at(2, BAL_STATE_IDLE);
    log_session_sample(&s, &r);
    r = row_at(3, BAL_STATE_RUN);
    assert(log_session_sample(&s, &r) == LOG_ACTION_LOST);
    // The new run records from its first row.
    assert(s.phase == LOG_PHASE_RECORDING && s.ring.count == 1);
    assert(log_ring_at(&s.ring, 0)->t_ms == 3);
}

static void test_session_long_run_keeps_newest(void)
{
    log_row_t store[4];
    log_session_t s;
    log_session_init(&s, store, 4, 2);
    for (uint32_t t = 0; t < 20; t++) {
        log_row_t r = row_at(t, BAL_STATE_RUN);
        log_session_sample(&s, &r);
    }
    log_row_t f = row_at(20, BAL_STATE_FAULT);
    log_session_sample(&s, &f);
    f = row_at(21, BAL_STATE_FAULT);
    assert(log_session_sample(&s, &f) == LOG_ACTION_SAVE);
    // The fall and the rows just before it survive; the start was dropped.
    assert(s.ring.count == 4 && s.ring.pushed == 22);
    assert(log_ring_at(&s.ring, 0)->t_ms == 18);
    assert(log_ring_at(&s.ring, 3)->t_ms == 21);
}

static log_header_t sample_header(void)
{
    log_header_t h;
    memset(&h, 0, sizeof h);
    h.row_count = 300;
    h.rate_hz = 100;
    h.overwritten = 7;
    h.save_seq = 3;
    h.end_state = BAL_STATE_FAULT;
    h.rows_crc = 0xDEADBEEFu;
    return h;
}

static void test_header_round_trip(void)
{
    uint8_t buf[64];
    log_header_t h = sample_header();
    assert(log_header_encode(&h, buf, sizeof buf) == LOG_HEADER_SIZE);
    log_header_t out;
    assert(log_header_decode(buf, sizeof buf, 1000, &out));
    assert(out.magic == LOG_MAGIC && out.version == LOG_VERSION);
    assert(out.row_size == LOG_ROW_SIZE && out.row_count == 300 && out.rate_hz == 100);
    assert(out.overwritten == 7 && out.save_seq == 3 && out.end_state == BAL_STATE_FAULT);
    assert(out.rows_crc == 0xDEADBEEFu);
    assert(log_header_encode(&h, buf, LOG_HEADER_SIZE - 1) == 0);
}

static void test_header_rejects_bad_records(void)
{
    uint8_t buf[64];
    log_header_t out;
    log_header_t h = sample_header();
    log_header_encode(&h, buf, sizeof buf);

    // Erased flash.
    uint8_t erased[64];
    memset(erased, 0xFF, sizeof erased);
    assert(!log_header_decode(erased, sizeof erased, 1000, &out));

    // Any single-bit flip in the header.
    for (size_t i = 0; i < LOG_HEADER_SIZE; i++) {
        buf[i] ^= 0x10u;
        assert(!log_header_decode(buf, sizeof buf, 1000, &out));
        buf[i] ^= 0x10u;
    }
    assert(log_header_decode(buf, sizeof buf, 1000, &out));

    // More rows than the region holds, or none.
    assert(!log_header_decode(buf, sizeof buf, 299, &out));
    h = sample_header();
    h.row_count = 0;
    log_header_encode(&h, buf, sizeof buf);
    assert(!log_header_decode(buf, sizeof buf, 1000, &out));

    // Short buffer.
    assert(!log_header_decode(buf, LOG_HEADER_SIZE - 1, 1000, &out));
}

int main(void)
{
    test_csv_matches_stream_format();
    test_csv_widest_row_fits();
    test_row_from_snapshot();
    test_ring_order_and_wrap();
    test_session_records_run_and_tail();
    test_session_disarm_ends_run();
    test_session_rearm_during_tail_loses_run();
    test_session_long_run_keeps_newest();
    test_header_round_trip();
    test_header_rejects_bad_records();
    printf("test_log_record: OK\n");
    return 0;
}
